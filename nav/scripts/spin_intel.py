#!/usr/bin/env python3
import os, time, math
import rospy, cv2, numpy as np, pyrealsense2 as rs
from ultralytics import YOLO
from std_msgs.msg import String
from geometry_msgs.msg import Twist

def clamp(x, lo, hi): return lo if x < lo else hi if x > hi else x

class YoloSpinRealSense:
    def __init__(self):
        rospy.init_node("yolo_spin_rs_simple")

        # ---- params (minimal) ----
        self.model_path    = rospy.get_param("~model", "yolo12n.pt")
        self.min_conf      = rospy.get_param("~min_conf", 0.60)
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/mobile_base/commands/velocity")
        self.scan_speed    = rospy.get_param("~scan_speed", 0.35)  # left = +z
        self.align_kp      = rospy.get_param("~align_kp", 1.0)
        self.align_w_max   = rospy.get_param("~align_w_max", 0.6)
        self.deadband_deg  = rospy.get_param("~deadband_deg", 4.0)
        self.stable_frames = rospy.get_param("~stable_frames", 8)
        self.imgsz         = rospy.get_param("~imgsz", 640)
        self.show_viz      = rospy.get_param("~show_viz", True)

        if not os.environ.get("DISPLAY"):
            self.show_viz = False

        # ---- pubs/subs ----
        self.vel_pub    = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=1)
        self.status_pub = rospy.Publisher("/rotator/status", String, queue_size=1)
        rospy.Subscriber("/rotator/control", String, self._on_ctrl)

        # ---- state ----
        self.mode = "idle"               # idle | scanning | aligning | locked
        self.prev_x1 = None              # saved x1 after lock
        self.mask_min_x = None           # active mask: [mask_min_x, width) is masked
        self.in_continue = False         # whether current cycle started with "continue"
        self.align_stable = 0
        self.last_bbox = None            # track bbox during align

        # ---- RealSense ----
        self.pipeline = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        prof = self.pipeline.start(cfg)
        col_stream = prof.get_stream(rs.stream.color)
        intr = col_stream.as_video_stream_profile().get_intrinsics()
        self.fx, self.cx = intr.fx, intr.ppx
        self.image_width, self.image_height = intr.width, intr.height

        # ---- YOLO ----
        try:
            import torch
            self.device = 0 if torch.cuda.is_available() else "cpu"
        except Exception:
            self.device = "cpu"
        self.model = YOLO(self.model_path)

        rospy.loginfo("Ready. Send 'start' or 'continue' to /rotator/control")

        # ---- main loop ----
        self._loop()

    # ---------- control ----------
    def _on_ctrl(self, msg: String):
        cmd = msg.data.strip().lower()
        if cmd == "start":
            # START: clear bbox & mask, scan left
            self.prev_x1 = None
            self.mask_min_x = None
            self.in_continue = False
            self._set_mode_scanning()
        elif cmd == "continue":
            # CONTINUE: mask from saved x1 to right edge (if available)
            self.in_continue = True
            if self.prev_x1 is not None:
                self.mask_min_x = int(self.prev_x1)
            else:
                self.mask_min_x = None  # no saved x1 → scan full image
            self._set_mode_scanning()
        elif cmd == "stop":
            self._stop()
        else:
            rospy.logwarn("Unknown command: %s", cmd)

    # ---------- helpers ----------
    def _stop(self):
        self.mode = "idle"
        self._send_w(0.0)
        self.status_pub.publish("idle")

    def _set_mode_scanning(self):
        self.mode = "scanning"
        self.align_stable = 0
        self.last_bbox = None
        self.status_pub.publish("scanning")

    def _send_w(self, w):
        tw = Twist()
        tw.angular.z = float(w)
        self.vel_pub.publish(tw)

    def _mask_image(self, img):
        """Mask columns [mask_min_x, width) to black."""
        if self.mask_min_x is None:
            return img
        m = img.copy()
        x = max(0, min(self.image_width, int(self.mask_min_x)))
        m[:, x:self.image_width] = 0
        return m

    def _draw_viz(self, img, bbox, yaw, locked=False):
        if not self.show_viz:
            return
        disp = img.copy()
        # center line
        cv2.line(disp, (int(self.cx), 0), (int(self.cx), self.image_height-1), (255,255,255), 1)
        # mask boundary
        if self.mask_min_x is not None:
            x = max(0, min(self.image_width-1, int(self.mask_min_x)))
            cv2.line(disp, (x, 0), (x, self.image_height-1), (0,0,255), 2)
            cv2.putText(disp, "MASK ->", (x+4, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)
        # bbox & yaw
        if bbox is not None:
            x1,y1,x2,y2 = bbox
            cv2.rectangle(disp, (x1,y1), (x2,y2), (0,255,0), 2)
            if yaw is not None:
                cv2.putText(disp, f"{math.degrees(yaw):.1f} deg", (x1, max(0,y1-6)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)
        if locked:
            cv2.putText(disp, "LOCKED", (10, self.image_height-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,200,0), 2)
        cv2.imshow("YOLO Spin (RS)", disp)
        cv2.waitKey(1)

    # ---------- main loop ----------
    def _loop(self):
        r = rospy.Rate(15)
        deadband = math.radians(self.deadband_deg)

        while not rospy.is_shutdown():
            # grab frame
            try:
                frames = self.pipeline.wait_for_frames()
                color_frame = frames.get_color_frame()
                if not color_frame:
                    r.sleep(); continue
                img = np.asanyarray(color_frame.get_data())
            except Exception as e:
                rospy.logwarn("RealSense error: %s", e)
                r.sleep(); continue

            # choose image for detection (apply mask in CONTINUE)
            det_img = self._mask_image(img) if (self.mask_min_x is not None) else img

            # YOLO inference
            try:
                results = self.model(det_img, imgsz=self.imgsz, verbose=False, device=self.device)
            except TypeError:
                results = self.model(det_img, imgsz=self.imgsz, verbose=False)

            # parse detections: pick person (cls==0) with smallest |yaw|
            yaw_best, box_best = None, None
            for res in results:
                if not getattr(res, "boxes", None): continue
                for b in res.boxes:
                    cls_id = int(b.cls[0].item()) if b.cls is not None else -1
                    conf   = float(b.conf[0].item()) if b.conf is not None else 0.0
                    if cls_id != 0 or conf < self.min_conf:  # 0 = person
                        continue
                    x1,y1,x2,y2 = map(int, b.xyxy[0].tolist())
                    u = 0.5*(x1+x2)
                    yaw = math.atan2((u - self.cx)/self.fx, 1.0)
                    if (yaw_best is None) or (abs(yaw) < abs(yaw_best)):
                        yaw_best, box_best = yaw, (x1,y1,x2,y2)

            # STATE MACHINE
            if self.mode == "scanning":
                # spin left continuously
                self._send_w(self.scan_speed)
                if box_best is not None:
                    # ALIGN with this bbox
                    self.mode = "aligning"
                    self.status_pub.publish("aligning")
                    self.align_stable = 0
                    self.last_bbox = box_best

                # viz
                self._draw_viz(img, box_best, yaw_best)
                r.sleep(); continue

            if self.mode == "aligning":
                if box_best is None:
                    # lost → back to scanning (keep mask as set by start/continue)
                    self._set_mode_scanning()
                    self._draw_viz(img, None, None)
                    r.sleep(); continue

                # update last bbox & yaw
                self.last_bbox = box_best
                x1,y1,x2,y2 = box_best

                # CONTINUE rule: while aligning, save x2 and, if x2 >= prev_x1, move mask_min_x to x2
                if self.in_continue and (self.prev_x1 is not None) and (x2 >= int(self.prev_x1)):
                    self.mask_min_x = int(x2) + 10

                # PD-less P-control
                w = clamp(-self.align_kp * yaw_best, -self.align_w_max, self.align_w_max)
                self._send_w(w)

                # lock when stabilized
                if abs(yaw_best) <= deadband:
                    self.align_stable += 1
                else:
                    self.align_stable = 0

                if self.align_stable >= self.stable_frames:
                    # LOCK
                    self._send_w(0.0)
                    self.mode = "locked"
                    self.status_pub.publish("person_detected")
                    rospy.logwarn("Person detected")
                    # save x1 of bbox for next continue
                    self.prev_x1 = int(x1)
                    # after lock, wait for next "continue" (or "start")
                    self._draw_viz(img, box_best, yaw_best, locked=True)
                    r.sleep(); continue

                # viz
                self._draw_viz(img, box_best, yaw_best)
                r.sleep(); continue

            if self.mode == "locked":
                # hold still, wait for /rotator/control
                self._send_w(0.0)
                self._draw_viz(img, self.last_bbox, 0.0, locked=True)
                r.sleep(); continue

            # idle
            self._send_w(0.0)
            r.sleep()

    def __del__(self):
        try:
            self.pipeline.stop()
        except:  # noqa: E722
            pass

if __name__ == "__main__":
    try:
        YoloSpinRealSense()
    except rospy.ROSInterruptException:
        pass
