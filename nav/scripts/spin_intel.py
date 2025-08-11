#!/usr/bin/env python3
import os, time, math
import rospy, cv2, numpy as np, pyrealsense2 as rs
from ultralytics import YOLO
from std_msgs.msg import String
from geometry_msgs.msg import Twist

def clamp(x, lo, hi): return lo if x < lo else hi if x > hi else x

class YoloSpinRealSense:
    def __init__(self):
        rospy.init_node("yolo_spin_rs_depth_filtered")

        # ---- params ----
        self.model_path    = rospy.get_param("~model", "yolo12n.pt")
        self.min_conf      = rospy.get_param("~min_conf", 0.60)
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")
        self.scan_speed    = rospy.get_param("~scan_speed", 0.35)
        self.align_kp      = rospy.get_param("~align_kp", 1.0)
        self.align_w_max   = rospy.get_param("~align_w_max", 0.6)
        self.deadband_deg  = rospy.get_param("~deadband_deg", 2.0)
        self.stable_frames = rospy.get_param("~stable_frames", 8)
        self.imgsz        = rospy.get_param("~imgsz", 640)
        self.show_viz      = rospy.get_param("~show_viz", True)
        self.max_distance  = rospy.get_param("~max_distance", 2.0)  # meters
        self.max_distance = min(max(self.max_distance, 0.1), 10.0)  # Clamp to valid range


        if not os.environ.get("DISPLAY"):
            self.show_viz = False

        # ---- pubs/subs ----
        self.vel_pub    = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=1)
        self.status_pub = rospy.Publisher("/rotator/status", String, queue_size=1)
        rospy.Subscriber("/rotator/control", String, self._on_ctrl)

        # ---- RealSense setup ----
        self.pipeline = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        prof = self.pipeline.start(cfg)
        depth_sensor = prof.get_device().first_depth_sensor()


        # ---- Depth filtering ----
        
        self.threshold_filter = rs.threshold_filter()
        self.threshold_filter.set_option(rs.option.max_distance, self.max_distance )  
        self.align = rs.align(rs.stream.color)  # align depth to color

        # ---- Camera intrinsics ----
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

        # ---- State ----
        self.mode = "idle"
        self.prev_x1 = None
        self.mask_min_x = None
        self.in_continue = False
        self.align_stable = 0
        self.last_bbox = None

        rospy.loginfo(f"Ready. Max distance: {self.max_distance}m")

        # ---- Main loop ----
        self._loop()

    def _on_ctrl(self, msg: String):
        cmd = msg.data.strip().lower()
        if cmd == "start":
            self.prev_x1 = None
            self.mask_min_x = None
            self.in_continue = False
            self._set_mode_scanning()
        elif cmd == "continue":
            self.in_continue = True
            if self.prev_x1 is not None and self.prev_x1 > 150:
                self.mask_min_x = int(self.prev_x1) 
            elif self.prev_x1 is not None and self.prev_x1 <= 150:
                self.mask_min_x = 150
            self._set_mode_scanning()
        elif cmd == "stop":
            self._stop()
        else:
            rospy.logwarn(f"Unknown command: {cmd}")

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
        if self.mask_min_x is None:
            return img
        m = img.copy()
        x = max(0, min(self.image_width, int(self.mask_min_x)))
        m[:, x:self.image_width] = 0
        return m

    def _filter_depth(self, depth_frame):
        """Apply threshold filter to depth frame."""
        return self.threshold_filter.process(depth_frame)

    def _draw_viz(self, img, bbox, yaw, locked=False, depth_mask=None):
        if not self.show_viz:
            return
        disp = img.copy()
        if depth_mask is not None:
            disp[depth_mask] = 0  # Black out far pixels
        cv2.line(disp, (int(self.cx), 0), (int(self.cx), self.image_height-1), (255,255,255), 1)
        if self.mask_min_x is not None:
            x = max(0, min(self.image_width-1, int(self.mask_min_x)))
            cv2.line(disp, (x, 0), (x, self.image_height-1), (0,0,255), 2)
        if bbox is not None:
            x1,y1,x2,y2 = bbox
            cv2.rectangle(disp, (x1,y1), (x2,y2), (0,255,0), 2)
            if yaw is not None:
                cv2.putText(disp, f"{math.degrees(yaw):.1f}°", (x1, max(0,y1-6)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)
        if locked:
            cv2.putText(disp, "LOCKED", (10, self.image_height-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,200,0), 2)
        cv2.imshow("YOLO Spin (Depth Filtered)", disp)
        cv2.waitKey(1)

    def _loop(self):
        r = rospy.Rate(15)
        deadband = math.radians(self.deadband_deg)

        while not rospy.is_shutdown():
            try:
                frames = self.pipeline.wait_for_frames()
                aligned_frames = self.align.process(frames)
                depth_frame = aligned_frames.get_depth_frame()
                color_frame = aligned_frames.get_color_frame()
                if not depth_frame or not color_frame:
                    r.sleep(); continue
            except Exception as e:
                rospy.logwarn(f"RealSense error: {e}")
                r.sleep(); continue

            # Filter depth
            filtered_depth = self._filter_depth(depth_frame)
            depth_image = np.asanyarray(filtered_depth.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            # Mask RGB based on depth (optional)
            depth_mask = (depth_image == 0)  # Pixels beyond max_distance or invalid

            # Apply spatial mask (for "continue")
            det_img = self._mask_image(color_image)

            # YOLO inference
            try:
                results = self.model(det_img, imgsz=self.imgsz, verbose=False, device=self.device)
            except Exception as e:
                rospy.logwarn(f"YOLO error: {e}")
                r.sleep(); continue

            # Parse detections (only people within max_distance)
            yaw_best, box_best = None, None
            for res in results:
                if not hasattr(res, "boxes"): continue
                for b in res.boxes:
                    if int(b.cls[0]) != 0 or float(b.conf[0]) < self.min_conf:
                        continue
                    x1, y1, x2, y2 = map(int, b.xyxy[0].tolist())
                    # Check if centroid is within depth range
                    centroid_u, centroid_v = (x1 + x2) // 2, (y1 + y2) // 2
                    if depth_mask[centroid_v, centroid_u]:
                        continue  # Skip if centroid is too far
                    u = 0.5 * (x1 + x2)
                    yaw = math.atan2((u - self.cx) / self.fx, 1.0)
                    if yaw_best is None or abs(yaw) < abs(yaw_best):
                        yaw_best, box_best = yaw, (x1, y1, x2, y2)

            # State machine
            if self.mode == "scanning":
                self._send_w(self.scan_speed)
                if box_best is not None:
                    self.mode = "aligning"
                    self.status_pub.publish("aligning")
                    self.align_stable = 0
                    self.last_bbox = box_best
                self._draw_viz(color_image, box_best, yaw_best, depth_mask=depth_mask)
                r.sleep(); continue

            if self.mode == "aligning":
                if box_best is None:
                    self._set_mode_scanning()
                    self._draw_viz(color_image, None, None, depth_mask=depth_mask)
                    r.sleep(); continue

                self.last_bbox = box_best
                x1, y1, x2, y2 = box_best
                if self.in_continue and (self.prev_x1 is not None) and (x2 >= int(self.prev_x1)):
                    self.mask_min_x = int(x2) + 10

                w = clamp(-self.align_kp * yaw_best, -self.align_w_max, self.align_w_max)
                self._send_w(w)

                if abs(yaw_best) <= deadband:
                    self.align_stable += 1
                else:
                    self.align_stable = 0

                if self.align_stable >= self.stable_frames:
                    self._send_w(0.0)
                    self.mode = "locked"
                    self.status_pub.publish("person_detected")
                    self.prev_x1 = int(x1)
                    self._draw_viz(color_image, box_best, yaw_best, locked=True, depth_mask=depth_mask)
                    r.sleep(); continue

                self._draw_viz(color_image, box_best, yaw_best, depth_mask=depth_mask)
                r.sleep(); continue

            if self.mode == "locked":
                self._send_w(0.0)
                self._draw_viz(color_image, self.last_bbox, 0.0, locked=True, depth_mask=depth_mask)
                r.sleep(); continue

            # Idle
            self._send_w(0.0)
            r.sleep()

    def __del__(self):
        try:
            self.pipeline.stop()
        except:
            pass

if __name__ == "__main__":
    try:
        YoloSpinRealSense()
    except rospy.ROSInterruptException:
        pass
