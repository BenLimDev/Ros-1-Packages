#!/usr/bin/env python3
import os, time, math, threading
import rospy, cv2, numpy as np, pyzed.sl as sl
from ultralytics import YOLO
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist

def clamp(x, lo, hi): return lo if x < lo else hi if x > hi else x

class YoloSpinZED:
    def __init__(self):
        rospy.init_node("yolo_spin_only")

        # -------- params --------
        self.model_path    = rospy.get_param("~model", "yolo12n.pt")
        self.min_conf      = rospy.get_param("~min_conf", 0.60)
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/mobile_base/commands/velocity")

        # scanning
        self.scan_speed    = rospy.get_param("~scan_speed", 0.35)   # rad/s (+left)
        self.skip_frames   = rospy.get_param("~skip_frames", 2)     # YOLO every N frames in SCANNING
        self.scan_imgsz    = rospy.get_param("~scan_imgsz", 512)

        # aligning
        self.align_enable        = rospy.get_param("~align_enable", True)
        self.align_kp            = rospy.get_param("~align_kp", 1.2)   # rad/s per rad
        self.align_w_max         = rospy.get_param("~align_w_max", 0.8)
        self.align_deadband_deg  = rospy.get_param("~align_deadband_deg", 2.0)
        self.align_stable_frames = rospy.get_param("~align_stable_frames", 6)
        self.align_lost_timeout  = rospy.get_param("~align_lost_timeout", 0.6)
        self.align_imgsz         = rospy.get_param("~align_imgsz", 640)

        # motion sign flip (if your base rotates opposite)
        self.invert_w     = rospy.get_param("~invert_w", False)

        # UI / misc
        self.ignore_sec   = rospy.get_param("~ignore_after_continue_sec", 0.4)
        self.show_viz     = rospy.get_param("~show_viz", True)
        self.flip_viz     = rospy.get_param("~flip_viz", False)
        self.debug_logs   = rospy.get_param("~debug_logs", True)

        if not os.environ.get("DISPLAY"):
            self.show_viz = False
            rospy.logwarn("DISPLAY not set; disabling OpenCV windows.")

        # -------- pubs/subs --------
        self.vel_pub     = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=1)
        self.status_pub  = rospy.Publisher("/rotator/status", String, queue_size=1)
        self.yaw_pub     = rospy.Publisher("/rotator/target_yaw", Float32, queue_size=1)
        rospy.Subscriber("/rotator/control", String, self._on_ctrl)

        # -------- state --------
        self.mode = "idle"   # "idle" | "scanning" | "aligning" | "locked"
        self.ignore_until = 0.0
        self.frame_i = 0
        self.align_stable = 0
        self.align_last_seen_ts = 0.0
        self._last_w = 0.0

        # shared frame
        self.latest = None
        self.lock = threading.Lock()

        # -------- ZED init (RGB only) --------
        self.zed = sl.Camera()
        ip = sl.InitParameters()
        ip.camera_resolution = sl.RESOLUTION.HD720
        ip.camera_fps = 30
        ip.depth_mode = sl.DEPTH_MODE.NONE
        ip.coordinate_units = sl.UNIT.METER
        err = self.zed.open(ip)
        if err != sl.ERROR_CODE.SUCCESS:
            rospy.logerr(f"ZED open failed: {err}")
            raise SystemExit(1)

        info = self.zed.get_camera_information()
        try:
            calib = info.camera_configuration.calibration_parameters.left_cam
        except AttributeError:
            calib = info.calibration_parameters.left_cam  # older SDK
        self.fx = float(calib.fx)
        self.cx = float(calib.cx)
        res = info.camera_configuration.resolution
        rospy.loginfo(f"ZED opened: {res.width}x{res.height}, fx={self.fx:.1f}, cx={self.cx:.1f}")

        self.img_mat = sl.Mat()
        self.rt = sl.RuntimeParameters()

        # -------- YOLO --------
        self.model = YOLO(self.model_path)
        try:
            self.model.cuda()
            rospy.loginfo(f"YOLO model '{self.model_path}' on CUDA")
        except Exception as e:
            rospy.logwarn(f"YOLO CUDA unavailable ({e}); running on CPU")

        # keepalive timer (refresh current cmd @5Hz)
        self._keepalive_timer = rospy.Timer(rospy.Duration(0.2), self._spin_keepalive_cb)

        # threads
        threading.Thread(target=self._capture_loop, daemon=True).start()
        threading.Thread(target=self._detect_loop,  daemon=True).start()

    # ---------- control ----------
    def _on_ctrl(self, msg: String):
        cmd = msg.data.strip().lower()
        rospy.loginfo(f"/rotator/control: '{cmd}'")
        if cmd in ("start", "continue"):
            self.ignore_until = time.time() + self.ignore_sec
            self._start_scan()
        elif cmd == "stop":
            self._stop_scan()
        else:
            rospy.logwarn(f"Unknown control command: {cmd}")

    def _start_scan(self):
        self.mode = "scanning"
        self.align_stable = 0
        self.status_pub.publish("scanning")
        self._spin_cmd(self.scan_speed)
        if self.debug_logs:
            rospy.loginfo(f"SCANNING… angular.z={self.scan_speed:.2f} (invert_w={self.invert_w})")

    def _stop_scan(self):
        self.mode = "idle"
        self._spin_cmd(0.0)
        self.status_pub.publish("idle")
        if self.debug_logs:
            rospy.loginfo("Stopped. Status=idle")

    def _spin_cmd(self, w):
        # send angular.z with optional sign flip
        if self.invert_w:
            w = -w
        self._last_w = float(w)
        tw = Twist(); tw.angular.z = self._last_w
        self.vel_pub.publish(tw)

    def _spin_keepalive_cb(self, _):
        if self.mode in ("scanning", "aligning"):
            self._spin_cmd(self._last_w)

    # ---------- capture ----------
    def _capture_loop(self):
        r = rospy.Rate(60)
        missed = 0
        while not rospy.is_shutdown():
            if self.zed.grab(self.rt) == sl.ERROR_CODE.SUCCESS:
                self.zed.retrieve_image(self.img_mat, sl.VIEW.LEFT)  # BGRA
                frame = cv2.cvtColor(self.img_mat.get_data(), cv2.COLOR_BGRA2BGR)
                with self.lock:
                    self.latest = frame
                missed = 0
            else:
                missed += 1
                if missed % 60 == 0:
                    rospy.logwarn("ZED grab failing intermittently…")
            r.sleep()

    # ---------- main loop ----------
    def _detect_loop(self):
        deadband = math.radians(self.align_deadband_deg)
        while not rospy.is_shutdown():
            if (self.mode in ("idle", "locked")) or (time.time() < self.ignore_until):
                time.sleep(0.01); continue

            with self.lock:
                img = None if self.latest is None else self.latest.copy()
            if img is None:
                time.sleep(0.005); continue

            self.frame_i += 1

            # In SCANNING, skip frames to save compute
            if self.mode == "scanning" and (self.frame_i % self.skip_frames != 0):
                if self.debug_logs and self.frame_i % (self.skip_frames*10) == 0:
                    rospy.loginfo(f"Skipping frames… frame_i={self.frame_i}")
                self._show(img); continue

            # YOLO inference
            t0 = time.time()
            try:
                imgsz = self.align_imgsz if self.mode == "aligning" else self.scan_imgsz
                results = self.model(img, verbose=False, imgsz=imgsz)
            except Exception as e:
                rospy.logerr(f"YOLO inference error: {e}")
                time.sleep(0.1); continue
            infer_ms = (time.time() - t0) * 1000.0

            # pick most frontal person (min |yaw|)
            yaw_best = None
            box_best = None
            det_count = 0
            for r in results:
                if not getattr(r, "boxes", None): continue
                for b in r.boxes:
                    cls_id = int(b.cls[0].item()) if b.cls is not None else -1
                    conf   = float(b.conf[0].item()) if b.conf is not None else 0.0
                    if cls_id != 0 or conf < self.min_conf: continue  # class 0 = person
                    x1, y1, x2, y2 = map(int, b.xyxy[0].tolist())
                    u = 0.5 * (x1 + x2)
                    # NOTE: yaw_raw > 0 means target is to the RIGHT of center.
                    yaw_raw = math.atan2((u - self.cx) / self.fx, 1.0)
                    det_count += 1
                    if (yaw_best is None) or (abs(yaw_raw) < abs(yaw_best)):
                        yaw_best = yaw_raw
                        box_best = (x1, y1, x2, y2)

            # draw overlays
            if box_best is not None:
                x1,y1,x2,y2 = box_best
                cv2.rectangle(img, (x1,y1), (x2,y2), (0,255,0), 2)
                cv2.putText(img, f"{math.degrees(yaw_best):.1f}°",
                            (x1, max(0,y1-7)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)

            # center + target columns
            if self.show_viz:
                h, w = img.shape[:2]
                cx_pix = int(self.cx)
                cv2.line(img, (cx_pix, 0), (cx_pix, h-1), (255,255,255), 1)
                if yaw_best is not None:
                    u_pix = int(self.cx + math.tan(yaw_best) * self.fx)
                    u_pix = max(0, min(w-1, u_pix))
                    cv2.line(img, (u_pix, 0), (u_pix, h-1), (0,255,0), 1)

            # ------------- state machine -------------
            if self.mode == "scanning":
                self._show(img)
                if self.debug_logs:
                    rospy.loginfo(f"YOLO {det_count} valid in {infer_ms:.1f} ms (SCANNING)")
                if yaw_best is not None:
                    if not self.align_enable:
                        # immediate lock (legacy behavior)
                        self._spin_cmd(0.0)
                        self.mode = "locked"
                        self.status_pub.publish("locked")
                        yaw_wrapped = (yaw_best + math.pi) % (2*math.pi) - math.pi
                        self.yaw_pub.publish(Float32(data=yaw_wrapped))
                        rospy.loginfo("LOCKED (no align) at %.1f° — waiting for 'continue'.",
                                      math.degrees(yaw_wrapped))
                    else:
                        # go align
                        self.mode = "aligning"
                        self.align_stable = 0
                        self.align_last_seen_ts = time.time()
                        self.status_pub.publish("aligning")
                        rospy.loginfo("Hit person at %.1f° → ALIGNING…", math.degrees(yaw_best))
                continue

            if self.mode == "aligning":
                self._show(img)

                # If no detection this frame: briefly coast, then give up
                if yaw_best is None:
                    if (time.time() - self.align_last_seen_ts) < self.align_lost_timeout:
                        # keep the last angular direction but clamp magnitude
                        w = self._last_w
                        if abs(w) < 0.2:
                            w = 0.2 if self.scan_speed >= 0 else -0.2
                        # _spin_cmd handles invert_w
                        self._spin_cmd(clamp(w, -self.align_w_max, self.align_w_max))
                        if self.debug_logs:
                            rospy.loginfo("ALIGN: lost target, coasting w=%+.2f… (last_w sent)", self._last_w)
                        continue
                    self.mode = "scanning"
                    self.status_pub.publish("scanning")
                    self._spin_cmd(self.scan_speed)
                    rospy.loginfo("ALIGN: lost too long → back to SCANNING.")
                    continue

                # yaw_err is simply yaw_best (positive -> target RIGHT)
                yaw_err = yaw_best
                # Control LAW: because yaw>0 when target is RIGHT, we must turn RIGHT -> w must be NEGATIVE.
                w_cmd = clamp(-self.align_kp * yaw_err, -self.align_w_max, self.align_w_max)
                self._spin_cmd(w_cmd)
                if self.debug_logs:
                    rospy.loginfo("ALIGN: yaw=%.2f° (err=%.4f rad) -> w=%+.2f rad/s",
                                  math.degrees(yaw_err), yaw_err, w_cmd)

                # Lock when within deadband for N frames
                if abs(yaw_err) <= deadband:
                    self._spin_cmd(0.0)
                    self.align_stable += 1
                    if self.debug_logs:
                        rospy.loginfo("ALIGN: |yaw|=%.1f° ≤ %.1f° → stable %d/%d",
                                      math.degrees(abs(yaw_err)), self.align_deadband_deg,
                                      self.align_stable, self.align_stable_frames)
                    if self.align_stable >= self.align_stable_frames:
                        self.mode = "locked"
                        self.status_pub.publish("locked")
                        yaw_wrapped = (yaw_err + math.pi) % (2*math.pi) - math.pi
                        self.yaw_pub.publish(Float32(data=yaw_wrapped))
                        rospy.loginfo("LOCKED (centered): yaw=%.1f° — waiting for 'continue'.",
                                      math.degrees(yaw_wrapped))
                else:
                    self.align_stable = 0
                continue

            # mode == "locked" → just show image; wait for control
            self._show(img)

    # ---------- viz ----------
    def _show(self, img):
        if not self.show_viz:
            return
        try:
            disp = img
            if self.flip_viz:
                disp = cv2.flip(disp, 0)
            cv2.imshow("ZED view (spin)", disp)
            cv2.waitKey(1)
        except cv2.error:
            self.show_viz = False
            rospy.logwarn("OpenCV window failed (headless?). Disabling viz.")

# ---------- main ----------
if __name__ == "__main__":
    try:
        YoloSpinZED()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
