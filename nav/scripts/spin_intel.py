#!/usr/bin/env python3
import os, time, math, threading
import rospy, cv2, numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist

def clamp(x, lo, hi): return lo if x < lo else hi if x > hi else x

def iou_xyxy(a, b):
    ax1, ay1, ax2, ay2 = a
    bx1, by1, bx2, by2 = b
    ix1, iy1 = max(ax1, bx1), max(ay1, by1)
    ix2, iy2 = min(ax2, bx2), min(ay2, by2)
    iw, ih = max(0, ix2 - ix1), max(0, iy2 - iy1)
    inter = iw * ih
    area_a = max(0, ax2 - ax1) * max(0, ay2 - ay1)
    area_b = max(0, bx2 - bx1) * max(0, by2 - by1)
    union = area_a + area_b - inter
    return inter / union if union > 0 else 0.0

class YoloSpinRealSense:
    def __init__(self):
        rospy.init_node("yolo_spin_only")

        # -------- params --------
        self.model_path    = rospy.get_param("~model", "yolo12n.pt")
        self.min_conf      = rospy.get_param("~min_conf", 0.60)
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/mobile_base/commands/velocity")

        # RealSense
        self.rs_width      = rospy.get_param("~rs_width", 1280)
        self.rs_height     = rospy.get_param("~rs_height", 720)
        self.rs_fps        = rospy.get_param("~rs_fps", 30)
        self.rs_serial     = rospy.get_param("~rs_serial", "")

        # scanning
        self.scan_speed    = rospy.get_param("~scan_speed", 0.35)   # rad/s (+left)
        self.skip_frames   = rospy.get_param("~skip_frames", 2)
        self.scan_imgsz    = rospy.get_param("~scan_imgsz", 512)

        # aligning
        self.align_enable        = rospy.get_param("~align_enable", True)
        self.align_kp            = rospy.get_param("~align_kp", 1.2)   # rad/s per rad
        self.align_w_max         = rospy.get_param("~align_w_max", 0.8)
        self.align_deadband_deg  = rospy.get_param("~align_deadband_deg", 2.0)
        self.align_lost_timeout  = rospy.get_param("~align_lost_timeout", 0.6)
        self.align_imgsz         = rospy.get_param("~align_imgsz", 640)
        self.align_dwell_sec     = rospy.get_param("~align_dwell_sec", 2.0)  # time-based lock dwell

        # motion sign flip
        self.invert_w     = rospy.get_param("~invert_w", False)

        # UI / misc
        self.ignore_sec   = rospy.get_param("~ignore_after_continue_sec", 0.4)
        self.show_viz     = rospy.get_param("~show_viz", True)
        self.flip_viz     = rospy.get_param("~flip_viz", False)
        self.debug_logs   = rospy.get_param("~debug_logs", True)

        # --- continue-mask behavior ---
        # mode 'leftedge': mask starts at last bbox left edge
        # mode 'relative': mask starts at (u_last + multiplier * w_last)
        self.continue_mask_mode        = rospy.get_param("~continue_mask_mode", "leftedge")
        self.continue_mask_multiplier  = rospy.get_param("~continue_mask_multiplier", 1.5)  # 1.5 == 3/2
        self.continue_mask_enable      = rospy.get_param("~continue_mask_enable", True)
        self.continue_mask_visualize   = rospy.get_param("~continue_mask_visualize", True)
        self.mask_shrink_rate_px_s     = rospy.get_param("~mask_shrink_rate_px_s", 400.0)  # px/sec while ALIGNING

        # target stickiness in ALIGNING (prevents switching persons)
        self.sticky_enable     = rospy.get_param("~sticky_enable", True)
        self.sticky_dx_max_px  = rospy.get_param("~sticky_dx_max_px", 200)  # allowable drift from initial target center
        self.sticky_iou_min    = rospy.get_param("~sticky_iou_min", 0.05)

        # state for continue mask
        self._mask_active = False
        self._mask_x_min  = 0               # current left boundary of the masked region [x>=mask_x_min ignored]
        self._mask_last_ts = time.time()

        # last locked bbox for "continue"
        self._last_lock_u = None
        self._last_lock_w = None
        self._last_lock_box = None  # (x1,y1,x2,y2)

        # ALIGNING stickiness
        self._align_seed_u  = None
        self._align_seed_box = None
        self._deadband_enter_ts = None

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
        self._last_w = 0.0

        # shared frame
        self.latest = None
        self.lock = threading.Lock()

        # -------- RealSense init (RGB only) --------
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        if self.rs_serial:
            self.config.enable_device(self.rs_serial)
        self.config.enable_stream(rs.stream.color, self.rs_width, self.rs_height, rs.format.bgr8, self.rs_fps)

        try:
            self.profile = self.pipeline.start(self.config)
        except Exception as e:
            rospy.logerr(f"RealSense start failed: {e}")
            raise SystemExit(1)

        color_stream = self.profile.get_stream(rs.stream.color).as_video_stream_profile()
        intr = color_stream.get_intrinsics()
        self.fx = float(intr.fx)
        self.cx = float(intr.ppx)
        self._img_w = intr.width
        self._img_h = intr.height
        rospy.loginfo(f"RealSense color: {intr.width}x{intr.height} @ {self.rs_fps} fps, fx={self.fx:.1f}, cx={self.cx:.1f}")

        rospy.on_shutdown(self._shutdown_rs)

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

        if cmd == "start":
            # start fresh: no mask
            self._mask_active = False
            self.ignore_until = time.time() + self.ignore_sec
            self._start_scan()

        elif cmd == "continue":
            # enable mask from previous lock
            self._mask_active = bool(self.continue_mask_enable)
            self.ignore_until = time.time() + self.ignore_sec
            # initialize mask boundary
            if (self._last_lock_u is not None) and (self._last_lock_w is not None):
                if self.continue_mask_mode.lower() == "leftedge":
                    last_x1 = int(self._last_lock_u - 0.5 * self._last_lock_w)
                    self._mask_x_min = max(0, min(self._img_w - 1, last_x1))
                else:  # 'relative'
                    self._mask_x_min = int(self._last_lock_u + self.continue_mask_multiplier * self._last_lock_w)
                    self._mask_x_min = max(0, min(self._img_w - 1, self._mask_x_min))
            else:
                # fallback: ignore right 2/3
                self._mask_x_min = self._img_w // 3
            self._mask_last_ts = time.time()
            self._start_scan()

        elif cmd == "stop":
            self._mask_active = False
            self._stop_scan()

        else:
            rospy.logwarn(f"Unknown control command: {cmd}")

    def _start_scan(self):
        self.mode = "scanning"
        self.status_pub.publish("scanning")
        self._deadband_enter_ts = None
        self._align_seed_u = None
        self._align_seed_box = None
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
        if self.invert_w: w = -w
        self._last_w = float(w)
        tw = Twist(); tw.angular.z = self._last_w
        self.vel_pub.publish(tw)

    def _spin_keepalive_cb(self, _):
        if self.mode in ("scanning", "aligning"):
            self._spin_cmd(self._last_w)

    # ---------- capture ----------
    def _capture_loop(self):
        while not rospy.is_shutdown():
            try:
                frames = self.pipeline.wait_for_frames()
            except Exception:
                continue
            color = frames.get_color_frame()
            if not color: continue
            frame = np.asanyarray(color.get_data())  # BGR8
            with self.lock:
                self.latest = frame

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

            w_img, h_img = img.shape[1], img.shape[0]

            # candidate selection
            yaw_best = None
            box_best = None
            det_count = 0

            for r in results:
                if not getattr(r, "boxes", None): continue
                for b in r.boxes:
                    cls_id = int(b.cls[0].item()) if b.cls is not None else -1
                    conf   = float(b.conf[0].item()) if b.conf is not None else 0.0
                    if cls_id != 0 or conf < self.min_conf:  # person class
                        continue

                    x1, y1, x2, y2 = map(int, b.xyxy[0].tolist())
                    u = 0.5 * (x1 + x2)

                    # Apply mask ONLY during SCANNING (so ALIGNING can center)
                    if self._mask_active and self.mode == "scanning" and (u >= self._mask_x_min):
                        continue

                    # Stickiness during ALIGNING: keep closest to the seed & sufficient IoU
                    if self.mode == "aligning" and self.sticky_enable and (self._align_seed_u is not None):
                        dx = abs(u - self._align_seed_u)
                        iou = iou_xyxy((x1,y1,x2,y2), self._align_seed_box) if self._align_seed_box else 0.0
                        if (dx > self.sticky_dx_max_px) and (iou < self.sticky_iou_min):
                            continue

                    yaw_raw = math.atan2((u - self.cx) / self.fx, 1.0)  # +right
                    det_count += 1
                    if (yaw_best is None) or (abs(yaw_raw) < abs(yaw_best)):
                        yaw_best = yaw_raw
                        box_best = (x1, y1, x2, y2)

            # draw overlays
            if box_best is not None:
                x1,y1,x2,y2 = box_best
                cv2.rectangle(img, (x1,y1), (x2,y2), (0,255,0), 2)
                cv2.putText(img, f"{math.degrees(yaw_best):.1f}°", (x1, max(0,y1-7)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)

            # center + target columns
            if self.show_viz:
                cx_pix = int(self.cx)
                cv2.line(img, (cx_pix, 0), (cx_pix, h_img-1), (255,255,255), 1)
                if yaw_best is not None:
                    u_pix = int(self.cx + math.tan(yaw_best) * self.fx)
                    u_pix = max(0, min(w_img-1, u_pix))
                    cv2.line(img, (u_pix, 0), (u_pix, h_img-1), (0,255,0), 1)

                # Show mask (during SCANNING) or shrinking mask (during ALIGNING) for visibility
                if self._mask_active and self.continue_mask_visualize:
                    overlay = img.copy()
                    cv2.rectangle(overlay, (self._mask_x_min, 0), (w_img - 1, h_img - 1), (50, 50, 50), thickness=-1)
                    img[:] = cv2.addWeighted(overlay, 0.35, img, 0.65, 0)
                    cv2.line(img, (self._mask_x_min, 0), (self._mask_x_min, h_img - 1), (0, 0, 255), 2)

            # ------------- state machine -------------
            if self.mode == "scanning":
                self._show(img)
                if self.debug_logs:
                    rospy.loginfo(f"YOLO {det_count} valid in {infer_ms:.1f} ms (SCANNING) mask_x_min={self._mask_x_min}")
                if yaw_best is not None:
                    if not self.align_enable:
                        # immediate lock (legacy behavior)
                        self._spin_cmd(0.0)
                        self.mode = "locked"
                        self._mask_active = False
                        self.status_pub.publish("locked")
                        if box_best is not None:
                            x1,y1,x2,y2 = box_best
                            self._last_lock_u = 0.5 * (x1 + x2)
                            self._last_lock_w = float(x2 - x1)
                            self._last_lock_box = (x1,y1,x2,y2)
                        yaw_wrapped = (yaw_best + math.pi) % (2*math.pi) - math.pi
                        self.yaw_pub.publish(Float32(data=yaw_wrapped))
                        rospy.loginfo("LOCKED (no align) at %.1f° — waiting for 'continue'.",
                                      math.degrees(yaw_wrapped))
                    else:
                        # go align; keep mask active for viz, but NOT applied to filtering anymore
                        self.mode = "aligning"
                        self.status_pub.publish("aligning")
                        # seed target for stickiness
                        if box_best is not None:
                            x1,y1,x2,y2 = box_best
                            self._align_seed_u = 0.5 * (x1 + x2)
                            self._align_seed_box = (x1,y1,x2,y2)
                        self._deadband_enter_ts = None
                        self._mask_last_ts = time.time()
                        rospy.loginfo("Hit person at %.1f° → ALIGNING…", math.degrees(yaw_best))
                continue

            if self.mode == "aligning":
                now = time.time()

                # shrink the mask gradually so the bbox can "slide" to center
                if self._mask_active:
                    dt = max(0.0, now - self._mask_last_ts)
                    self._mask_last_ts = now
                    self._mask_x_min = max(0, int(self._mask_x_min - self.mask_shrink_rate_px_s * dt))
                    # ensure center eventually unmasked
                    if self._mask_x_min > int(self.cx):
                        self._mask_x_min = max(int(self.cx), self._mask_x_min - 1)

                self._show(img)

                # target temporarily lost? brief coast then back to scanning
                if yaw_best is None:
                    if (self._deadband_enter_ts is None) or ((now - self._deadband_enter_ts) < self.align_lost_timeout):
                        w_cmd = self._last_w
                        if abs(w_cmd) < 0.2: w_cmd = 0.2 if self.scan_speed >= 0 else -0.2
                        self._spin_cmd(clamp(w_cmd, -self.align_w_max, self.align_w_max))
                        continue
                    self.mode = "scanning"
                    self.status_pub.publish("scanning")
                    self._spin_cmd(self.scan_speed)
                    self._mask_active = True  # resume visible mask while re-scanning
                    continue

                # control
                yaw_err = yaw_best
                w_cmd = clamp(-self.align_kp * yaw_err, -self.align_w_max, self.align_w_max)
                self._spin_cmd(w_cmd)

                # dwell-based lock: inside deadband for align_dwell_sec
                if abs(yaw_err) <= deadband:
                    if self._deadband_enter_ts is None:
                        self._deadband_enter_ts = now
                    dwell = now - self._deadband_enter_ts
                    if self.debug_logs:
                        rospy.loginfo("ALIGN: |yaw|=%.2f° ≤ %.2f°  dwell=%.2fs/%.2fs  w=%+.2f",
                                      math.degrees(abs(yaw_err)), self.align_deadband_deg,
                                      dwell, self.align_dwell_sec, w_cmd)
                    if dwell >= self.align_dwell_sec:
                        self.mode = "locked"
                        self._mask_active = False
                        self.status_pub.publish("locked")
                        # record last lock box
                        if box_best is not None:
                            x1,y1,x2,y2 = box_best
                            self._last_lock_u = 0.5 * (x1 + x2)
                            self._last_lock_w = float(x2 - x1)
                            self._last_lock_box = (x1,y1,x2,y2)
                        else:
                            self._last_lock_u = float(self.cx)
                            self._last_lock_w = 0.2 * self._img_w
                            self._last_lock_box = (int(self.cx-0.1*self._img_w), 0, int(self.cx+0.1*self._img_w), self._img_h)
                        yaw_wrapped = (yaw_err + math.pi) % (2*math.pi) - math.pi
                        self.yaw_pub.publish(Float32(data=yaw_wrapped))
                        rospy.loginfo("LOCKED (centered): yaw=%.1f° — waiting for 'continue'.",
                                      math.degrees(yaw_wrapped))
                else:
                    self._deadband_enter_ts = None
                continue

            # mode == "locked"
            self._show(img)

    # ---------- viz ----------
    def _show(self, img):
        if not self.show_viz:
            return
        try:
            disp = img
            if self.flip_viz:
                disp = cv2.flip(disp, 0)
            cv2.imshow("RealSense view (spin)", disp)
            cv2.waitKey(1)
        except cv2.error:
            self.show_viz = False
            rospy.logwarn("OpenCV window failed (headless?). Disabling viz.")

    # ---------- shutdown ----------
    def _shutdown_rs(self):
        try:
            self.pipeline.stop()
        except Exception:
            pass

# ---------- main ----------
if __name__ == "__main__":
    try:
        YoloSpinRealSense()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
