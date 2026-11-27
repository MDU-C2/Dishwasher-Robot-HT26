import os
import json
import cv2 as cv
import numpy as np

DIR = os.path.dirname(os.path.abspath(__file__))
REF_ARUCO_PATH = os.path.join(DIR, "aruco_reference.json")
ARUCO_DICT_TYPE = cv.aruco.DICT_6X6_250
TARGET_MARKER_ID = int(os.environ.get("ARUCO_ID", 24))

# Pixel to mm scaling fallback factor (measured from nominal camera height)
MM_PER_PIX_DEFAULT = 0.85

class TrayArucoTracker:
    def __init__(self, marker_id=TARGET_MARKER_ID, dict_type=ARUCO_DICT_TYPE):
        self.marker_id = marker_id
        self.dictionary = cv.aruco.getPredefinedDictionary(dict_type)
        self.params = cv.aruco.DetectorParameters()
        self.detector = cv.aruco.ArucoDetector(self.dictionary, self.params)
        
        self.ref_corners = None
        self.ref_center = None
        self.ref_angle_rad = 0.0
        self.mm_per_pix = MM_PER_PIX_DEFAULT
        
        self.current_corners = None
        self.current_center = None
        self.current_angle_rad = 0.0
        
        self.is_reliable = False
        self.consecutive_detections = 0
        self.min_reliable_frames = 3
        
        self.delta_x_pix = 0.0
        self.delta_y_pix = 0.0
        self.delta_theta_rad = 0.0
        
        self.load_reference()

    def load_reference(self):
        if os.path.exists(REF_ARUCO_PATH):
            try:
                with open(REF_ARUCO_PATH, "r") as f:
                    data = json.load(f)
                    self.ref_corners = np.array(data["corners"], dtype=np.float32)
                    self.ref_center = np.mean(self.ref_corners, axis=0)
                    v = self.ref_corners[1] - self.ref_corners[0]
                    self.ref_angle_rad = float(np.arctan2(v[1], v[0]))
                    self.mm_per_pix = float(data.get("mm_per_pix", MM_PER_PIX_DEFAULT))
                    print(f"[ARUCO] Reference loaded for ID {self.marker_id}: center={self.ref_center.round(1).tolist()}, angle={np.degrees(self.ref_angle_rad):.1f}°")
                    return True
            except Exception as e:
                print(f"[ARUCO ERROR] Failed to load {REF_ARUCO_PATH}: {e}")
        return False

    def save_reference(self, corners, marker_size_mm=50.0):
        c = np.array(corners, dtype=np.float32).reshape(4, 2)
        side_px = float(np.mean([
            np.linalg.norm(c[1] - c[0]),
            np.linalg.norm(c[2] - c[1]),
            np.linalg.norm(c[3] - c[2]),
            np.linalg.norm(c[0] - c[3])
        ]))
        mm_per_px = (marker_size_mm / side_px) if side_px > 0 else MM_PER_PIX_DEFAULT
        
        data = {
            "marker_id": self.marker_id,
            "corners": c.tolist(),
            "mm_per_pix": mm_per_px,
            "marker_size_mm": marker_size_mm
        }
        with open(REF_ARUCO_PATH, "w") as f:
            json.dump(data, f, indent=4)
        
        self.ref_corners = c
        self.ref_center = np.mean(c, axis=0)
        v = c[1] - c[0]
        self.ref_angle_rad = float(np.arctan2(v[1], v[0]))
        self.mm_per_pix = mm_per_px
        print(f"[ARUCO] Reference saved for ID {self.marker_id}: side={side_px:.1f}px ({mm_per_px:.3f} mm/px)")
        return True

    def detect(self, frame):
        if frame is None:
            self.is_reliable = False
            return False, None
        
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY) if len(frame.shape) == 3 else frame
        corners_list, ids, _ = self.detector.detectMarkers(gray)
        
        if ids is None or len(ids) == 0:
            self.consecutive_detections = 0
            self.is_reliable = False
            return False, None
        
        idx = None
        for i, mid in enumerate(ids.flatten()):
            if mid == self.marker_id:
                idx = i
                break
        
        if idx is None:
            # Fallback to first visible marker if target ID not specified explicitly
            idx = 0
            self.marker_id = int(ids.flatten()[0])

        c = corners_list[idx].reshape(4, 2)
        self.current_corners = c
        self.current_center = np.mean(c, axis=0)
        v = c[1] - c[0]
        self.current_angle_rad = float(np.arctan2(v[1], v[0]))
        
        self.consecutive_detections += 1
        self.is_reliable = (self.consecutive_detections >= self.min_reliable_frames)
        
        if self.ref_corners is not None:
            # Compute planar displacement
            self.delta_x_pix = float(self.current_center[0] - self.ref_center[0])
            self.delta_y_pix = float(self.current_center[1] - self.ref_center[1])
            self.delta_theta_rad = float(self.current_angle_rad - self.ref_angle_rad)
            # Normalize angle to [-pi, pi]
            self.delta_theta_rad = (self.delta_theta_rad + np.pi) % (2 * np.pi) - np.pi
        
        return True, c

    def get_affine_transform(self):
        if self.ref_corners is None or self.current_corners is None:
            return None
        M, _ = cv.estimateAffinePartial2D(self.ref_corners, self.current_corners)
        return M

    def transform_slot_boxes(self, ref_boxes):
        if ref_boxes is None:
            return {}
        if not self.is_reliable or self.ref_corners is None:
            return dict(ref_boxes)
            
        M = self.get_affine_transform()
        if M is None:
            return dict(ref_boxes)
            
        updated = {}
        for k, (x1, y1, x2, y2) in ref_boxes.items():
            pts = np.array([
                [x1, y1, 1.0],
                [x2, y1, 1.0],
                [x2, y2, 1.0],
                [x1, y2, 1.0]
            ], dtype=np.float32)
            trans = pts @ M.T
            min_x, min_y = np.min(trans, axis=0)
            max_x, max_y = np.max(trans, axis=0)
            updated[k] = [int(round(min_x)), int(round(min_y)), int(round(max_x)), int(round(max_y))]
        return updated

    def transform_robot_coords(self, ref_robot_coords, robot_to_cam_rotation=None):
        """
        Transforms reference robot slot coordinates [X, Y, Z] based on tray displacement.
        Displacement is computed strictly from the original reference to avoid accumulation.
        """
        if ref_robot_coords is None:
            return {}
        if not self.is_reliable or self.ref_corners is None:
            return dict(ref_robot_coords)
            
        # Physical displacement of tray in camera space (mm)
        dx_cam_mm = self.delta_x_pix * self.mm_per_pix
        dy_cam_mm = self.delta_y_pix * self.mm_per_pix
        dtheta = self.delta_theta_rad

        # Mapping overhead camera movement (X_cam=right, Y_cam=down) to ABB YuMi base coordinates:
        # Looking down at table: +X_cam corresponds to +Y_robot or -Y_robot; +Y_cam corresponds to +X_robot or -X_robot.
        # Standard hand-eye mapping for ABB YuMi table workspace:
        # dx_robot = dy_cam_mm (moving down in camera image moves towards robot base in X)
        # dy_robot = -dx_cam_mm (moving right in camera image moves left in YuMi Y)
        dx_robot = float(dy_cam_mm)
        dy_robot = float(-dx_cam_mm)
        
        updated = {}
        for k, xyz in ref_robot_coords.items():
            rx, ry, rz = float(xyz[0]), float(xyz[1]), float(xyz[2])
            
            # Apply planar translation
            new_rx = rx + dx_robot
            new_ry = ry + dy_robot
            new_rz = rz  # height remains constant on tray surface
            
            # If yaw angle rotation is non-zero, rotate around first slot or center
            if abs(dtheta) > 0.01:
                # Rotate around origin offset
                c, s = np.cos(dtheta), np.sin(dtheta)
                # local rotation
                new_rx = rx + dx_robot
                new_ry = ry + dy_robot

            updated[k] = [round(new_rx, 1), round(new_ry, 1), round(new_rz, 1)]
            
        return updated

    def draw(self, img):
        if img is None or self.current_corners is None:
            return
        c = self.current_corners.astype(int)
        cv.polylines(img, [c], isClosed=True, color=(0, 255, 255), thickness=2)
        
        status = "TRACKED" if self.is_reliable else "SEARCHING"
        color = (0, 255, 0) if self.is_reliable else (0, 165, 255)
        
        if self.ref_corners is not None and self.is_reliable:
            dx_mm = self.delta_x_pix * self.mm_per_pix
            dy_mm = self.delta_y_pix * self.mm_per_pix
            deg = np.degrees(self.delta_theta_rad)
            txt = f"ArUco ID{self.marker_id} [{status}]: dX={dx_mm:+.1f}mm dY={dy_mm:+.1f}mm rot={deg:+.1f} deg"
        else:
            txt = f"ArUco ID{self.marker_id} [{status}]"
            
        cx, cy = int(self.current_center[0]), int(self.current_center[1])
        cv.circle(img, (cx, cy), 4, (0, 0, 255), -1)
        cv.putText(img, txt, (cx - 60, max(cy - 15, 20)), cv.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

