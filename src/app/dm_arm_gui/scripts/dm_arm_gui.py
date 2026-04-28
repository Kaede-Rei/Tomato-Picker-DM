#! /usr/bin/env python
import sys
import time
from dataclasses import dataclass
from typing import Any, Optional, Tuple, Union

import cv2
import numpy as np

import rospy
import actionlib
import tf2_ros
import tf2_geometry_msgs  # noqa: F401
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped, PointStamped
from dm_arm_msgs.msg import (
    PickTaskAction,
    PickTaskGoal,
    SimpleMoveArmAction,
    SimpleMoveArmGoal,
)
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float32

from PyQt5.QtCore import Qt, QPoint, pyqtSignal, QTimer
from PyQt5.QtGui import (
    QBrush,
    QColor,
    QFont,
    QImage,
    QPainter,
    QPen,
    QPixmap,
    QPolygonF,
    QKeySequence,
)
from PyQt5.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QFormLayout,
    QHBoxLayout,
    QGroupBox,
    QLabel,
    QMainWindow,
    QLineEdit,
    QPushButton,
    QShortcut,
    QScrollArea,
    QSizePolicy,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)


@dataclass(frozen=True)
class DepthConfig:
    min_mm: int = 100
    max_mm: int = 10000
    kernel_size: int = 15


@dataclass(frozen=True)
class LRMConfig:
    valid_min_mm: int = 1
    valid_max_mm: int = 400


@dataclass(frozen=True)
class CameraConfig:
    rotate_180: bool = False


@dataclass(frozen=True)
class RosConfig:
    node_name: str = "tomato_picking_gui"
    action_name: str = "/pick_action"
    simple_move_action_name: str = "/simple_move_arm"
    pick_group_name: str = "gui_pick"
    flange_frame: str = "link6"
    tcp_frame: str = "link_tcp"
    place_frame: str = "base_link"
    camera_color_frame: str = "eef_camera_color_optical_frame"
    action_wait_sec: float = 5.0
    result_wait_sec: float = 10.0
    tf_wait_sec: float = 0.2
    camera_color_topic: str = "/dm_arm/camera/orbbec/color/image_raw"
    camera_color_info_topic: str = "/dm_arm/camera/orbbec/color/camera_info"
    camera_depth_raw_topic: str = "/dm_arm/camera/orbbec/depth/image_raw"
    camera_depth_raw_info_topic: str = "/dm_arm/camera/orbbec/depth/camera_info"
    camera_depth_registered_topic: str = (
        "/dm_arm/camera/orbbec/depth_registered/image_raw"
    )
    camera_depth_registered_info_topic: str = (
        "/dm_arm/camera/orbbec/depth_registered/camera_info"
    )
    camera_lrm_topic: str = "/dm_arm/camera/orbbec/lrm_distance"


@dataclass(frozen=True)
class UiConfig:
    window_title: str = "番茄"
    window_x: int = 100
    window_y: int = 100
    window_w: int = 1280
    window_h: int = 720
    image_min_w: int = 640
    image_min_h: int = 480
    info_font_name: str = "Arial"
    info_font_size: int = 10
    render_fps_limit: int = 20


@dataclass(frozen=True)
class TargetSelectionConfig:
    point_method: str = "skeleton_centroid"
    mask_close_kernel: int = 5
    mask_open_kernel: int = 3
    depth_kernel_size: int = 7
    depth_trim_ratio: float = 0.15
    center_band_min_dist_px: float = 1.0
    prefer_lrm_override: bool = True


@dataclass(frozen=True)
class ResidualCompensationConfig:
    enabled: bool = False
    dx: float = 0.0200
    dy: float = 0.0200
    dz: float = 0.0100


@dataclass(frozen=True)
class IntrinsicsConfig:
    fx: float = 612.28741455
    fy: float = 612.24603271
    cx: float = 638.46850586
    cy: float = 399.93331909


DEPTH_CFG = DepthConfig()
LRM_CFG = LRMConfig()
CAMERA_CFG = CameraConfig()
ROS_CFG = RosConfig()
UI_CFG = UiConfig()
TARGET_CFG = TargetSelectionConfig()
RESIDUAL_COMP_CFG = ResidualCompensationConfig()
INTRINSICS_CFG = IntrinsicsConfig()

DEFAULT_INTRINSICS = np.array(
    [
        [INTRINSICS_CFG.fx, 0.0, INTRINSICS_CFG.cx],
        [0.0, INTRINSICS_CFG.fy, INTRINSICS_CFG.cy],
        [0.0, 0.0, 1.0],
    ],
    dtype=np.float64,
)

MIN_DEPTH = DEPTH_CFG.min_mm
MAX_DEPTH = DEPTH_CFG.max_mm
CAMERA_ROTATE_180 = CAMERA_CFG.rotate_180


def rotate_point_180(u: int, v: int, width: int, height: int) -> Tuple[int, int]:
    return width - 1 - int(u), height - 1 - int(v)


def cam_point_to_flange_point(
    tf_buffer: tf2_ros.Buffer,
    source_frame_id: str,
    x_cam: float,
    y_cam: float,
    z_cam: float,
) -> Tuple[float, float, float]:
    if tf_buffer is None:
        raise RuntimeError("ROS/TF 未初始化，无法执行 camera -> flange 变换")
    if not source_frame_id:
        raise RuntimeError("相机坐标系未就绪，无法执行 camera -> flange 变换")

    pt_cam = PointStamped()
    pt_cam.header.frame_id = source_frame_id
    pt_cam.header.stamp = rospy.Time(0)
    pt_cam.point.x = x_cam
    pt_cam.point.y = y_cam
    pt_cam.point.z = z_cam

    pt_flange = tf_buffer.transform(
        pt_cam, ROS_CFG.flange_frame, rospy.Duration(ROS_CFG.tf_wait_sec)
    )
    return pt_flange.point.x, pt_flange.point.y, pt_flange.point.z


def get_mask_center(mask: np.ndarray) -> Optional[Tuple[float, float]]:
    M = cv2.moments(mask.astype(np.uint8))
    if M["m00"] == 0:
        return None
    cx = float(M["m10"] / M["m00"])
    cy = float(M["m01"] / M["m00"])
    return cx, cy


def preprocess_selection_mask(
    mask: np.ndarray,
    close_kernel: int = TARGET_CFG.mask_close_kernel,
    open_kernel: int = TARGET_CFG.mask_open_kernel,
) -> np.ndarray:
    binary_mask = (mask > 0).astype(np.uint8) * 255
    if binary_mask.size == 0:
        return binary_mask

    close_kernel = max(1, int(close_kernel))
    open_kernel = max(1, int(open_kernel))
    if close_kernel % 2 == 0:
        close_kernel += 1
    if open_kernel % 2 == 0:
        open_kernel += 1

    kernel_close = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE, (close_kernel, close_kernel)
    )
    kernel_open = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE, (open_kernel, open_kernel)
    )
    refined = cv2.morphologyEx(binary_mask, cv2.MORPH_CLOSE, kernel_close)
    refined = cv2.morphologyEx(refined, cv2.MORPH_OPEN, kernel_open)
    return refined


def build_morphological_skeleton(binary_mask: np.ndarray) -> np.ndarray:
    work = (binary_mask > 0).astype(np.uint8) * 255
    if work.size == 0 or cv2.countNonZero(work) == 0:
        return np.zeros_like(work)

    skel = np.zeros_like(work)
    element = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))
    while True:
        eroded = cv2.erode(work, element)
        opened = cv2.dilate(eroded, element)
        temp = cv2.subtract(work, opened)
        skel = cv2.bitwise_or(skel, temp)
        work = eroded
        if cv2.countNonZero(work) == 0:
            break
    return skel


def find_cutting_point_from_convexity_defect(
    binary_mask: np.ndarray, stem_center: Tuple[float, float]
) -> Tuple[float, float]:
    contours, _ = cv2.findContours(
        binary_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    if len(contours) == 0:
        return stem_center

    largest_contour = max(contours, key=cv2.contourArea)
    epsilon = 0.001 * cv2.arcLength(largest_contour, True)
    largest_contour = cv2.approxPolyDP(largest_contour, epsilon, True)
    hull = cv2.convexHull(largest_contour, returnPoints=False)
    try:
        defects = cv2.convexityDefects(largest_contour, hull)
        if defects is None:
            return stem_center
        cutting_points = []
        for i in range(defects.shape[0]):
            _, _, f, _ = defects[i, 0]
            far_point = tuple(largest_contour[f][0])
            cutting_points.append(far_point)
        if not cutting_points:
            return stem_center
        return min(
            cutting_points,
            key=lambda p: np.sqrt(
                (p[0] - stem_center[0]) ** 2 + (p[1] - stem_center[1]) ** 2
            ),
        )
    except cv2.error:
        return stem_center


def find_cutting_point(
    stem_mask: np.ndarray,
    stem_center: Tuple[float, float],
    original_image: np.ndarray,
    method: str = TARGET_CFG.point_method,
    close_kernel: int = TARGET_CFG.mask_close_kernel,
    open_kernel: int = TARGET_CFG.mask_open_kernel,
) -> Tuple[float, float]:
    del original_image

    ys, xs = np.where(stem_mask > 0)
    if len(xs) == 0:
        return stem_center

    x0, x1 = max(0, int(xs.min()) - 2), int(xs.max()) + 3
    y0, y1 = max(0, int(ys.min()) - 2), int(ys.max()) + 3
    cropped_mask = stem_mask[y0:y1, x0:x1]
    binary_mask = preprocess_selection_mask(cropped_mask, close_kernel, open_kernel)
    local_center = (stem_center[0] - x0, stem_center[1] - y0)

    if method == "centroid":
        return stem_center
    if method == "legacy_defect":
        px, py = find_cutting_point_from_convexity_defect(binary_mask, local_center)
        return float(px + x0), float(py + y0)

    skeleton = build_morphological_skeleton(binary_mask)
    skel_y, skel_x = np.where(skeleton > 0)
    if len(skel_x) == 0:
        px, py = find_cutting_point_from_convexity_defect(binary_mask, local_center)
        return float(px + x0), float(py + y0)

    dx = skel_x.astype(np.float64) - float(local_center[0])
    dy = skel_y.astype(np.float64) - float(local_center[1])
    idx = int(np.argmin(dx * dx + dy * dy))
    return float(skel_x[idx] + x0), float(skel_y[idx] + y0)


def trimmed_median(values: np.ndarray, trim_ratio: float) -> float:
    if values.size == 0:
        return 0.0
    values = np.sort(values.astype(np.float64))
    trim_ratio = float(max(0.0, min(0.45, trim_ratio)))
    trim_n = int(values.size * trim_ratio)
    if trim_n > 0 and values.size > (2 * trim_n):
        values = values[trim_n:-trim_n]
    return float(np.median(values))


def depth_to_pointcloud(
    u: int,
    v: int,
    depth_m: float,
    intrinsics: np.ndarray,
) -> Tuple[float, float, float]:
    fx, fy = intrinsics[0, 0], intrinsics[1, 1]
    cx, cy = intrinsics[0, 2], intrinsics[1, 2]
    x = (u - cx) * depth_m / fx
    y = (v - cy) * depth_m / fy
    z = depth_m
    return x, y, z


class ImageDisplayWidget(QLabel):
    selection_changed = pyqtSignal(tuple, object)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(UI_CFG.image_min_w, UI_CFG.image_min_h)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setStyleSheet("background-color: black;")
        self.setAlignment(Qt.AlignCenter)

        self._pixmap = None
        self._latest_cv_image = None
        self._img_points = []
        self._cutting_point_img = None
        self._is_polygon_closed = False
        self._last_render_ts = 0.0
        self._zoom = 1.0
        self._pan_x = 0.0
        self._pan_y = 0.0
        self._is_panning = False
        self._last_pan_pos = None
        self.point_method = TARGET_CFG.point_method
        self.mask_close_kernel = TARGET_CFG.mask_close_kernel
        self.mask_open_kernel = TARGET_CFG.mask_open_kernel

    def sync_pick_config(
        self,
        point_method: str,
        mask_close_kernel: int,
        mask_open_kernel: int,
    ) -> None:
        self.point_method = point_method
        self.mask_close_kernel = max(1, int(mask_close_kernel))
        self.mask_open_kernel = max(1, int(mask_open_kernel))

    def set_image(self, cv_img: np.ndarray) -> None:
        if cv_img is None:
            return
        now = time.monotonic()
        min_interval = 1.0 / max(1, UI_CFG.render_fps_limit)
        if (now - self._last_render_ts) < min_interval:
            return
        self._last_render_ts = now
        self._latest_cv_image = cv_img
        rgb_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_img.shape
        qt_img = QImage(rgb_img.data, w, h, ch * w, QImage.Format_RGB888)
        self._pixmap = QPixmap.fromImage(qt_img)
        self.update()

    def paintEvent(self, event):
        super().paintEvent(event)
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        if self._pixmap:
            x, y, disp_w, disp_h = self._get_view_transform()
            scaled_pix = self._pixmap.scaled(
                int(disp_w), int(disp_h), Qt.IgnoreAspectRatio, Qt.SmoothTransformation
            )
            painter.drawPixmap(int(x), int(y), scaled_pix)

        if len(self._img_points) > 0:
            screen_points = [self._img_to_screen(x, y) for x, y in self._img_points]
            pen = QPen(QColor(0, 255, 0), 3, Qt.SolidLine)
            painter.setPen(pen)
            for i in range(len(screen_points) - 1):
                painter.drawLine(screen_points[i], screen_points[i + 1])

            if len(screen_points) >= 3 or self._is_polygon_closed:
                if len(screen_points) >= 3:
                    painter.drawLine(screen_points[-1], screen_points[0])
                    painter.setBrush(QBrush(QColor(0, 255, 0, 80)))
                    painter.drawPolygon(QPolygonF(screen_points))
                    painter.setBrush(Qt.NoBrush)

            pen_point = QPen(QColor(255, 0, 0), 12, Qt.SolidLine)
            pen_point.setCapStyle(Qt.RoundCap)
            painter.setPen(pen_point)
            for p in screen_points:
                painter.drawPoint(p)

        if self._cutting_point_img:
            cut_pt = self._img_to_screen(
                self._cutting_point_img[0], self._cutting_point_img[1]
            )
            pen_cut = QPen(QColor(255, 0, 255), 15, Qt.SolidLine)
            pen_cut.setCapStyle(Qt.RoundCap)
            painter.setPen(pen_cut)
            painter.drawPoint(cut_pt)

    def _get_view_transform(self, zoom=None, pan_x=None, pan_y=None):
        if not self._pixmap:
            return 0.0, 0.0, 1.0, 1.0
        zoom = self._zoom if zoom is None else zoom
        pan_x = self._pan_x if pan_x is None else pan_x
        pan_y = self._pan_y if pan_y is None else pan_y

        base_scale = min(
            self.width() / max(1, self._pixmap.width()),
            self.height() / max(1, self._pixmap.height()),
        )
        disp_w = max(1.0, self._pixmap.width() * base_scale * zoom)
        disp_h = max(1.0, self._pixmap.height() * base_scale * zoom)
        x = (self.width() - disp_w) / 2.0 + pan_x
        y = (self.height() - disp_h) / 2.0 + pan_y
        return x, y, disp_w, disp_h

    def _screen_to_img(self, pt: QPoint) -> Tuple[float, float]:
        if self._pixmap is None:
            return 0.0, 0.0
        x, y, disp_w, disp_h = self._get_view_transform()
        img_x = np.clip(
            (pt.x() - x) / disp_w * self._pixmap.width(), 0, self._pixmap.width() - 1
        )
        img_y = np.clip(
            (pt.y() - y) / disp_h * self._pixmap.height(), 0, self._pixmap.height() - 1
        )
        return img_x, img_y

    def _img_to_screen(self, x: float, y: float) -> QPoint:
        if self._pixmap is None:
            return QPoint()
        view_x, view_y, disp_w, disp_h = self._get_view_transform()
        return QPoint(
            int(x / self._pixmap.width() * disp_w + view_x),
            int(y / self._pixmap.height() * disp_h + view_y),
        )

    def wheelEvent(self, event):
        if self._pixmap is None:
            return
        angle = event.angleDelta().y()
        if angle == 0:
            return

        factor = 1.1 if angle > 0 else 1 / 1.1
        old_zoom = self._zoom
        new_zoom = max(1.0, min(8.0, old_zoom * factor))
        if abs(new_zoom - old_zoom) < 1e-6:
            return

        cursor_pos = event.pos()
        img_x, img_y = self._screen_to_img(cursor_pos)
        self._zoom = new_zoom
        mapped_pos = self._img_to_screen(img_x, img_y)
        self._pan_x += cursor_pos.x() - mapped_pos.x()
        self._pan_y += cursor_pos.y() - mapped_pos.y()
        self.update()
        event.accept()

    def mousePressEvent(self, event):
        if event.button() == Qt.RightButton:
            self._is_panning = True
            self._last_pan_pos = event.pos()
            self.setCursor(Qt.ClosedHandCursor)
            return

        if self._is_polygon_closed:
            self.clear_selection()

        if event.button() == Qt.LeftButton:
            self._img_points.append(self._screen_to_img(event.pos()))
            self._cutting_point_img = None
            self.update()
            if len(self._img_points) >= 3:
                self._update_mask_and_calculate()

    def mouseMoveEvent(self, event):
        if self._is_panning and self._last_pan_pos is not None:
            delta = event.pos() - self._last_pan_pos
            self._pan_x += delta.x()
            self._pan_y += delta.y()
            self._last_pan_pos = event.pos()
            self.update()

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.RightButton:
            self._is_panning = False
            self._last_pan_pos = None
            self.unsetCursor()

    def mouseDoubleClickEvent(self, event):
        if event.button() == Qt.LeftButton and len(self._img_points) >= 3:
            self._is_polygon_closed = True
            self.update()
            self._update_mask_and_calculate()

    def _update_mask_and_calculate(self) -> None:
        if len(self._img_points) < 3 or self._latest_cv_image is None:
            return

        img_pts_np = np.array(self._img_points, dtype=np.int32)
        h, w = self._latest_cv_image.shape[:2]
        mask = np.zeros((h, w), dtype=np.uint8)
        cv2.fillPoly(mask, [img_pts_np], 255)

        stem_center = get_mask_center(mask)
        if stem_center is None:
            return

        cutting_point = find_cutting_point(
            mask,
            stem_center,
            self._latest_cv_image,
            self.point_method,
            self.mask_close_kernel,
            self.mask_open_kernel,
        )
        if cutting_point is None:
            return

        self._cutting_point_img = cutting_point
        self.update()
        self.selection_changed.emit(cutting_point, mask)

    def clear_selection(self) -> None:
        self._img_points = []
        self._cutting_point_img = None
        self._is_polygon_closed = False
        self.update()

    def reset_view(self) -> None:
        self._zoom = 1.0
        self._pan_x = 0.0
        self._pan_y = 0.0
        self._is_panning = False
        self._last_pan_pos = None
        self.unsetCursor()
        self.update()


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle(UI_CFG.window_title)
        self.setGeometry(
            UI_CFG.window_x, UI_CFG.window_y, UI_CFG.window_w, UI_CFG.window_h
        )

        self.current_depth_data = None
        self.current_depth_raw_data = None
        self.current_depth_registered_data = None
        self.current_color_size = None
        self.current_depth_intrinsics = None
        self.current_depth_raw_intrinsics = None
        self.current_depth_registered_intrinsics = None
        self.current_color_intrinsics = None
        self.current_projection_mode = "invalid"
        self.current_depth_frame_id = ROS_CFG.camera_color_frame
        self.current_lrm_mm = 0
        self.current_lrm_m = 0.0
        self.current_cutting_pixel = None

        self.target_tcp_coord = None
        self.raw_target_tcp_coord = None
        self.last_cam_coord = None
        self.last_flange_coord = None

        self.ros_available = False
        self.pick_action_available = False
        self.pick_client = None
        self.simple_move_client = None
        self.simple_move_available = False
        self.last_request_type = None

        self.bridge = CvBridge()
        self.latest_color_image = None
        self.latest_color_size = None

        self.init_ros()

        self.tf_buffer = None
        self.tf_listener = None
        if self.ros_available:
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.init_ui()
        self.init_camera_subscribers()

        self.camera_render_timer = QTimer(self)
        self.camera_render_timer.timeout.connect(self.flush_latest_frame)
        self.camera_render_timer.start(
            max(10, int(1000 / max(1, UI_CFG.render_fps_limit)))
        )

        self._toggle_window_shortcut = QShortcut(QKeySequence("F11"), self)
        self._toggle_window_shortcut.activated.connect(self.toggle_window_state)
        self._reset_view_shortcut = QShortcut(QKeySequence("R"), self)
        self._reset_view_shortcut.activated.connect(self.video_label.reset_view)

    def toggle_window_state(self) -> None:
        if self.isMaximized() or self.isFullScreen():
            self.showNormal()
        else:
            self.showMaximized()

    def init_ros(self) -> None:
        try:
            if not rospy.core.is_initialized():
                rospy.init_node(ROS_CFG.node_name, anonymous=True, disable_signals=True)
            self.ros_available = True

            self.pick_client = actionlib.SimpleActionClient(
                ROS_CFG.action_name, PickTaskAction
            )
            self.pick_action_available = self.pick_client.wait_for_server(
                rospy.Duration(ROS_CFG.action_wait_sec)
            )
            if not self.pick_action_available:
                print(
                    f"[ROS] 警告: {ROS_CFG.action_name} action 不可用，写入/执行任务按钮将不可用"
                )

            self.simple_move_client = actionlib.SimpleActionClient(
                ROS_CFG.simple_move_action_name, SimpleMoveArmAction
            )
            self.simple_move_available = self.simple_move_client.wait_for_server(
                rospy.Duration(ROS_CFG.action_wait_sec)
            )
            if not self.simple_move_available:
                print(
                    f"[ROS] 警告: {ROS_CFG.simple_move_action_name} action 不可用，返回安全区按钮将不可用"
                )

            print(f"[ROS] 节点初始化成功: {ROS_CFG.node_name}")
        except Exception as e:
            print(f"[ROS] 初始化失败: {e}")
            self.ros_available = False
            self.pick_action_available = False
            self.simple_move_available = False

    def flange_point_to_tcp_point(
        self,
        x_flange: float,
        y_flange: float,
        z_flange: float,
    ) -> Tuple[float, float, float]:
        if not self.ros_available or self.tf_buffer is None:
            raise RuntimeError("ROS/TF 未初始化，无法执行 flange -> tcp 变换")

        pt_flange = PointStamped()
        pt_flange.header.frame_id = ROS_CFG.flange_frame
        pt_flange.header.stamp = rospy.Time(0)
        pt_flange.point.x = x_flange
        pt_flange.point.y = y_flange
        pt_flange.point.z = z_flange

        pt_tcp = self.tf_buffer.transform(
            pt_flange, ROS_CFG.tcp_frame, rospy.Duration(ROS_CFG.tf_wait_sec)
        )
        return pt_tcp.point.x, pt_tcp.point.y, pt_tcp.point.z

    def send_arm_command(self, x: float, y: float, z: float) -> bool:
        del x, y, z
        self.upsert_current_task()
        return True

    def init_ui(self) -> None:
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)

        self.video_label = ImageDisplayWidget()
        self.video_label.selection_changed.connect(self.on_roi_selected)
        main_layout.addWidget(self.video_label, stretch=3)

        self.side_scroll = QScrollArea()
        self.side_scroll.setWidgetResizable(True)
        self.side_panel = QWidget()
        panel_layout = QHBoxLayout(self.side_panel)
        panel_layout.setContentsMargins(0, 0, 0, 0)
        self.side_scroll.setWidget(self.side_panel)
        main_layout.addWidget(self.side_scroll, stretch=2)

        info_column = QVBoxLayout()
        control_column = QVBoxLayout()
        panel_layout.addLayout(info_column, stretch=1)
        panel_layout.addLayout(control_column, stretch=1)

        info_group = QGroupBox("目标锁定 / 坐标信息")
        info_group_layout = QVBoxLayout(info_group)
        self.info_label = QLabel(
            "系统初始化中...\n\n操作说明：\n1. 左键绘制目标区域\n2. 双击左键闭合区域\n3. 鼠标滚轮缩放，右键拖拽平移\n4. R 键重置视图\n"
        )
        self.info_label.setWordWrap(True)
        self.info_label.setFont(QFont(UI_CFG.info_font_name, UI_CFG.info_font_size))
        self.info_label.setTextFormat(Qt.RichText)
        self.info_label.setAlignment(Qt.AlignTop | Qt.AlignLeft)
        info_group_layout.addWidget(self.info_label)
        info_group.setMinimumWidth(300)
        info_column.addWidget(info_group)
        info_column.addStretch()

        self.task_status_label = QLabel("采摘任务状态：未发送")
        self.task_status_label.setWordWrap(True)
        self.task_status_label.setFont(
            QFont(UI_CFG.info_font_name, UI_CFG.info_font_size)
        )
        control_column.addWidget(self.task_status_label)

        control_tabs = QTabWidget()
        control_column.addWidget(control_tabs)

        config_tab = QWidget()
        config_layout = QVBoxLayout(config_tab)

        action_tab = QWidget()
        action_layout = QVBoxLayout(action_tab)

        control_tabs.addTab(config_tab, "参数配置")
        control_tabs.addTab(action_tab, "操作执行")

        task_group = QGroupBox("任务组配置")
        task_layout = QFormLayout(task_group)
        self.group_name_edit = QLineEdit(ROS_CFG.pick_group_name)
        self.task_id_edit = QLineEdit("1")
        self.task_desc_edit = QLineEdit("GUI采摘任务")
        self.task_type_box = QComboBox()
        self.task_type_box.addItem("PICK", PickTaskGoal.TASK_PICK)
        self.task_type_box.addItem("MOVE_ONLY", PickTaskGoal.TASK_MOVE_ONLY)
        self.group_sort_type_box = QComboBox()
        self.group_sort_type_box.addItem("按任务 ID 排序", PickTaskGoal.GROUP_SORT_ID)
        self.group_sort_type_box.addItem("按距离排序", PickTaskGoal.GROUP_SORT_DIST)
        self.group_weight_orient_edit = QLineEdit("0.30")
        self.group_weight_orient_edit.setPlaceholderText("0.0 ~ 1.0")
        self.retry_times_edit = QLineEdit("0")
        self.chk_use_eef = QCheckBox("执行夹爪动作")
        self.chk_use_eef.setChecked(True)
        self.chk_go_home_after_finish = QCheckBox("任务组完成后回到初始位")
        self.chk_go_home_after_finish.setChecked(True)
        self.chk_go_safe_after_cancel = QCheckBox("取消后自动回安全位")
        self.chk_go_safe_after_cancel.setChecked(True)
        task_layout.addRow("任务组", self.group_name_edit)
        task_layout.addRow("任务组排序", self.group_sort_type_box)
        task_layout.addRow("姿态权重", self.group_weight_orient_edit)
        task_layout.addRow(self.chk_go_home_after_finish)
        task_layout.addRow("任务 ID", self.task_id_edit)
        task_layout.addRow("任务类型", self.task_type_box)
        task_layout.addRow("描述", self.task_desc_edit)
        task_layout.addRow("重试次数（不含第一次执行）", self.retry_times_edit)
        task_layout.addRow(self.chk_use_eef)
        task_layout.addRow(self.chk_go_safe_after_cancel)
        config_layout.addWidget(task_group)

        place_group = QGroupBox("放置区配置（base_link）")
        place_layout = QFormLayout(place_group)
        self.chk_use_place_pose = QCheckBox("启用放置动作")
        self.chk_use_place_pose.setChecked(True)
        self.place_target_type_box = QComboBox()
        self.place_target_type_box.addItem(
            "Point(base_link)", PickTaskGoal.PLACE_TARGET_POINT
        )
        self.place_target_type_box.addItem(
            "Pose(base_link)", PickTaskGoal.PLACE_TARGET_POSE
        )
        self.place_x_edit = QLineEdit("0.00")
        self.place_y_edit = QLineEdit("0.20")
        self.place_z_edit = QLineEdit("0.20")
        self.place_roll_edit = QLineEdit("0.00")
        self.place_pitch_edit = QLineEdit("0.00")
        self.place_yaw_edit = QLineEdit("0.00")
        place_layout.addRow(self.chk_use_place_pose)
        place_layout.addRow("放置目标类型", self.place_target_type_box)
        place_layout.addRow("X", self.place_x_edit)
        place_layout.addRow("Y", self.place_y_edit)
        place_layout.addRow("Z", self.place_z_edit)
        place_layout.addRow("Roll", self.place_roll_edit)
        place_layout.addRow("Pitch", self.place_pitch_edit)
        place_layout.addRow("Yaw", self.place_yaw_edit)
        config_layout.addWidget(place_group)

        target_group = QGroupBox("取点 / TCP补偿")
        target_layout = QFormLayout(target_group)
        self.point_method_box = QComboBox()
        self.point_method_box.addItem("中心线最近质心", "skeleton_centroid")
        self.point_method_box.addItem("区域质心", "centroid")
        self.point_method_box.addItem("轮廓凹陷兼容", "legacy_defect")
        self.mask_close_kernel_edit = QLineEdit(str(TARGET_CFG.mask_close_kernel))
        self.mask_open_kernel_edit = QLineEdit(str(TARGET_CFG.mask_open_kernel))
        self.depth_kernel_edit = QLineEdit(str(TARGET_CFG.depth_kernel_size))
        self.depth_trim_ratio_edit = QLineEdit(f"{TARGET_CFG.depth_trim_ratio:.2f}")
        self.chk_prefer_lrm_override = QCheckBox("近距优先使用 LRM")
        self.chk_prefer_lrm_override.setChecked(TARGET_CFG.prefer_lrm_override)
        self.chk_enable_tcp_comp = QCheckBox("启用 TCP 平移补偿")
        self.chk_enable_tcp_comp.setChecked(RESIDUAL_COMP_CFG.enabled)
        self.tcp_comp_x_edit = QLineEdit(f"{RESIDUAL_COMP_CFG.dx:.4f}")
        self.tcp_comp_y_edit = QLineEdit(f"{RESIDUAL_COMP_CFG.dy:.4f}")
        self.tcp_comp_z_edit = QLineEdit(f"{RESIDUAL_COMP_CFG.dz:.4f}")
        target_layout.addRow("取点方式", self.point_method_box)
        target_layout.addRow("闭运算核", self.mask_close_kernel_edit)
        target_layout.addRow("开运算核", self.mask_open_kernel_edit)
        target_layout.addRow("深度核尺寸", self.depth_kernel_edit)
        target_layout.addRow("深度裁剪比例", self.depth_trim_ratio_edit)
        target_layout.addRow(self.chk_prefer_lrm_override)
        target_layout.addRow(self.chk_enable_tcp_comp)
        target_layout.addRow("补偿 X", self.tcp_comp_x_edit)
        target_layout.addRow("补偿 Y", self.tcp_comp_y_edit)
        target_layout.addRow("补偿 Z", self.tcp_comp_z_edit)
        config_layout.addWidget(target_group)
        config_layout.addStretch()

        self.btn_reset = QPushButton("重置选择")
        self.btn_reset.clicked.connect(self.video_label.clear_selection)
        self.btn_reset.setMinimumHeight(40)
        self.btn_reset.setStyleSheet(
            "background-color: #3498db; color: white; font-size:14px; font-weight:bold;"
        )
        action_layout.addWidget(self.btn_reset)

        self.btn_update_group_config = QPushButton("仅更新任务组配置")
        self.btn_update_group_config.clicked.connect(self.update_task_group_config)
        self.btn_update_group_config.setMinimumHeight(40)
        self.btn_update_group_config.setStyleSheet(
            "background-color: #16a085; color: white; font-size:14px; font-weight:bold;"
        )
        self.btn_update_group_config.setEnabled(self.pick_action_available)
        action_layout.addWidget(self.btn_update_group_config)

        self.btn_upsert_task = QPushButton("写入 / 更新当前任务")
        self.btn_upsert_task.clicked.connect(self.manual_send_command)
        self.btn_upsert_task.setMinimumHeight(40)
        self.btn_upsert_task.setStyleSheet(
            "background-color: #27ae60; color: white; font-size:14px; font-weight:bold;"
        )
        self.btn_upsert_task.setEnabled(False)
        action_layout.addWidget(self.btn_upsert_task)

        self.btn_execute_group = QPushButton("执行当前任务组")
        self.btn_execute_group.clicked.connect(self.execute_current_group)
        self.btn_execute_group.setMinimumHeight(40)
        self.btn_execute_group.setStyleSheet(
            "background-color: #8e44ad; color: white; font-size:14px; font-weight:bold;"
        )
        self.btn_execute_group.setEnabled(self.pick_action_available)
        action_layout.addWidget(self.btn_execute_group)

        self.btn_cancel_cmd = QPushButton("取消当前执行")
        self.btn_cancel_cmd.clicked.connect(self.cancel_pick_task)
        self.btn_cancel_cmd.setMinimumHeight(40)
        self.btn_cancel_cmd.setStyleSheet(
            "background-color: #c0392b; color: white; font-size:14px; font-weight:bold;"
        )
        action_layout.addWidget(self.btn_cancel_cmd)

        self.btn_go_home = QPushButton("返回安全区")
        self.btn_go_home.clicked.connect(self.go_home_to_safe)
        self.btn_go_home.setMinimumHeight(40)
        self.btn_go_home.setStyleSheet(
            "background-color: #d35400; color: white; font-size:14px; font-weight:bold;"
        )
        self.btn_go_home.setEnabled(self.ros_available and self.simple_move_available)
        action_layout.addWidget(self.btn_go_home)

        action_layout.addStretch()
        control_column.addStretch()
        self._sync_pick_config_to_view()
        self.point_method_box.currentIndexChanged.connect(
            self._sync_pick_config_to_view
        )
        self.mask_close_kernel_edit.editingFinished.connect(
            self._sync_pick_config_to_view
        )
        self.mask_open_kernel_edit.editingFinished.connect(
            self._sync_pick_config_to_view
        )

    def _sync_pick_config_to_view(self) -> None:
        self.video_label.sync_pick_config(
            str(self.point_method_box.currentData()),
            self._parse_int(self.mask_close_kernel_edit, TARGET_CFG.mask_close_kernel),
            self._parse_int(self.mask_open_kernel_edit, TARGET_CFG.mask_open_kernel),
        )

    def init_camera_subscribers(self) -> None:
        if not self.ros_available:
            return

        rospy.Subscriber(
            ROS_CFG.camera_color_topic,
            Image,
            self.on_color_image,
            queue_size=1,
            buff_size=2**24,
        )
        rospy.Subscriber(
            ROS_CFG.camera_color_info_topic,
            CameraInfo,
            self.on_color_camera_info,
            queue_size=1,
        )
        rospy.Subscriber(
            ROS_CFG.camera_depth_raw_topic,
            Image,
            self.on_depth_raw_image,
            queue_size=1,
            buff_size=2**24,
        )
        rospy.Subscriber(
            ROS_CFG.camera_depth_raw_info_topic,
            CameraInfo,
            self.on_depth_raw_camera_info,
            queue_size=1,
        )
        rospy.Subscriber(
            ROS_CFG.camera_depth_registered_topic,
            Image,
            self.on_depth_registered_image,
            queue_size=1,
            buff_size=2**24,
        )
        rospy.Subscriber(
            ROS_CFG.camera_depth_registered_info_topic,
            CameraInfo,
            self.on_depth_registered_camera_info,
            queue_size=1,
        )
        rospy.Subscriber(
            ROS_CFG.camera_lrm_topic,
            Float32,
            self.on_lrm_distance,
            queue_size=1,
        )

    def on_color_image(self, msg: Image) -> None:
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.latest_color_image = image.copy()
            self.latest_color_size = (image.shape[1], image.shape[0])
            self.current_color_size = self.latest_color_size
        except Exception as e:
            print(f"[GUI] 彩色图转换失败: {e}")

    def flush_latest_frame(self) -> None:
        if self.latest_color_image is None:
            return
        self.video_label.set_image(self.latest_color_image)

    def on_color_camera_info(self, msg: CameraInfo) -> None:
        try:
            self.current_color_intrinsics = np.array(msg.K, dtype=np.float64).reshape(
                3, 3
            )
            if msg.width > 0 and msg.height > 0:
                self.current_color_size = (msg.width, msg.height)
        except Exception as e:
            print(f"[GUI] 彩色相机内参解析失败: {e}")

    def on_depth_raw_image(self, msg: Image) -> None:
        try:
            depth_m = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
            if depth_m is None:
                return
            self.current_depth_raw_data = depth_m.astype(np.float32) * 1000.0
        except Exception as e:
            print(f"[GUI] 原始深度图转换失败: {e}")

    def on_depth_raw_camera_info(self, msg: CameraInfo) -> None:
        try:
            self.current_depth_raw_intrinsics = np.array(
                msg.K, dtype=np.float64
            ).reshape(3, 3)
        except Exception as e:
            print(f"[GUI] 原始深度内参解析失败: {e}")

    def on_depth_registered_image(self, msg: Image) -> None:
        try:
            depth_m = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
            if depth_m is None:
                return
            self.current_depth_registered_data = depth_m.astype(np.float32) * 1000.0
            self.current_depth_data = self.current_depth_registered_data
            self.current_depth_frame_id = (
                msg.header.frame_id or ROS_CFG.camera_color_frame
            )
            if self.current_depth_registered_intrinsics is not None:
                self.current_depth_intrinsics = self.current_depth_registered_intrinsics
            self.current_projection_mode = "depth_registered"
        except Exception as e:
            print(f"[GUI] 对齐深度图转换失败: {e}")

    def on_depth_registered_camera_info(self, msg: CameraInfo) -> None:
        try:
            self.current_depth_registered_intrinsics = np.array(
                msg.K, dtype=np.float64
            ).reshape(3, 3)
            self.current_depth_intrinsics = self.current_depth_registered_intrinsics
            self.current_depth_frame_id = (
                msg.header.frame_id or ROS_CFG.camera_color_frame
            )
            self.current_projection_mode = "depth_registered"
        except Exception as e:
            print(f"[GUI] 对齐深度内参解析失败: {e}")

    def on_lrm_distance(self, msg: Float32) -> None:
        try:
            self.current_lrm_m = float(msg.data)
            self.current_lrm_mm = int(round(self.current_lrm_m * 1000.0))
        except Exception:
            self.current_lrm_m = 0.0
            self.current_lrm_mm = 0

    def get_valid_depth_around(
        self,
        depth_img: np.ndarray,
        u: int,
        v: int,
        stem_mask: np.ndarray,
        kernel_size: int = DEPTH_CFG.kernel_size,
        trim_ratio: float = TARGET_CFG.depth_trim_ratio,
    ) -> float:
        h, w = depth_img.shape
        if stem_mask.shape[:2] != (h, w):
            stem_mask = cv2.resize(stem_mask, (w, h), interpolation=cv2.INTER_NEAREST)

        binary_mask = preprocess_selection_mask(
            stem_mask,
            self._parse_int(self.mask_close_kernel_edit, TARGET_CFG.mask_close_kernel),
            self._parse_int(self.mask_open_kernel_edit, TARGET_CFG.mask_open_kernel),
        )
        kernel_size = max(3, int(kernel_size))
        if kernel_size % 2 == 0:
            kernel_size += 1
        half_k = kernel_size // 2
        start_x, end_x = max(0, u - half_k), min(w, u + half_k + 1)
        start_y, end_y = max(0, v - half_k), min(h, v + half_k + 1)

        distance_map = cv2.distanceTransform(binary_mask, cv2.DIST_L2, 5)
        local_radius = float(distance_map[v, u]) if 0 <= v < h and 0 <= u < w else 0.0
        center_threshold = max(TARGET_CFG.center_band_min_dist_px, local_radius * 0.5)
        center_mask = (distance_map >= center_threshold).astype(np.uint8) * 255
        erode_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        eroded_mask = cv2.erode(binary_mask, erode_kernel, iterations=1)

        masks_to_try = [center_mask, eroded_mask, binary_mask]
        for candidate_mask in masks_to_try:
            depth_roi = depth_img[start_y:end_y, start_x:end_x]
            mask_roi = candidate_mask[start_y:end_y, start_x:end_x]
            valid_mask = (mask_roi > 0) & (depth_roi > 0)
            valid_depths = depth_roi[valid_mask]
            if valid_depths.size >= 3:
                return trimmed_median(valid_depths, trim_ratio)

        full_valid_depths = depth_img[(binary_mask > 0) & (depth_img > 0)]
        return (
            trimmed_median(full_valid_depths, trim_ratio)
            if full_valid_depths.size > 0
            else 0.0
        )

    def _get_depth_value_mm(self, depth_raw: float) -> Tuple[float, bool]:
        use_lrm = False
        if (
            self.chk_prefer_lrm_override.isChecked()
            and LRM_CFG.valid_min_mm <= self.current_lrm_mm <= LRM_CFG.valid_max_mm
        ):
            depth_raw = float(self.current_lrm_mm)
            use_lrm = True
        return depth_raw, use_lrm

    def _apply_tcp_residual_compensation(
        self, tcp_xyz: Tuple[float, float, float]
    ) -> Tuple[float, float, float]:
        if not self.chk_enable_tcp_comp.isChecked():
            return tcp_xyz
        dx = self._parse_float(self.tcp_comp_x_edit, 0.0)
        dy = self._parse_float(self.tcp_comp_y_edit, 0.0)
        dz = self._parse_float(self.tcp_comp_z_edit, 0.0)
        return tcp_xyz[0] + dx, tcp_xyz[1] + dy, tcp_xyz[2] + dz

    def _update_info_label(
        self,
        u: float,
        v: float,
        depth_m: float,
        use_lrm: bool,
        cam_xyz: Tuple[float, float, float],
        flange_xyz: Tuple[float, float, float],
        raw_tcp_xyz: Tuple[float, float, float],
        tcp_xyz: Tuple[float, float, float],
    ) -> None:
        ros_status = "已连接" if self.ros_available else "未连接"
        lrm_note = " (LRM)" if use_lrm else ""
        x_cam, y_cam, z_cam = cam_xyz
        x_flange, y_flange, z_flange = flange_xyz
        x_tcp_raw, y_tcp_raw, z_tcp_raw = raw_tcp_xyz
        x_tcp, y_tcp, z_tcp = tcp_xyz
        point_method = self.point_method_box.currentText()

        self.info_label.setText(f"""
            <h3>目标已锁定{lrm_note}</h3>
            <p><b>ROS:</b> {ros_status}</p>
            <p><b>D2C模式:</b> {self.current_projection_mode}</p>
            <p><b>取点方式:</b> {point_method}</p>
            <p><b>像素:</b> ({u:.1f}, {v:.1f})</p>
            <p><b>深度:</b> {depth_m:.3f} m</p>
            <hr>
            <p><b>Camera:</b> {x_cam:.4f}, {y_cam:.4f}, {z_cam:.4f}</p>
            <p><b>Flange:</b> {x_flange:.4f}, {y_flange:.4f}, {z_flange:.4f}</p>
            <p><b>TCP 原始:</b> {x_tcp_raw:.4f}, {y_tcp_raw:.4f}, {z_tcp_raw:.4f}</p>
            <p><b>TCP 发送:</b> {x_tcp:.4f}, {y_tcp:.4f}, {z_tcp:.4f}</p>
        """)

    def manual_send_command(self) -> None:
        self.upsert_current_task()

    def _parse_float(self, edit: QLineEdit, default: float = 0.0) -> float:
        try:
            return float(edit.text().strip())
        except Exception:
            return default

    def _parse_int(self, edit: QLineEdit, default: int = 0) -> int:
        try:
            return int(edit.text().strip())
        except Exception:
            return default

    def _build_pose_stamped(
        self, frame_id: str, x: float, y: float, z: float
    ) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = rospy.Time.now() if self.ros_available else rospy.Time(0)
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.w = 1.0
        return pose

    def _fill_point_msg(self, point_msg, x: float, y: float, z: float) -> None:
        point_msg.x = x
        point_msg.y = y
        point_msg.z = z

    def _fill_pose_msg(
        self,
        pose_msg,
        x: float,
        y: float,
        z: float,
        roll: float = 0.0,
        pitch: float = 0.0,
        yaw: float = 0.0,
    ) -> None:
        qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw)
        pose_msg.position.x = x
        pose_msg.position.y = y
        pose_msg.position.z = z
        pose_msg.orientation.x = qx
        pose_msg.orientation.y = qy
        pose_msg.orientation.z = qz
        pose_msg.orientation.w = qw

    def _send_goal(self, goal: PickTaskGoal) -> None:
        self.pick_client.send_goal(
            goal,
            done_cb=self._on_pick_done,
            active_cb=self._on_pick_active,
            feedback_cb=self._on_pick_feedback,
        )

    def _fill_group_config(self, goal: PickTaskGoal) -> None:
        goal.group_sort_type = int(self.group_sort_type_box.currentData())
        goal.group_dist_weight_orient = float(
            max(0.0, min(1.0, self._parse_float(self.group_weight_orient_edit, 0.3)))
        )
        goal.group_go_home_after_finish = self.chk_go_home_after_finish.isChecked()

    def update_task_group_config(self) -> None:
        if not self.pick_action_available or self.pick_client is None:
            self.task_status_label.setText("采摘任务状态：ROS 未连接")
            return

        goal = PickTaskGoal()
        goal.request_type = PickTaskGoal.UPDATE_TASK_GROUP_CONFIG
        goal.group_name = self.group_name_edit.text().strip() or ROS_CFG.pick_group_name
        self._fill_group_config(goal)

        self.last_request_type = goal.request_type
        self.task_status_label.setText(
            f"采摘任务状态：正在更新任务组 '{goal.group_name}' 的配置"
        )
        self._send_goal(goal)

    def upsert_current_task(self) -> None:
        if not self.pick_action_available or self.pick_client is None:
            self.task_status_label.setText("采摘任务状态：ROS 未连接")
            return

        if self.target_tcp_coord is None:
            self.task_status_label.setText("采摘任务状态：请先选择目标")
            return

        x, y, z = self.target_tcp_coord
        goal = PickTaskGoal()
        goal.request_type = PickTaskGoal.UPSERT_TASK
        goal.group_name = self.group_name_edit.text().strip() or ROS_CFG.pick_group_name
        goal.id = max(0, self._parse_int(self.task_id_edit, 1))
        goal.task_type = int(self.task_type_box.currentData())
        goal.description = self.task_desc_edit.text().strip() or "GUI采摘任务"
        self._fill_group_config(goal)
        goal.target_type = PickTaskGoal.TARGET_POINT
        self._fill_point_msg(goal.target_point, x, y, z)
        goal.target_frame_id = ROS_CFG.tcp_frame

        goal.use_place_pose = self.chk_use_place_pose.isChecked()
        goal.use_eef = self.chk_use_eef.isChecked()
        goal.go_safe_after_cancel = self.chk_go_safe_after_cancel.isChecked()
        goal.retry_times = max(0, self._parse_int(self.retry_times_edit, 0))
        goal.retry_times = min(goal.retry_times, 255)

        if goal.task_type == PickTaskGoal.TASK_PICK and goal.use_place_pose:
            place_target_type = int(self.place_target_type_box.currentData())
            goal.place_target_type = place_target_type
            goal.place_frame_id = ROS_CFG.place_frame

            px = self._parse_float(self.place_x_edit, 0.0)
            py = self._parse_float(self.place_y_edit, 0.0)
            pz = self._parse_float(self.place_z_edit, 0.0)
            proll = self._parse_float(self.place_roll_edit, 0.0)
            ppitch = self._parse_float(self.place_pitch_edit, 0.0)
            pyaw = self._parse_float(self.place_yaw_edit, 0.0)

            if place_target_type == PickTaskGoal.PLACE_TARGET_POSE:
                self._fill_pose_msg(goal.place_pose, px, py, pz, proll, ppitch, pyaw)
            else:
                goal.place_target_type = PickTaskGoal.PLACE_TARGET_POINT
                self._fill_point_msg(goal.place_point, px, py, pz)
        else:
            goal.use_place_pose = False
            goal.place_target_type = PickTaskGoal.PLACE_TARGET_NONE

        self.last_request_type = goal.request_type
        self.task_status_label.setText(
            f"采摘任务状态：正在写入任务组 '{goal.group_name}'，任务 ID={goal.id}，目标=({x:.3f}, {y:.3f}, {z:.3f})"
        )
        self._send_goal(goal)

    def execute_current_group(self) -> None:
        if not self.pick_action_available or self.pick_client is None:
            self.task_status_label.setText("采摘任务状态：ROS 未连接")
            return

        goal = PickTaskGoal()
        goal.request_type = PickTaskGoal.EXECUTE_TASK_GROUP
        goal.group_name = self.group_name_edit.text().strip() or ROS_CFG.pick_group_name
        self._fill_group_config(goal)
        goal.use_eef = self.chk_use_eef.isChecked()
        goal.go_safe_after_cancel = self.chk_go_safe_after_cancel.isChecked()
        goal.retry_times = max(0, self._parse_int(self.retry_times_edit, 0))
        goal.retry_times = min(goal.retry_times, 255)
        self.last_request_type = goal.request_type
        self.task_status_label.setText(
            f"采摘任务状态：已发送任务组执行请求，任务组='{goal.group_name}'"
        )
        self._send_goal(goal)

    def send_pick_task(self) -> None:
        self.upsert_current_task()

    def cancel_pick_task(self) -> None:
        if self.pick_action_available and self.pick_client is not None:
            self.pick_client.cancel_all_goals()
            self.task_status_label.setText("采摘任务状态：已请求取消")

    def go_home_to_safe(self) -> None:
        if (
            not self.ros_available
            or self.simple_move_client is None
            or not self.simple_move_available
        ):
            self.task_status_label.setText(
                "采摘任务状态：返回安全区失败，/simple_move_arm 未连接"
            )
            return

        goal = SimpleMoveArmGoal()
        goal.command_type = SimpleMoveArmGoal.MOVE_TO_ZERO
        goal.target_type = SimpleMoveArmGoal.TARGET_POSE
        goal.x = [0.0]
        goal.y = [0.0]
        goal.z = [0.0]
        goal.roll = [0.0]
        goal.pitch = [0.0]
        goal.yaw = [0.0]

        self.task_status_label.setText("采摘任务状态：正在返回安全区")
        self.simple_move_client.send_goal(goal, done_cb=self._on_go_home_done)

    def _on_go_home_done(self, state, result) -> None:
        success = bool(getattr(result, "success", False))
        msg = getattr(result, "message", "")
        if success:
            self.task_status_label.setText("采摘任务状态：返回安全区成功")
            return

        if msg:
            self.task_status_label.setText(f"采摘任务状态：返回安全区失败 - {msg}")
            return

        self.task_status_label.setText(f"采摘任务状态：返回安全区失败，state={state}")

    def _on_pick_active(self) -> None:
        if self.last_request_type == PickTaskGoal.UPSERT_TASK:
            self.task_status_label.setText("采摘任务状态：写入任务中")
        elif self.last_request_type == PickTaskGoal.UPDATE_TASK_GROUP_CONFIG:
            self.task_status_label.setText("采摘任务状态：更新任务组配置中")
        else:
            self.task_status_label.setText("采摘任务状态：执行任务组中")

    def _on_pick_feedback(self, feedback) -> None:
        try:
            if self.last_request_type == PickTaskGoal.EXECUTE_TASK_GROUP:
                self.task_status_label.setText(
                    f"采摘任务状态：{feedback.stage_text} | step {feedback.current_step_index}/{feedback.total_steps}"
                )
        except Exception:
            pass

    def _on_pick_done(self, state, result) -> None:
        try:
            msg = getattr(result, "message", "")
            if getattr(result, "success", False):
                if self.last_request_type == PickTaskGoal.UPSERT_TASK:
                    self.task_status_label.setText(
                        f"采摘任务状态：任务写入成功 - {msg}"
                    )
                elif self.last_request_type == PickTaskGoal.UPDATE_TASK_GROUP_CONFIG:
                    self.task_status_label.setText(
                        f"采摘任务状态：任务组配置更新成功 - {msg}"
                    )
                else:
                    self.task_status_label.setText(
                        f"采摘任务状态：任务组执行完成 - {msg}"
                    )
            elif getattr(result, "canceled", False):
                self.task_status_label.setText(f"采摘任务状态：已取消 - {msg}")
            else:
                if self.last_request_type == PickTaskGoal.UPSERT_TASK:
                    self.task_status_label.setText(
                        f"采摘任务状态：任务写入失败 - {msg}"
                    )
                elif self.last_request_type == PickTaskGoal.UPDATE_TASK_GROUP_CONFIG:
                    self.task_status_label.setText(
                        f"采摘任务状态：任务组配置更新失败 - {msg}"
                    )
                else:
                    self.task_status_label.setText(
                        f"采摘任务状态：任务组执行失败 - {msg}"
                    )
        except Exception:
            self.task_status_label.setText(f"采摘任务状态：结束，state={state}")

    def on_roi_selected(self, cutting_pixel, mask) -> None:
        if (
            self.current_depth_data is None
            or self.current_color_size is None
            or mask is None
        ):
            self.btn_upsert_task.setEnabled(False)
            return

        self.current_cutting_pixel = cutting_pixel
        u_disp, v_disp = cutting_pixel
        color_w, color_h = self.current_color_size
        depth_h, depth_w = self.current_depth_data.shape[:2]

        if (depth_w, depth_h) != (color_w, color_h):
            self.info_label.setText(
                f"⚠️ 当前投影深度尺寸为 {depth_w}x{depth_h}，彩色尺寸为 {color_w}x{color_h}。\n"
                "D2C 未真正落到 color 像素坐标系，当前不会继续计算。"
            )
            self.target_tcp_coord = None
            self.btn_upsert_task.setEnabled(False)
            return

        if CAMERA_ROTATE_180:
            du, dv = rotate_point_180(
                int(round(u_disp)), int(round(v_disp)), color_w, color_h
            )
            depth_mask = cv2.rotate(mask, cv2.ROTATE_180)
        else:
            du = int(np.clip(round(u_disp), 0, depth_w - 1))
            dv = int(np.clip(round(v_disp), 0, depth_h - 1))
            depth_mask = mask

        du = int(np.clip(du, 0, depth_w - 1))
        dv = int(np.clip(dv, 0, depth_h - 1))

        depth_kernel = self._parse_int(
            self.depth_kernel_edit, TARGET_CFG.depth_kernel_size
        )
        trim_ratio = self._parse_float(
            self.depth_trim_ratio_edit, TARGET_CFG.depth_trim_ratio
        )
        depth_raw = self.get_valid_depth_around(
            self.current_depth_data, du, dv, depth_mask, depth_kernel, trim_ratio
        )
        depth_raw, use_lrm = self._get_depth_value_mm(depth_raw)

        if depth_raw == 0:
            self.info_label.setText("深度无效，请调整选择区域")
            self.raw_target_tcp_coord = None
            self.target_tcp_coord = None
            self.btn_upsert_task.setEnabled(False)
            return

        depth_m = depth_raw / 1000.0
        cam_xyz = depth_to_pointcloud(du, dv, depth_m, self.current_depth_intrinsics)
        flange_xyz = cam_point_to_flange_point(
            self.tf_buffer, self.current_depth_frame_id, *cam_xyz
        )

        try:
            raw_tcp_xyz = self.flange_point_to_tcp_point(*flange_xyz)
        except Exception as e:
            self.info_label.setText(f"TF 变换失败: {e}")
            self.raw_target_tcp_coord = None
            self.target_tcp_coord = None
            self.btn_upsert_task.setEnabled(False)
            return

        tcp_xyz = self._apply_tcp_residual_compensation(raw_tcp_xyz)

        self.last_cam_coord = cam_xyz
        self.last_flange_coord = flange_xyz
        self.raw_target_tcp_coord = raw_tcp_xyz
        self.target_tcp_coord = tcp_xyz
        self.btn_upsert_task.setEnabled(True)

        self._update_info_label(
            u_disp, v_disp, depth_m, use_lrm, cam_xyz, flange_xyz, raw_tcp_xyz, tcp_xyz
        )

    def closeEvent(self, event):
        if self.ros_available:
            rospy.signal_shutdown("GUI Closed")
        event.accept()


def main() -> None:
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    QTimer.singleShot(0, window.showMaximized)
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
