#!/usr/bin/env python3
from __future__ import annotations

import math
from typing import Any, Dict, List, Optional

import cv2
import numpy as np
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
import tf.transformations as tf_trans
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float32

from pyorbbecsdk import (
    OBAlignMode,
    OBFormat,
    OBPropertyID,
    OBSensorType,
    Config,
    Pipeline,
    VideoFrame,
)

try:
    from pyorbbecsdk import AlignFilter, OBFrameAggregateOutputMode, OBStreamType
except ImportError:
    AlignFilter = None
    OBFrameAggregateOutputMode = None
    OBStreamType = None


DEFAULT_ZERO_DISTORTION = [0.0, 0.0, 0.0, 0.0, 0.0]


def intrinsic_to_matrix(intrinsic: Any) -> np.ndarray:
    return np.array(
        [
            [float(intrinsic.fx), 0.0, float(intrinsic.cx)],
            [0.0, float(intrinsic.fy), float(intrinsic.cy)],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )


def frame_to_bgr_image(frame: VideoFrame) -> Optional[np.ndarray]:
    width = frame.get_width()
    height = frame.get_height()
    color_format = frame.get_format()
    data = np.asanyarray(frame.get_data())

    if color_format == OBFormat.RGB:
        image = np.frombuffer(frame.get_data(), dtype=np.uint8).reshape(
            height, width, 3
        )
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

    if color_format == OBFormat.YUYV:
        image = np.frombuffer(frame.get_data(), dtype=np.uint8).reshape(
            height, width, 2
        )
        return cv2.cvtColor(image, cv2.COLOR_YUV2BGR_YUYV)

    if color_format == OBFormat.MJPG:
        return cv2.imdecode(data, cv2.IMREAD_COLOR)

    return None


def rotate_intrinsics_180(K: np.ndarray, width: int, height: int) -> np.ndarray:
    rotated = np.array(K, dtype=np.float64, copy=True)
    rotated[0, 2] = float(width - 1) - float(K[0, 2])
    rotated[1, 2] = float(height - 1) - float(K[1, 2])
    return rotated


class DMArmOrbbec:
    def __init__(self):
        rospy.init_node("dm_arm_orbbec")
        self.bridge = CvBridge()

        self.color_width = rospy.get_param("~color_width", 1280)
        self.color_height = rospy.get_param("~color_height", 720)
        self.color_fps = rospy.get_param("~color_fps", 30)

        self.depth_width = rospy.get_param("~depth_width", 1280)
        self.depth_height = rospy.get_param("~depth_height", 800)
        self.depth_fps = rospy.get_param("~depth_fps", 30)

        self.rotate_180 = bool(rospy.get_param("~rotate_180", False))
        self.min_depth_m = float(rospy.get_param("~min_depth_m", 0.10))
        self.max_depth_m = float(rospy.get_param("~max_depth_m", 10.0))
        self.enable_lrm = bool(rospy.get_param("~enable_lrm", True))
        self.publish_rate = float(
            rospy.get_param("~publish_rate", max(1.0, self.color_fps))
        )

        self.color_frame_id = rospy.get_param(
            "~color_frame_id", "eef_camera_color_optical_frame"
        )
        self.depth_frame_id = rospy.get_param(
            "~depth_frame_id", "eef_camera_depth_optical_frame"
        )
        self.depth_registered_frame_id = rospy.get_param(
            "~depth_registered_frame_id", self.color_frame_id
        )

        self._static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster()
        self._publish_cam_to_flange_tf()

        self.distortion_model = rospy.get_param("~distortion_model", "plumb_bob")
        self.color_distortion_override = self._normalize_distortion_param(
            rospy.get_param("~color_distortion", DEFAULT_ZERO_DISTORTION)
        )
        self.depth_distortion_override = self._normalize_distortion_param(
            rospy.get_param("~depth_distortion", DEFAULT_ZERO_DISTORTION)
        )

        self.pipeline = None
        self.device = None
        self.align_filter = None

        self.color_pub = rospy.Publisher(
            "/dm_arm/camera/orbbec/color/image_raw", Image, queue_size=1
        )
        self.color_info_pub = rospy.Publisher(
            "/dm_arm/camera/orbbec/color/camera_info", CameraInfo, queue_size=1
        )
        self.depth_raw_pub = rospy.Publisher(
            "/dm_arm/camera/orbbec/depth/image_raw", Image, queue_size=1
        )
        self.depth_raw_info_pub = rospy.Publisher(
            "/dm_arm/camera/orbbec/depth/camera_info", CameraInfo, queue_size=1
        )
        self.depth_registered_pub = rospy.Publisher(
            "/dm_arm/camera/orbbec/depth_registered/image_raw", Image, queue_size=1
        )
        self.depth_registered_info_pub = rospy.Publisher(
            "/dm_arm/camera/orbbec/depth_registered/camera_info",
            CameraInfo,
            queue_size=1,
        )
        self.lrm_pub = rospy.Publisher(
            "/dm_arm/camera/orbbec/lrm_distance", Float32, queue_size=1
        )

        self.color_model_source = "unknown"
        self.depth_model_source = "unknown"

    def _publish_cam_to_flange_tf(self):
        matrix_raw = rospy.get_param("~hand_eye/T_cam_to_flange", None)

        if matrix_raw is None:
            rospy.logwarn("未找到手眼标定参数 ~hand_eye/T_cam_to_flange，TF 不发布")
            return

        try:
            T = np.array(matrix_raw)

            trans = T[:3, 3]
            quat = tf_trans.quaternion_from_matrix(T)

            static_transform_stamped = TransformStamped()
            static_transform_stamped.header.stamp = rospy.Time.now()
            static_transform_stamped.header.frame_id = rospy.get_param(
                "~hand_eye/flange_id", "link6-7"
            )
            static_transform_stamped.child_frame_id = self.color_frame_id

            static_transform_stamped.transform.translation.x = float(trans[0])
            static_transform_stamped.transform.translation.y = float(trans[1])
            static_transform_stamped.transform.translation.z = float(trans[2])
            static_transform_stamped.transform.rotation.x = float(quat[0])
            static_transform_stamped.transform.rotation.y = float(quat[1])
            static_transform_stamped.transform.rotation.z = float(quat[2])
            static_transform_stamped.transform.rotation.w = float(quat[3])

            self._static_tf_broadcaster.sendTransform(static_transform_stamped)
            rospy.loginfo("已发布相机到机械臂法兰的静态 TF")
        except Exception as exc:
            rospy.logwarn("手眼标定参数解析失败，TF 不发布: %s", exc)

    @staticmethod
    def _normalize_distortion_param(raw: Any) -> List[float]:
        if raw is None:
            return list(DEFAULT_ZERO_DISTORTION)
        if isinstance(raw, (int, float)):
            return [float(raw)]
        if isinstance(raw, (list, tuple)):
            return [float(v) for v in raw]
        return list(DEFAULT_ZERO_DISTORTION)

    def _select_color_profile(self, profile_list):
        try:
            return profile_list.get_video_stream_profile(
                self.color_width, self.color_height, OBFormat.RGB, self.color_fps
            )
        except Exception:
            try:
                return profile_list.get_video_stream_profile(
                    self.color_width, self.color_height, OBFormat.MJPG, self.color_fps
                )
            except Exception:
                return profile_list.get_default_video_stream_profile()

    def _select_depth_profile(self, profile_list):
        try:
            return profile_list.get_video_stream_profile(
                self.depth_width, self.depth_height, OBFormat.Y16, self.depth_fps
            )
        except Exception:
            return profile_list.get_default_video_stream_profile()

    def _set_disparity_search_range_256(self) -> bool:
        if self.device is None:
            return False

        prop = getattr(OBPropertyID, "OB_PROP_DISP_SEARCH_RANGE_MODE_INT", None)
        if prop is None:
            rospy.logwarn("pyorbbecsdk 未暴露 OB_PROP_DISP_SEARCH_RANGE_MODE_INT")
            return False

        try:
            self.device.set_int_property(prop, 2)
            applied = self.device.get_int_property(prop)
            rospy.loginfo("视差搜索范围模式值: %s", applied)
            return applied == 2
        except Exception as exc:
            rospy.logwarn("视差搜索范围设置失败: %s", exc)
            return False

    def _build_depth_m(self, depth_frame: VideoFrame) -> np.ndarray:
        depth_u16 = np.frombuffer(depth_frame.get_data(), dtype=np.uint16).reshape(
            (depth_frame.get_height(), depth_frame.get_width())
        )
        depth_u16 = cv2.medianBlur(depth_u16, 5)
        depth_m = (
            depth_u16.astype(np.float32) * float(depth_frame.get_depth_scale()) / 1000.0
        )
        depth_m = np.where(
            (depth_m > self.min_depth_m) & (depth_m < self.max_depth_m), depth_m, 0.0
        ).astype(np.float32)
        return depth_m

    def _extract_distortion_from_profile(self, profile: Any) -> Optional[List[float]]:
        candidates = []
        for getter_name in (
            "get_distortion",
            "get_distortion_param",
            "get_distortion_params",
            "get_camera_param",
            "get_camera_intrinsic",
        ):
            getter = getattr(profile, getter_name, None)
            if callable(getter):
                try:
                    candidates.append(getter())
                except Exception:
                    pass

        for candidate in candidates:
            parsed = self._parse_distortion_object(candidate)
            if parsed is not None:
                return parsed
        return None

    def _parse_distortion_object(self, obj: Any) -> Optional[List[float]]:
        if obj is None:
            return None
        if isinstance(obj, (list, tuple, np.ndarray)):
            values = [float(v) for v in obj]
            if len(values) >= 4:
                return values
        if hasattr(obj, "coeffs"):
            return self._parse_distortion_object(getattr(obj, "coeffs"))
        fields = []
        for name in ("k1", "k2", "p1", "p2", "k3", "k4", "k5", "k6"):
            if hasattr(obj, name):
                fields.append(float(getattr(obj, name)))
        if len(fields) >= 4:
            return fields
        for name in (
            "color_distortion",
            "depth_distortion",
            "rgb_distortion",
            "distortion",
        ):
            if hasattr(obj, name):
                parsed = self._parse_distortion_object(getattr(obj, name))
                if parsed is not None:
                    return parsed
        return None

    def _resolve_distortion(
        self, profile: Any, override: List[float], label: str
    ) -> List[float]:
        extracted = self._extract_distortion_from_profile(profile)
        if extracted is not None:
            source = "sdk_profile"
            distortion = extracted
        else:
            distortion = list(override)
            source = "override_param"
            if not any(abs(v) > 1e-12 for v in distortion):
                source = "override_zero"

        if label == "color":
            self.color_model_source = source
        else:
            self.depth_model_source = source
        return distortion

    def _build_camera_info(
        self,
        width: int,
        height: int,
        K: np.ndarray,
        D: List[float],
        frame_id: str,
        stamp: rospy.Time,
    ) -> CameraInfo:
        msg = CameraInfo()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.width = int(width)
        msg.height = int(height)
        msg.distortion_model = self.distortion_model
        msg.D = [float(v) for v in D]
        msg.K = [
            float(K[0, 0]),
            0.0,
            float(K[0, 2]),
            0.0,
            float(K[1, 1]),
            float(K[1, 2]),
            0.0,
            0.0,
            1.0,
        ]
        msg.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        msg.P = [
            float(K[0, 0]),
            0.0,
            float(K[0, 2]),
            0.0,
            0.0,
            float(K[1, 1]),
            float(K[1, 2]),
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
        ]
        return msg

    def _to_msg(
        self, image: np.ndarray, encoding: str, stamp: rospy.Time, frame_id: str
    ) -> Image:
        msg = self.bridge.cv2_to_imgmsg(image, encoding=encoding)
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        return msg

    def setup(self):
        self.pipeline = Pipeline()
        self.device = self.pipeline.get_device()
        config = Config()

        color_profile_list = self.pipeline.get_stream_profile_list(
            OBSensorType.COLOR_SENSOR
        )
        depth_profile_list = self.pipeline.get_stream_profile_list(
            OBSensorType.DEPTH_SENSOR
        )

        color_profile = self._select_color_profile(color_profile_list)
        depth_profile = self._select_depth_profile(depth_profile_list)

        config.enable_stream(color_profile)
        config.enable_stream(depth_profile)

        if hasattr(config, "set_align_mode"):
            try:
                config.set_align_mode(OBAlignMode.SW_MODE)
            except Exception:
                pass

        if (
            hasattr(config, "set_frame_aggregate_output_mode")
            and OBFrameAggregateOutputMode is not None
            and hasattr(OBFrameAggregateOutputMode, "FULL_FRAME_REQUIRE")
        ):
            try:
                config.set_frame_aggregate_output_mode(
                    OBFrameAggregateOutputMode.FULL_FRAME_REQUIRE
                )
            except Exception:
                pass

        self.pipeline.enable_frame_sync()
        self.pipeline.start(config)

        if AlignFilter is not None and OBStreamType is not None:
            try:
                self.align_filter = AlignFilter(
                    align_to_stream=OBStreamType.COLOR_STREAM
                )
                rospy.loginfo("已启用 AlignFilter: Depth -> Color")
            except Exception as exc:
                self.align_filter = None
                rospy.logwarn("AlignFilter 初始化失败: %s", exc)
        else:
            rospy.logwarn(
                "当前 pyorbbecsdk 未暴露 AlignFilter，将不发布 depth_registered"
            )

        disparity_ok = self._set_disparity_search_range_256()

        if self.enable_lrm:
            try:
                self.device.set_bool_property(OBPropertyID.OB_PROP_LDP_BOOL, True)
                rospy.loginfo("LRM 激光补盲模块已开启")
            except Exception as exc:
                rospy.logwarn("LRM 开启失败: %s", exc)

        rospy.loginfo(
            "相机启动成功, disp256=%s, rotate_180=%s, color_frame=%s, depth_frame=%s, depth_registered_frame=%s",
            "OK" if disparity_ok else "NO",
            self.rotate_180,
            self.color_frame_id,
            self.depth_frame_id,
            self.depth_registered_frame_id,
        )

    def spin(self):
        rate = rospy.Rate(self.publish_rate)

        while not rospy.is_shutdown():
            frames = self.pipeline.wait_for_frames(100)
            if frames is None:
                rate.sleep()
                continue

            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            if color_frame is None or depth_frame is None:
                rate.sleep()
                continue

            aligned_depth_frame = None
            if self.align_filter is not None:
                try:
                    aligned_frames = self.align_filter.process(frames)
                    if aligned_frames is not None:
                        aligned_depth_frame = (
                            aligned_frames.as_frame_set().get_depth_frame()
                        )
                except Exception as exc:
                    rospy.logwarn_throttle(2.0, "AlignFilter 处理失败: %s", exc)
                    aligned_depth_frame = None

            color_image = frame_to_bgr_image(color_frame)
            if color_image is None:
                rate.sleep()
                continue

            raw_depth_m = self._build_depth_m(depth_frame)
            aligned_depth_m = None
            if aligned_depth_frame is not None:
                try:
                    aligned_depth_m = self._build_depth_m(aligned_depth_frame)
                except Exception as exc:
                    rospy.logwarn_throttle(2.0, "对齐深度解析失败: %s", exc)
                    aligned_depth_m = None

            color_profile = color_frame.get_stream_profile().as_video_stream_profile()
            depth_profile = depth_frame.get_stream_profile().as_video_stream_profile()
            K_color = intrinsic_to_matrix(color_profile.get_intrinsic())
            K_depth = intrinsic_to_matrix(depth_profile.get_intrinsic())
            D_color = self._resolve_distortion(
                color_profile, self.color_distortion_override, "color"
            )
            D_depth = self._resolve_distortion(
                depth_profile, self.depth_distortion_override, "depth"
            )

            if self.rotate_180:
                color_image = cv2.rotate(color_image, cv2.ROTATE_180)
                raw_depth_m = cv2.rotate(raw_depth_m, cv2.ROTATE_180)
                K_color = rotate_intrinsics_180(
                    K_color, color_image.shape[1], color_image.shape[0]
                )
                K_depth = rotate_intrinsics_180(
                    K_depth, raw_depth_m.shape[1], raw_depth_m.shape[0]
                )
                if aligned_depth_m is not None:
                    aligned_depth_m = cv2.rotate(aligned_depth_m, cv2.ROTATE_180)

            publish_registered = False
            if (
                aligned_depth_m is not None
                and aligned_depth_m.shape[:2] == color_image.shape[:2]
            ):
                publish_registered = True
            elif aligned_depth_m is not None:
                rospy.logwarn_throttle(
                    2.0,
                    "depth_registered 尺寸异常: aligned=%s color=%s，当前帧不发布 depth_registered",
                    aligned_depth_m.shape[:2],
                    color_image.shape[:2],
                )

            now = rospy.Time.now()

            color_msg = self._to_msg(color_image, "bgr8", now, self.color_frame_id)
            color_info_msg = self._build_camera_info(
                color_image.shape[1],
                color_image.shape[0],
                K_color,
                D_color,
                self.color_frame_id,
                now,
            )
            depth_raw_msg = self._to_msg(raw_depth_m, "32FC1", now, self.depth_frame_id)
            depth_raw_info_msg = self._build_camera_info(
                raw_depth_m.shape[1],
                raw_depth_m.shape[0],
                K_depth,
                D_depth,
                self.depth_frame_id,
                now,
            )

            self.color_pub.publish(color_msg)
            self.color_info_pub.publish(color_info_msg)
            self.depth_raw_pub.publish(depth_raw_msg)
            self.depth_raw_info_pub.publish(depth_raw_info_msg)

            if publish_registered:
                K_depth_registered = np.array(K_color, dtype=np.float64, copy=True)
                depth_registered_msg = self._to_msg(
                    aligned_depth_m,
                    "32FC1",
                    now,
                    self.depth_registered_frame_id,
                )
                depth_registered_info_msg = self._build_camera_info(
                    aligned_depth_m.shape[1],
                    aligned_depth_m.shape[0],
                    K_depth_registered,
                    D_color,
                    self.depth_registered_frame_id,
                    now,
                )
                self.depth_registered_pub.publish(depth_registered_msg)
                self.depth_registered_info_pub.publish(depth_registered_info_msg)

            if self.enable_lrm:
                try:
                    lrm_dist_mm = self.device.get_int_property(
                        OBPropertyID.OB_PROP_LDP_MEASURE_DISTANCE_INT
                    )
                    self.lrm_pub.publish(Float32(data=float(lrm_dist_mm) / 1000.0))
                except Exception:
                    pass

            rate.sleep()


if __name__ == "__main__":
    node = DMArmOrbbec()
    node.setup()
    node.spin()
