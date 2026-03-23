"""Depth projection node: semantic class mask + depth image -> obstacle PointCloud2.

Projects non-drivable pixels from the segmentation mask into 3D using the
aligned depth image and camera intrinsics. Publishes a filtered PointCloud2
for Nav2's VoxelLayer.

Subscribes (time-synchronized):
  /perception/class_mask                          sensor_msgs/Image (mono8)
  /camera/camera/aligned_depth_to_color/image_raw sensor_msgs/Image (16UC1 depth mm)
  /camera/camera/aligned_depth_to_color/camera_info sensor_msgs/CameraInfo

Publishes:
  /perception/obstacle_cloud  sensor_msgs/PointCloud2 (frame: camera_depth_optical_frame)
"""

import struct

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from cv_bridge import CvBridge
import message_filters


class DepthProjectionNode(Node):
    """Projects non-drivable semantic pixels into a 3D obstacle point cloud."""

    def __init__(self):
        super().__init__('depth_projection_node')

        # Parameters
        self.declare_parameter('mask_topic', '/perception/class_mask')
        self.declare_parameter('depth_topic',
                               '/camera/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('info_topic',
                               '/camera/camera/aligned_depth_to_color/camera_info')
        self.declare_parameter('output_topic', '/perception/obstacle_cloud')
        self.declare_parameter('output_frame', 'camera_depth_optical_frame')
        self.declare_parameter('drivable_classes', [0, 1, 9])  # road, sidewalk, terrain
        self.declare_parameter('ignore_classes', [10])          # sky
        self.declare_parameter('min_depth', 0.3)
        self.declare_parameter('max_depth', 5.0)
        self.declare_parameter('downsample_factor', 4)

        mask_topic = self.get_parameter('mask_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        info_topic = self.get_parameter('info_topic').value
        output_topic = self.get_parameter('output_topic').value
        self._output_frame = self.get_parameter('output_frame').value
        self._drivable = set(self.get_parameter('drivable_classes').value)
        self._ignore = set(self.get_parameter('ignore_classes').value)
        self._min_depth = self.get_parameter('min_depth').value
        self._max_depth = self.get_parameter('max_depth').value
        self._ds = self.get_parameter('downsample_factor').value

        self._bridge = CvBridge()
        self._fx = self._fy = self._cx = self._cy = None

        # Time-synchronized subscribers
        mask_sub = message_filters.Subscriber(self, Image, mask_topic)
        depth_sub = message_filters.Subscriber(self, Image, depth_topic)
        info_sub = message_filters.Subscriber(self, CameraInfo, info_topic)

        self._sync = message_filters.ApproximateTimeSynchronizer(
            [mask_sub, depth_sub, info_sub], queue_size=5, slop=0.1)
        self._sync.registerCallback(self._synced_callback)

        # Publisher
        self._cloud_pub = self.create_publisher(PointCloud2, output_topic, 1)

        self.get_logger().info(
            f'Depth projection node ready — drivable classes: {self._drivable}, '
            f'depth: {self._min_depth}-{self._max_depth}m, downsample: {self._ds}x')

    def _synced_callback(self, mask_msg, depth_msg, info_msg):
        """Process synchronized mask + depth + camera_info."""
        # Cache intrinsics from first message
        if self._fx is None:
            K = info_msg.k
            self._fx, self._fy = K[0], K[4]
            self._cx, self._cy = K[2], K[5]
            self.get_logger().info(
                f'Camera intrinsics: fx={self._fx:.1f} fy={self._fy:.1f} '
                f'cx={self._cx:.1f} cy={self._cy:.1f}')

        # Decode images
        class_mask = self._bridge.imgmsg_to_cv2(mask_msg, desired_encoding='mono8')
        depth_image = self._bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')

        # Convert depth to meters (RealSense publishes 16UC1 in millimeters)
        if depth_image.dtype == np.uint16:
            depth_m = depth_image.astype(np.float32) / 1000.0
        else:
            depth_m = depth_image.astype(np.float32)

        # Downsample for speed
        ds = self._ds
        class_ds = class_mask[::ds, ::ds]
        depth_ds = depth_m[::ds, ::ds]

        # Build obstacle mask: not drivable, not ignore, valid depth
        skip_classes = self._drivable | self._ignore
        obstacle_mask = np.ones(class_ds.shape, dtype=bool)
        for cls in skip_classes:
            obstacle_mask &= (class_ds != cls)

        obstacle_mask &= (depth_ds > self._min_depth)
        obstacle_mask &= (depth_ds < self._max_depth)

        if not np.any(obstacle_mask):
            return  # No obstacles — skip publishing empty cloud

        # Get pixel coordinates and depths of obstacle points
        vs, us = np.where(obstacle_mask)  # row (v), col (u) in downsampled image
        depths = depth_ds[vs, us]

        # Scale pixel coords back to original resolution for intrinsics
        us_orig = us.astype(np.float32) * ds
        vs_orig = vs.astype(np.float32) * ds

        # Back-project to 3D in camera optical frame (Z-forward, X-right, Y-down)
        x = (us_orig - self._cx) * depths / self._fx
        y = (vs_orig - self._cy) * depths / self._fy
        z = depths

        # Build PointCloud2
        points = np.column_stack([x, y, z]).astype(np.float32)
        cloud_msg = self._create_pointcloud2(points, mask_msg.header.stamp)
        self._cloud_pub.publish(cloud_msg)

    def _create_pointcloud2(self, points, stamp):
        """Create a sensor_msgs/PointCloud2 from Nx3 float32 array."""
        msg = PointCloud2()
        msg.header.stamp = stamp
        msg.header.frame_id = self._output_frame
        msg.height = 1
        msg.width = len(points)
        msg.is_dense = True
        msg.is_bigendian = False
        msg.point_step = 12  # 3 x float32
        msg.row_step = msg.point_step * msg.width
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.data = points.tobytes()
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = DepthProjectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
