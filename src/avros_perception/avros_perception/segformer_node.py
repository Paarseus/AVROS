"""SegFormer semantic segmentation node: camera image -> TensorRT inference -> class mask.

Subscribes:
  /camera/camera/color/image_raw   sensor_msgs/Image (BGR8/RGB8, 1280x720 @ 30Hz)

Publishes:
  /perception/class_mask           sensor_msgs/Image (mono8, per-pixel Cityscapes class ID)

Requires a pre-built TensorRT engine file (see README for export steps).
"""

import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# ImageNet normalization (matches SegFormer Cityscapes preprocessor_config.json)
IMAGENET_MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
IMAGENET_STD = np.array([0.229, 0.224, 0.225], dtype=np.float32)


class SegFormerNode(Node):
    """Runs SegFormer-B0 TensorRT inference on camera images."""

    def __init__(self):
        super().__init__('segformer_node')

        # Parameters
        self.declare_parameter('engine_path', '')
        self.declare_parameter('input_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('output_topic', '/perception/class_mask')
        self.declare_parameter('inference_height', 512)
        self.declare_parameter('inference_width', 1024)
        self.declare_parameter('publish_rate', 10.0)

        self._engine_path = self.get_parameter('engine_path').value
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self._inf_h = self.get_parameter('inference_height').value
        self._inf_w = self.get_parameter('inference_width').value
        publish_rate = self.get_parameter('publish_rate').value

        if not self._engine_path:
            self.get_logger().fatal('engine_path parameter is required')
            raise SystemExit('engine_path not set')

        # TensorRT engine (lazy import — only available on Jetson)
        self._trt_engine = None
        self._load_engine()

        self._bridge = CvBridge()
        self._min_interval = 1.0 / publish_rate
        self._last_inference_time = 0.0

        # Subscriber / Publisher
        self._image_sub = self.create_subscription(
            Image, input_topic, self._image_callback, 1)
        self._mask_pub = self.create_publisher(Image, output_topic, 1)

        self.get_logger().info(
            f'SegFormer node ready — engine: {self._engine_path}, '
            f'inference: {self._inf_w}x{self._inf_h}, rate: {publish_rate} Hz')

    def _load_engine(self):
        """Load TensorRT engine from file."""
        import tensorrt as trt
        import pycuda.driver as cuda
        import pycuda.autoinit  # noqa: F401 — initializes CUDA context

        logger = trt.Logger(trt.Logger.WARNING)
        with open(self._engine_path, 'rb') as f:
            runtime = trt.Runtime(logger)
            engine = runtime.deserialize_cuda_engine(f.read())

        context = engine.create_execution_context()
        stream = cuda.Stream()

        # Tensor names and shapes
        input_name = engine.get_tensor_name(0)
        output_name = engine.get_tensor_name(1)
        input_shape = context.get_tensor_shape(input_name)
        output_shape = context.get_tensor_shape(output_name)

        # Allocate page-locked host buffers
        h_input = cuda.pagelocked_empty(trt.volume(input_shape), dtype=np.float32)
        h_output = cuda.pagelocked_empty(trt.volume(output_shape), dtype=np.float32)

        # Allocate device buffers
        d_input = cuda.mem_alloc(h_input.nbytes)
        d_output = cuda.mem_alloc(h_output.nbytes)

        # Bind tensor addresses
        context.set_tensor_address(input_name, int(d_input))
        context.set_tensor_address(output_name, int(d_output))

        self._trt_engine = {
            'context': context,
            'stream': stream,
            'h_input': h_input,
            'h_output': h_output,
            'd_input': d_input,
            'd_output': d_output,
            'output_shape': tuple(output_shape),
        }

        self.get_logger().info(
            f'TensorRT engine loaded — input: {tuple(input_shape)}, '
            f'output: {tuple(output_shape)}')

    def _preprocess(self, cv_image):
        """BGR image -> NCHW float32 tensor, ImageNet-normalized."""
        img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        img = cv2.resize(img, (self._inf_w, self._inf_h),
                         interpolation=cv2.INTER_LINEAR)
        img = img.astype(np.float32) / 255.0
        img = (img - IMAGENET_MEAN) / IMAGENET_STD
        img = np.transpose(img, (2, 0, 1))  # HWC -> CHW
        return np.ascontiguousarray(np.expand_dims(img, 0))

    def _infer(self, input_nchw):
        """Run TensorRT inference, return raw logits."""
        import pycuda.driver as cuda

        e = self._trt_engine
        np.copyto(e['h_input'], input_nchw.ravel())
        cuda.memcpy_htod_async(e['d_input'], e['h_input'], e['stream'])
        e['context'].execute_async_v3(stream_handle=e['stream'].handle)
        cuda.memcpy_dtoh_async(e['h_output'], e['d_output'], e['stream'])
        e['stream'].synchronize()
        return e['h_output'].reshape(e['output_shape'])

    def _postprocess(self, logits, original_h, original_w):
        """Logits (1, 19, H/4, W/4) -> class map (H, W) uint8."""
        class_map = np.argmax(logits[0], axis=0).astype(np.uint8)
        return cv2.resize(class_map, (original_w, original_h),
                          interpolation=cv2.INTER_NEAREST)

    def _image_callback(self, msg):
        """Process incoming camera image."""
        now = time.monotonic()
        if now - self._last_inference_time < self._min_interval:
            return  # Rate limit
        self._last_inference_time = now

        cv_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        original_h, original_w = cv_image.shape[:2]

        input_tensor = self._preprocess(cv_image)
        logits = self._infer(input_tensor)
        class_map = self._postprocess(logits, original_h, original_w)

        mask_msg = self._bridge.cv2_to_imgmsg(class_map, encoding='mono8')
        mask_msg.header = msg.header
        self._mask_pub.publish(mask_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SegFormerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
