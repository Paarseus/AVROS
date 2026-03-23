# avros_perception

Semantic segmentation perception for the AVROS autonomous vehicle. Uses SegFormer-B0 (Cityscapes 19 classes) with TensorRT inference on the Jetson Orin to classify the camera view into drivable and non-drivable areas, then projects non-drivable pixels into a 3D obstacle point cloud for Nav2.

## Overview

### Architecture

```
D455 /camera/camera/color/image_raw (1280x720 @ 30Hz)
  |
  v
[segformer_node] -- TensorRT FP16 inference, ~10Hz
  |
  v
/perception/class_mask (mono8, per-pixel Cityscapes class ID)
  |
  v
[depth_projection_node] -- back-projects non-drivable pixels to 3D
  |  (uses D455 aligned depth + camera intrinsics)
  v
/perception/obstacle_cloud (PointCloud2, frame: camera_depth_optical_frame)
  |
  v
Nav2 VoxelLayer (semantic_obstacles observation source)
```

### Nodes

| Node | Input | Output | Purpose |
|------|-------|--------|---------|
| `segformer_node` | D455 color image | Class mask (mono8) | TensorRT SegFormer-B0 inference |
| `depth_projection_node` | Class mask + depth + camera_info | Obstacle PointCloud2 | Semantic filtering + 3D projection |

### Class Remapping (Campus Config)

The model outputs 19 Cityscapes classes. For campus sidewalk driving, classes are remapped:

| Class ID | Label | Costmap Treatment |
|----------|-------|-------------------|
| 0 | road | **Drivable** (free space) |
| 1 | sidewalk | **Drivable** (free space) |
| 9 | terrain | **Drivable** (free space) |
| 10 | sky | Ignored |
| 2-8, 11-18 | building, wall, fence, pole, traffic light, traffic sign, vegetation, person, rider, car, truck, bus, train, motorcycle, bicycle | **Obstacle** (projected to costmap) |

This is configured via the `drivable_classes` and `ignore_classes` parameters in `perception_params.yaml`.

## Setup

### Prerequisites

- NVIDIA Jetson Orin with JetPack 6.1+
- ROS 2 Humble
- RealSense D455 camera working (`ros2 launch avros_bringup sensors.launch.py`)
- Python packages on Jetson: `pycuda` (TensorRT and numpy ship with JetPack)

### Step 1: Export ONNX Model

Run this on any machine with Python (laptop, desktop, or the Jetson itself):

```bash
pip install "optimum[onnx]"

optimum-cli export onnx \
  --model nvidia/segformer-b0-finetuned-cityscapes-1024-1024 \
  --task semantic-segmentation \
  --opset 17 \
  --no-dynamic-axes \
  --height 512 \
  --width 1024 \
  segformer_b0_onnx/
```

This downloads the pre-trained SegFormer-B0 from HuggingFace and converts it from PyTorch to ONNX format. The output is `segformer_b0_onnx/model.onnx` (~15MB).

- `--opset 17` ensures LayerNorm exports correctly for TensorRT
- `--no-dynamic-axes` locks the input shape for TensorRT optimization
- `--height 512 --width 1024` sets the inference resolution (D455 is 1280x720, this is close enough aspect ratio)

### Step 2: Copy ONNX to Jetson

```bash
scp segformer_b0_onnx/model.onnx jetson:~/avros_models/
```

Or if you exported on the Jetson directly:

```bash
mkdir -p ~/avros_models
cp segformer_b0_onnx/model.onnx ~/avros_models/
```

### Step 3: Build TensorRT Engine (on Jetson)

This must be done on the Jetson itself. TensorRT engines are hardware-specific and not portable between machines.

```bash
ssh jetson

mkdir -p ~/avros_models

/usr/src/tensorrt/bin/trtexec \
  --onnx=~/avros_models/model.onnx \
  --saveEngine=~/avros_models/segformer_b0_fp16.engine \
  --fp16
```

This takes 2-5 minutes. TensorRT analyzes every operation and compiles an optimized engine for the Orin GPU with FP16 precision.

If you see NaN/garbage output later (rare LayerNorm FP16 overflow), rebuild with:

```bash
/usr/src/tensorrt/bin/trtexec \
  --onnx=~/avros_models/model.onnx \
  --saveEngine=~/avros_models/segformer_b0_fp16.engine \
  --fp16 \
  --precisionConstraints=obey \
  --layerPrecisions="/*/LayerNorm*:fp32"
```

### Step 4: Install pycuda (on Jetson)

```bash
export CUDA_ROOT=/usr/local/cuda
pip install pycuda
```

### Step 5: Build the ROS 2 Package

```bash
cd ~/AVROS
colcon build --symlink-install --packages-select avros_perception
colcon build --symlink-install
source install/setup.bash
```

### Step 6: Verify Engine Path

Check that `perception_params.yaml` points to your engine file:

```yaml
segformer_node:
  ros__parameters:
    engine_path: "/home/dinosaur/avros_models/segformer_b0_fp16.engine"
```

Update the path if your Jetson user or directory differs.

## Usage

### Bench Test (camera only, no navigation)

```bash
# Terminal 1: Start camera
ros2 launch avros_bringup sensors.launch.py enable_velodyne:=false enable_ntrip:=false

# Terminal 2: Start perception
ros2 launch avros_bringup perception.launch.py

# Terminal 3: Verify topics
ros2 topic hz /perception/class_mask        # Should show ~10 Hz
ros2 topic hz /perception/obstacle_cloud    # Should show ~10 Hz

# Visualize in RViz:
#   - Add Image display for /perception/class_mask
#   - Add PointCloud2 display for /perception/obstacle_cloud
```

Point the camera at a sidewalk, wall, or person and verify the class mask looks reasonable before field testing.

### Full Navigation with Perception

```bash
ros2 launch avros_bringup navigation.launch.py enable_perception:=true
```

### Without Perception (default, no change from before)

```bash
ros2 launch avros_bringup navigation.launch.py
```

## Configuration

All parameters are in `avros_bringup/config/perception_params.yaml`:

### segformer_node

| Parameter | Default | Description |
|-----------|---------|-------------|
| `engine_path` | `/home/dinosaur/avros_models/segformer_b0_fp16.engine` | TensorRT engine file |
| `input_topic` | `/camera/camera/color/image_raw` | D455 color image topic |
| `output_topic` | `/perception/class_mask` | Published segmentation mask |
| `inference_height` | `512` | Must match ONNX export height |
| `inference_width` | `1024` | Must match ONNX export width |
| `publish_rate` | `10.0` | Max inference rate in Hz |

### depth_projection_node

| Parameter | Default | Description |
|-----------|---------|-------------|
| `drivable_classes` | `[0, 1, 9]` | Cityscapes classes treated as drivable (road, sidewalk, terrain) |
| `ignore_classes` | `[10]` | Classes to skip (sky) |
| `min_depth` | `0.3` | D455 minimum reliable depth (meters) |
| `max_depth` | `5.0` | D455 effective depth range (meters) |
| `downsample_factor` | `4` | Process every Nth pixel for speed |

## Troubleshooting

| Problem | Cause | Fix |
|---------|-------|-----|
| `engine_path not set` | Missing parameter | Check `perception_params.yaml` has correct path |
| Engine load fails | Wrong TensorRT version | Rebuild engine on the Jetson with `trtexec` |
| Class mask is all zeros | FP16 LayerNorm overflow | Rebuild engine with `--precisionConstraints=obey --layerPrecisions="/*/LayerNorm*:fp32"` |
| No obstacle cloud published | Everything classified as drivable | Check camera is seeing non-drivable objects; verify class mask in RViz |
| Low FPS | Engine not FP16 or power mode | Run `sudo nvpmodel -m 0 && sudo jetson_clocks` |
| `pycuda` import error | Not installed or CUDA path wrong | `export CUDA_ROOT=/usr/local/cuda && pip install pycuda` |
| Depth projection wrong | Camera intrinsics mismatch | Verify `/camera/camera/aligned_depth_to_color/camera_info` is publishing |

## Fine-Tuning (Optional)

If the off-the-shelf Cityscapes model doesn't work well on your specific campus:

1. Record rosbags while driving campus paths
2. Extract ~200-500 frames from the D455 color stream
3. Annotate with [CVAT](https://www.cvat.ai/) or [Label Studio](https://labelstud.io/) — binary labels: `drivable` vs `obstacle`
4. Fine-tune SegFormer-B0 from the Cityscapes checkpoint using the [HuggingFace tutorial](https://huggingface.co/blog/fine-tune-segformer)
5. Re-export to ONNX and rebuild the TensorRT engine
