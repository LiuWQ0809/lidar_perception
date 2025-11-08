# Models Directory

This directory contains the pre-trained models for fusion perception.

## YOLOv8 Model

- **File**: `yolov8n.pt`
- **Type**: Object Detection
- **Framework**: Ultralytics YOLOv8
- **Input Size**: 640x640
- **Classes**: COCO 80 classes
- **Download**: Automatically downloaded by the script

### Manual Download

If automatic download fails, you can manually download the model:

```bash
wget https://github.com/ultralytics/assets/releases/download/v0.0.0/yolov8n.pt
```

## Supported Models

### YOLOv8 Variants
- `yolov8n.pt` - Nano (fastest, recommended for Jetson)
- `yolov8s.pt` - Small
- `yolov8m.pt` - Medium
- `yolov8l.pt` - Large
- `yolov8x.pt` - Extra Large

### Usage

To use a different YOLOv8 model, update the configuration file:

```yaml
models:
  detector:
    model_path: "models/yolov8s.pt"  # Change to your preferred model
```

## Model Performance

On Jetson Orin:
- YOLOv8n: ~30-50 FPS
- YOLOv8s: ~20-30 FPS
- YOLOv8m: ~10-15 FPS

Choose the model based on your accuracy and speed requirements.
