# Web Visualizer

Web-based dashboard for fusion perception results. It streams camera images from the first enabled camera in `config/fusion_config.yaml` (e.g. `/cr/camera/bgr/front_960_768`) and draws 3D bounding boxes from `/fusion_perception/obstacles` on a birdseye map.

## Features
- MJPEG live view with camera selection
- Birdseye map with oriented 3D boxes
- Object list with distance + size
- Live status (FPS / age)
- Reads topics and perception range from `config/fusion_config.yaml`

## Quick Start
1. Ensure the camera topic and obstacle markers are available:
   - camera topic (default: first enabled camera in config)
   - `/fusion_perception/obstacles`
2. Launch the web visualizer:
   ```bash
   cd web_visualizer
   ./start.sh
   ```
3. Open browser: `http://<device-ip>:8080`

## Environment Overrides
- `CONFIG_PATH`: path to `fusion_config.yaml`
- `HOST`, `PORT`: server binding
- `IMAGE_TOPIC`, `MARKER_TOPIC`: override ROS topics
- `JPEG_QUALITY`: 1-100, default 80
- `MAX_FPS`: MJPEG max FPS, default 15

## Python Dependencies
See `requirements.txt`.

## Troubleshooting
If the page stays empty:
- Confirm the camera topic exists and has `sensor_msgs/Image`.
- Confirm `/fusion_perception/obstacles` exists and has `MarkerArray`.
- If you want fused overlays instead of raw camera, set `publisher.visualize: true` and run with `IMAGE_TOPIC=/fusion_perception/visualization`.
- Run `ros2 topic list` to validate names.
