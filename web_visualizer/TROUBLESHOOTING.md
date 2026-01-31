# Troubleshooting

## No image stream
- Verify the camera topic exists: `ros2 topic list | grep /cr/camera/bgr/front_960_768`.
- Check that the image encoding is supported (bgr8/mono8). The node converts to BGR.

## No markers / empty map
- Verify the marker topic: `ros2 topic echo /fusion_perception/obstacles --once`.
- The visualizer ignores markers with action != ADD.

## Web UI loads but stuck offline
- Confirm the web process is running on the correct host/port.
- Check firewall rules if accessing remotely.

## Performance is slow
- Lower `MAX_FPS` or `JPEG_QUALITY` environment variables.
- Reduce the perception publish rate if needed.
