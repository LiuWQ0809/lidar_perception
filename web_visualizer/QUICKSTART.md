# Quickstart

```bash
cd /home/nvidia/workspaces/perception/lidar_perception/web_visualizer
./start.sh
```

Open `http://<device-ip>:8080`.

## Required ROS topics
- `/cr/camera/bgr/front_960_768` (`sensor_msgs/Image`) or first enabled camera in config
- `/fusion_perception/obstacles` (`visualization_msgs/MarkerArray`)
