# Camera–IMU Calibration with Kalibr

This README describes the steps used to calibrate a camera and a camera–IMU system using **Kalibr** and ROS.

## Prerequisites

- ROS installed and sourced
- Kalibr installed
- Recorded rosbag file
- AprilTag calibration target YAML
- IMU noise parameters computed using **Allan variance**

---

## Camera Calibration

The following command calibrates the camera using an AprilTag grid target.

```bash
rosrun kalibr kalibr_calibrate_cameras \
  --target april_6x6_80x80cm.yaml \
  --models pinhole-equi \
  --topics /camera/image_raw \
  --bag rosbag_raven3.bag \
  --bag-freq 10.0
