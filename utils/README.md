# Utilities Directory

This directory contains various utility scripts for the RC Car project. Below is a brief description of each script and its functionality.

## Pre-requisite

On Raspberry Pi, run with: `sudo apt-get install -y python3-yaml`

## Scripts

### `camera_calib.py`
This script is used for camera calibration. It supports the following commands:
- `python3 camera_calib.py undistort <filename>`: Undistorts the specified image file by using the configuration from `camera_calibration.yaml`.
- `python3 camera_calib.py cali`: Or `calibr`, starts the camera calibration process. If calibration succeeded, save the configuration to `camera_calibration.yaml`.
- `python3 camera_calib.py collect`: Captures images every 2 seconds for later calibration.

### `camera_demo.py`
This script opens a live video stream with a resolution of 640x480 by default. It supports the following functionalities:
- Press `s` to save the current frame to the current folder.
- Use the `python3 camera_demo.py modes` parameter to check which video modes are supported by the onboard camera.

### `camera_warp.py`
This script is used for image warping and visualization of points. It supports the following commands:
- `python3 camera_warp.py warp_image <filename> [--undistorted] [--save]`: Warps the specified image file. Optionally undistorts the image before warping and saves the result.
- `python3 camera_warp.py visualize_points <filename> [--undistorted]`: Visualizes warp points on the specified image file. Optionally undistorts the image before visualization.

### `steering_calibration.py`
This script opens a GUI to control the steering of the RC car. It allows you to adjust the steering and save the configuration to `steering.yaml`.

### `longitudinal_control.py`
This script opens a GUI to control the drive system of the RC car.

## Usage

To run any of these scripts, use the following command format:
```
python <script_name.py> [parameters]
```

For example, to undistort an image using `camera_calib.py`, you would use:
```
python camera_calib.py undistort <filename>
```

Make sure to replace `<filename>` with the actual name of the image file you want to undistort.

## Camera Calibration Examples

![](640x480.jpg?raw=true)
![](Undistorted-640x480.png?raw=true)
![](2592x1944.jpg?raw=true)
![](Undistorted-2592x1944.png?raw=true)
