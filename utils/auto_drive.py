import argparse
import math
import time
import numpy as np
import sys
from collections import defaultdict

import cv2
import matplotlib.pyplot as plt
from picamera2 import Picamera2

from longitudinal_control import LongitudinalControl
from steering_control import SteeringControl

sys.path.append('../src')
from lane_detector import LaneDetector
from pid_control import PID

FIXED_FORWARD_SPEED = 10.0  # units per second

def _put_text(img, text, pos, font=cv2.FONT_HERSHEY_SIMPLEX, scale=0.5, color=(255, 255, 255), thickness=1):
    """
    Put text on an image.
    Args:
        img (numpy.ndarray): Image to draw text on.
        text (str): Text to draw.
        pos (tuple): Position to draw the text at.
        font (int): Font type.
        scale (float): Font scale.
        color (tuple): Color of the text in BGR format.
        thickness (int): Thickness of the text.
    """
    cv2.putText(img, text, pos, font, scale, color, thickness)

class CarController:
    """
    Class to control the car's movement and steering using camera input for lane detection.
    Args:
        camera_resolution (tuple): Resolution of the camera.
        offset_weight (float): Weight for the offset in lane detection, [0.0, 1.0].
        pid_weights (tuple): Weights for PID control, (Kp, Ki, Kd).
        calibration_conf_path (str): Path to the camera calibration configuration file.
        warp_conf_path (str): Path to the camera warp configuration file.
        car_kinematics_conf_path (str): Path to the car kinematics configuration file.
        steering_conf_path (str): Path to the steering configuration file.
    """
    def __init__(self,
                 camera_resolution=(640, 480),
                 offset_weight=0.9,
                 pid_weights=(0.5, 0.1, 0.1),
                 calibration_conf_path='camera_calibration.yaml',
                 warp_conf_path='camera_warp.yaml',
                 car_kinematics_conf_path='car_kinematics.yaml',
                 steering_conf_path='steering.yaml'):
        """
        self.longitudinal_control = LongitudinalControl()
        self.steering_control = SteeringControl()
        """
        self.camera = Picamera2()
        self.camera_config = self.camera.create_video_configuration(main={"size": camera_resolution})
        self.camera.configure(self.camera_config)

        self.lane_detector = LaneDetector(calibration_conf_path, warp_conf_path)

        self.longitudinal_control = LongitudinalControl(master=None, gui=False)
        self.steering_control = SteeringControl(car_kinematics_conf_path, steering_conf_path)
        self.pid = PID(*pid_weights)

        self.offset_weight = offset_weight
        assert 0.0 <= self.offset_weight <= 1.0, "Offset weight must be in the range [0.0, 1.0]"
        self.heading_weight = 1.0 - offset_weight

        # Records for plotting and analysis.
        self.records = defaultdict(list)

    def update_drive(self, gear: str, speed: float, steering_degree: float):
        """
        Update the car's drive speed and steering angle.
        
        Args:
            speed (float): The forward speed command.
            steering_degree (float): The steering command in degrees (0.0 means straight), value will be camp to [-8.0, 8.0].
        """
        assert self.longitudinal_control.set_gear(gear)
        assert self.longitudinal_control.set_speed(speed) == speed # Make sure speed is set correctly
        self.steering_control.set_angle_degrees(steering_degree)

    def test_drive(self, gear: str, speeds, steering_angles, FPS=1):
        """
        Drives the car at a fixed forward speed.
        """
        assert len(speeds) == len(steering_angles), "Speeds and steering angles must have the same length"

        time_delay = 1.0 / FPS
        print("Starting auto drive...")
        try:
            self.update_drive('Neutral', 0.0, 0.0)  # Reset to neutral at the beginning.

            for speed, steering_angle in zip(speeds, steering_angles):
                print('Gear:', gear, 'Speed:', speed, 'Steering Angle:', steering_angle)
                self.update_drive(gear, speed, steering_angle)
                time.sleep(time_delay)

            self.update_drive('Neutral', 0.0, 0.0)  # Reset to neutral at the end.
        except KeyboardInterrupt:
            print("Auto drive stopped by the user.")

    def get_steer(self, pid_gain):
        """  
        Convert the control signal to steering angle.  

        Args:  
            pid_gain (float): The control signal  

        Returns:  
            steering_angle: The steering angle in degrees.
        """  
        # Assuming a linear mapping from control signal to steering angle  
        # This can be adjusted based on the specific vehicle dynamics  
        min_pid_gain = -10   # Adjust based on realistic PID gain distributions
        max_pid_gain = 10
        max_steering_angle = 8.0 / 180 * math.pi  # Convert degrees to radians
        min_steering_angle = -8.0 / 180 * math.pi

        steering_angle = min_steering_angle + ((pid_gain - min_pid_gain) * (max_steering_angle - min_steering_angle)   
                                           / (max_pid_gain - min_pid_gain))
        # For example, if d-offset is negative (left to the center line), the error is positive, gain is positive.
        # Should steer right.
        return steering_angle
    
    def auto_drive_forward(self, fixed_speed, FPS, view=True, debug=False):
        """
        Drives the car forward at a fixed speed. And steering with camera lane detection.
        """
        sleep_time = 1.0 / FPS

        self.camera.start()
        print("Camera started successfully")
        start_time = time.time()
        cur_time = 0.0
        pre_time = 0.0
        target_offset = 0.0
        target_heading = 0.0
        control_target = target_offset * self.offset_weight + target_heading * self.heading_weight
        try:
            print("Press Ctrl+C to exit the program.") 
            while True:
                # Capture image from camera
                frame = self.camera.capture_array()

                cur_time = time.time()
                delta_time = cur_time - pre_time

                # Process the frame for lane detection.
                offset_pixel, heading_degree = self.lane_detector.process_frame(frame)
                observation = offset_pixel * self.offset_weight + heading_degree * self.heading_weight
                pid_gain = self.pid.update(control_target, observation, delta_time)
                steering_angle = self.get_steer(pid_gain)

                self.update_drive('Forward', fixed_speed, steering_angle)

                self.records['time'].append(cur_time - start_time)
                self.records['offset'].append(offset_pixel)
                self.records['heading'].append(heading_degree)
                self.records['observation'].append(observation)
                self.records['pid_gain'].append(pid_gain)
                self.records['steering_angle'].append(steering_angle)

                pre_time = cur_time
                if view:
                    _put_text(frame, f"Time: {cur_time - start_time:.2f}", (10, 0))
                    _put_text(frame, f"Offset: {offset_pixel:.2f}", (10, 30))
                    _put_text(frame, f"Heading: {heading_degree:.2f}", (10, 60))
                    _put_text(frame, f"PID Gain: {pid_gain:.2f}", (10, 90))
                    _put_text(frame, f"Steering Angle: {steering_angle:.2f}", (10, 120))

                    cv2.imshow("Raw frame", frame)
                if debug:
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('q'):
                        break
                time.sleep(sleep_time)
        except KeyboardInterrupt:
            print("Stopped by the user.")
        finally:
            self.update_drive('Neutral', 0.0, 0.0)

    def plot_records(self):
        """
        Plot the records of the car's movement and steering.
        """
        plt.figure(figsize=(12, 8))

        plt.subplot(3, 2, 1)
        plt.plot(self.records['time'], self.records['offset'], label='Offset')
        plt.title('Offset over Time')
        plt.xlabel('Time (s)')
        plt.ylabel('Offset (pixels)')
        plt.grid()

        plt.subplot(3, 2, 2)
        plt.plot(self.records['time'], self.records['heading'], label='Heading')
        plt.title('Heading over Time')
        plt.xlabel('Time (s)')
        plt.ylabel('Heading (degrees)')
        plt.grid()

        plt.subplot(3, 2, 3)
        plt.plot(self.records['time'], self.records['observation'], label='Observation')
        plt.title('Observation over Time')
        plt.xlabel('Time (s)')
        plt.ylabel('Observation')
        plt.grid()

        plt.subplot(3, 2, 4)
        plt.plot(self.records['time'], self.records['pid_gain'], label='PID Gain')
        plt.title('PID Gain over Time')
        plt.xlabel('Time (s)')
        plt.ylabel('PID Gain')
        plt.grid()

        plt.subplot(3, 2, 5)
        plt.plot(self.records['time'], self.records['steering_angle'], label='Steering Angle')
        plt.title('Steering Angle over Time')
        plt.xlabel('Time (s)')
        plt.ylabel('Steering Angle (degrees)')
        plt.grid()

        # Adjust layout and show the plot
        plt.tight_layout()
        plt.show()


def main(args):
    controller = CarController(camera_resolution=(640, 480),
                               calibration_conf_path='../conf/camera_calibration.yaml',
                               warp_conf_path='../conf/camera_warp.yaml',
                               car_kinematics_conf_path=
                                '../conf/car_kinematics.yaml',
                               steering_conf_path=
                               '../conf/steering.yaml')
    if args.mode == "test_car":
        speeds = [] 
        angles = []
        for i in range(50):
            speeds.append(args.speed)
            angles.append(math.sin(i / 10.0) * 8.0)  # The max abs steering angle is about 8.0 degrees.
        controller.test_drive('Forward', speeds, angles, FPS=args.fps)
    elif args.mode == "auto_drive":
        controller.auto_drive_forward(fixed_speed=args.speed, FPS=args.fps)
        controller.plot_records()
    elif args.mode == "debug":
        controller.auto_drive_forward(fixed_speed=args.speed, FPS=args.fps, debug=True)
    else:
        print("Invalid mode. Use 'test_car' or 'auto_drive'.")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Auto RCCar control.")
    parser.add_argument("mode", 
                        choices=["test", "auto", "debug"], 
                        help="Mode to run: 'test' for test mechanism, 'auto' for camera-guided drive, 'debug' for frame level debug.")
    parser.add_argument("--speed", type=float, default=37.0, help="Fixed drive speed.")
    parser.add_argument("--fps", type=int, default=10, help="Frames per second to drive the car.")
    args = parser.parse_args()

    main(args)
