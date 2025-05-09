import argparse
import ast
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
                 pid_weights=(0.1, 0.1, 0.1),
                 calibration_conf_path='camera_calibration.yaml',
                 warp_conf_path='camera_warp.yaml',
                 car_kinematics_conf_path='car_kinematics.yaml',
                 steering_conf_path='steering.yaml'):

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
        assert self.longitudinal_control.set_speed(speed) == speed # Make sure speed is correctly set.
        self.steering_control.set_angle_degrees(steering_degree)

    def test_drive(self, gear: str, speeds, steering_angles, FPS=1):
        """
        Drives the car following the given speeds and steering angles.
        """
        assert len(speeds) == len(steering_angles), "Speeds and steering angles must have the same length"

        time_delay = 1.0 / FPS
        print("Starting test drive...")
        try:
            self.update_drive('Neutral', 0.0, 0.0)  # Reset to neutral at the beginning.

            for speed, steering_angle in zip(speeds, steering_angles):
                print('Gear:', gear, 'Speed:', speed, 'Steering Angle:', steering_angle)
                self.update_drive(gear, speed, steering_angle)
                time.sleep(time_delay)

            self.update_drive('Neutral', 0.0, 0.0)  # Reset to neutral at the end.
        except KeyboardInterrupt:
            print("Auto drive stopped by the user.")

    def get_steer(self, pid_gain, weight, max_steering_angle=8.0):
        """  
        Convert the control signal to steering angle.  

        Args:  
            pid_gain (float): The control signal.
            weight (float): The weight for the PID gain. Smaller values means less angle for same gain.

        Returns:  
            steering_angle: The steering angle in degrees.
        """  
        # This can be adjusted based on the specific vehicle dynamics  
        pid_gain = math.tanh(pid_gain * weight)  # Normalize the PID gain to be between -1 and 1.
        min_pid_gain = -1.0   # Adjust based on realistic PID gain distributions
        max_pid_gain = 1.0
        max_steering_angle = max_steering_angle / 180 * math.pi  # Convert degrees to radians
        min_steering_angle = -1.0 * max_steering_angle

        steering_angle = min_steering_angle + ((pid_gain - min_pid_gain) * (max_steering_angle - min_steering_angle)   
                                           / (max_pid_gain - min_pid_gain))
        # For example, if d-offset is negative (left to the center line), the error is positive, gain is positive.
        # Should steer right.
        return steering_angle * 180 / math.pi  # Convert radians to degrees
    
    def auto_drive_forward(self, fixed_speed, FPS, debug=False):
        """
        Drives the car forward at a fixed speed. And steering with camera lane detection.
        """
        sleep_time = 1.0 / FPS

        self.camera.start()
        print("Camera started successfully")

        start_time = time.time()
        cur_time = 0.0
        pre_time = time.time()
        target_offset = 0.0
        target_heading = 0.0
        control_target = target_offset * self.offset_weight + target_heading * self.heading_weight

        print("Press q on camera window to exit the program.") 
        try:
            while True:
                # Capture image from camera
                frame = self.camera.capture_array()

                cur_time = time.time()
                delta_time = cur_time - pre_time

                # Process the frame for lane detection.
                offset_pixel, heading_degree = self.lane_detector.process_frame(frame)
                if offset_pixel is None or heading_degree is None:
                    print("No offset or heading detected. Skipping frame and keep straight.")
                    offset_pixel = 0.0
                    heading_degree = 0.0
                observation = offset_pixel * self.offset_weight + heading_degree * self.heading_weight
                pid_gain = self.pid.update(control_target, observation, delta_time)
                steering_deg = self.get_steer(pid_gain, weight=args.steer_weight)

                self.update_drive('Forward', fixed_speed, steering_deg)

                self.records['time'].append(cur_time - start_time)
                self.records['offset'].append(offset_pixel)
                self.records['heading'].append(heading_degree)
                self.records['observation'].append(observation)
                self.records['pid_gain'].append(pid_gain)
                self.records['steering_angle'].append(steering_deg)

                pre_time = cur_time

                _put_text(frame, f"Time: {cur_time - start_time:.2f}", (10, 30))
                _put_text(frame, f"Offset: {offset_pixel:.2f}", (10, 60))
                _put_text(frame, f"Heading: {heading_degree:.2f}", (10, 90))
                _put_text(frame, f"PID Gain: {pid_gain:.2f}", (10, 120))
                _put_text(frame, f"Steering Angle(deg): {steering_deg:.2f}", (10, 150))
                cv2.imshow("Frame", frame)

                time.sleep(sleep_time)

                if debug:
                    self.update_drive('Forward', 0.0, steering_deg)  # Stop the car for debugging
                    key = cv2.waitKey(0) & 0xFF
                    if key == ord('q'):
                        break
                    if key == ord('r'):
                        self.pid.reset()  # Reset the PID controller.
                        print("PID controller reset.")
                        self.lane_detector.reset()  # Reset the lane detector.
                        print("Lane detector reset.")
                        self.update_drive('Neutral', 0.0, 0.0)  # Stop the car.
                        print("Car stopped and no steering reset.")

        except Exception as e:
            print(f"An error occurred:\n{str(e)}")
            import traceback
            traceback.print_exc()
        finally:
            # cv2.destroyAllWindows()
            self.camera.stop()
            self.update_drive('Neutral', 0.0, 0.0)  # Make sure the car stops.

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
    pid_weights = ast.literal_eval(args.pid)
    controller = CarController(camera_resolution=(640, 480),
                               calibration_conf_path='../conf/camera_calibration.yaml',
                               pid_weights=pid_weights,
                               warp_conf_path='../conf/camera_warp.yaml',
                               car_kinematics_conf_path=
                                '../conf/car_kinematics.yaml',
                               steering_conf_path=
                               '../conf/steering.yaml')
    if args.mode == "test":
        speeds = [] 
        angles = []
        for i in range(50):
            speeds.append(args.speed)
            angles.append(math.sin(i / 10.0) * 8.0)  # The max abs steering angle is about 8.0 degrees.
        controller.test_drive('Forward', speeds, angles, FPS=args.fps)
    elif args.mode == "auto":
        controller.auto_drive_forward(fixed_speed=args.speed, FPS=args.fps)
        controller.plot_records()
    elif args.mode == "debug":
        controller.auto_drive_forward(fixed_speed=args.speed, FPS=args.fps, debug=True)
    else:
        print("Invalid mode.")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Auto RCCar control.")
    parser.add_argument("mode", 
                        choices=["test", "auto", "debug"], 
                        help="Mode to run: 'test' for test mechanism, 'auto' for camera-guided drive, 'debug' for camera-guided drive with frame level debug.")
    parser.add_argument("--speed", type=float, default=37.0, help="Fixed drive speed.")
    parser.add_argument("--fps", type=int, default=10, help="Frames per second to drive the car.")
    parser.add_argument("--pid", type=str, default="(0.1, 0.1, 0.1)", help="Weights for PID controller.")
    parser.add_argument("--steer_weight", type=float, default=1.0, help="Weight for steering control. Less weight means less steer for the same pid gain.")
    args = parser.parse_args()

    main(args)
