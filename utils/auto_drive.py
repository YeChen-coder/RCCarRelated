import math
import time

from steering_control import SteeringControl
from longitudinal_control import LongitudinalControl

FIXED_FORWARD_SPEED = 10.0  # units per second

class CarController:
    def __init__(self, car_kinematics_conf_path='car_kinematics.yaml', steering_conf_path='steering.yaml'):
        """
        self.longitudinal_control = LongitudinalControl()
        self.steering_control = SteeringControl()
        """
        self.longitudinal_control = LongitudinalControl(master=None, gui=False)
        self.steering_control = SteeringControl(car_kinematics_conf_path, steering_conf_path)

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

def main():
    controller = CarController('../conf/car_kinematics.yaml', '../conf/steering.yaml')

    speeds = [] 
    angles = []
    for i in range(50):
        speeds.append(FIXED_FORWARD_SPEED)
        angles.append(math.sin(i / 10.0) * 8.0)  # The max abs steering angle is about 8.0 degrees.
    controller.test_drive('Forward', speeds, angles, FPS=5)

if __name__ == "__main__":
    main()
