import math
import yaml  # Requires 'pyyaml' package: sudo apt-get install python3-yaml

import pigpio
import numpy as np

from steering_calibration import SteeringServoControl
from time import sleep

class SteeringControl:
    """
    Class to control the steering of a car using a servo motor.
    
    Args:
        servoControl: SteeringServoControl object
    """
    def __init__(self, car_kinematics_conf_path='car_kinematics.yaml', steering_conf_path='steering.yaml'):
        self.servoControl = SteeringServoControl(steering_conf_path, master=None, gui=False)

        with open(car_kinematics_conf_path, 'r') as file:
            yml_file = yaml.safe_load(file)
            car_params = yml_file["car_kinematics"]
        
        required_keys = ["wheel_base", "left_turing_radius", "right_turing_radius"]
        for key in required_keys:
            if key not in car_params:
                raise KeyError(f"Missing required key: {key}")

        left_turning_angle = math.atan(car_params["wheel_base"] / car_params["left_turing_radius"])
        right_turning_angle = math.atan(car_params["wheel_base"] / car_params["right_turing_radius"])

        car_params["maximum_left_turning_angle"] = left_turning_angle
        car_params["maximum_right_turning_angle"] = right_turning_angle
        self.car_params = car_params

        print("Car kinematics parameters loaded successfully:")
        print(self.car_params)

    def set_angle_degrees(self, angle: float):
        """
        Set the steering angle in degrees.
        Args:
            angle: Steering angle in degrees (angle < 0: left, angle > 0: right)
                   value will be camp to [-8.0, 8.0].
        """
        # Convert degrees to radians
        angle_radians = math.radians(angle)
        self.set_angle_radians(angle_radians)

    def set_angle_radians(self, angle: float):
        """
        Set the steering angle of the car.
        
        Args:
            angle: Steering angle in radians (angle < 0: left, angle > 0: right)

        Notes: 
            - The steering angle is capped to the maximum turning angle
            - For the real RC car kinematics, since turning angle is a relatively small angle,
                we can approximate the relationship between servo angle and steering angle to be linear
        """
        # Capped the angle to the maximum turning angle
        if angle < 0:
            angle = -self.car_params["maximum_left_turning_angle"] if abs(angle) > abs(self.car_params["maximum_left_turning_angle"]) else angle
            steering_value = angle / self.car_params["maximum_left_turning_angle"] * 50 + 50
            self.servoControl.set_steering(steering_value)
            self.servoControl.update_servo()
        elif angle > 0:
            angle = self.car_params["maximum_right_turning_angle"] if abs(angle) > abs(self.car_params["maximum_right_turning_angle"]) else angle
            steering_value = angle / self.car_params["maximum_right_turning_angle"] * 50 + 50
            self.servoControl.set_steering(steering_value)
        else:
            angle = 10e-3 # Epislon to avoid division by zero
            steering_value = angle * 50 + 50
            self.servoControl.set_steering(steering_value)

def load_car_kinematics(yaml_path='car_kinematics.yaml'):
    """
    Load car kinematics parameters from YAML file.
    
    Args:
        yaml_path: Path to YAML file containing car kinematics parameters

    Returns:
        car_params: Dictionary containing car kinematics parameters
    """
    with open(yaml_path, 'r') as file:
        yml_file = yaml.safe_load(file)
        car_params = yml_file["car_kinematics"]
    
    required_keys = ["wheel_base", "left_turing_radius", "right_turing_radius"]
    for key in required_keys:
        if key not in car_params:
            raise KeyError(f"Missing required key: {key}")

    left_turning_angle = math.atan(car_params["wheel_base"] / car_params["left_turing_radius"])
    right_turning_angle = math.atan(car_params["wheel_base"] / car_params["right_turing_radius"])

    car_params["maximum_left_turning_angle"] = left_turning_angle
    car_params["maximum_right_turning_angle"] = right_turning_angle

    return car_params

def turning_radius_to_angle(wheel_base, turning_radius):
    """
    Calculate the steering angle from the turning radius.
    
    Args:
        wheel_base: Distance between front and rear axles
        turning_radius: Turning radius of the car
        
    Returns:
        steering_angle: Steering angle in radians
    """
    return math.atan(wheel_base / turning_radius)

def turning_angle_to_radius(wheel_base, steering_angle):
    """
    Calculate the turning radius from the steering angle.
    
    Args:
        wheel_base: Distance between front and rear axles
        steering_angle: Steering angle in radians
        
    Returns:
        turning_radius: Turning radius of the car
    """
    return wheel_base / math.tan(steering_angle)

def set_angle_degrees(angle: float, servoControl):
    angle_radians = math.radians(angle)
    set_angle_radians(angle_radians, servoControl)

def set_angle_radians(angle: float, servoControl):
    """
    Set the steering angle of the car.
    
    Args:
        angle: Steering angle in radians (angle < 0: left, angle > 0: right)
        servoControl: SteeringServoControl object

    Notes: 
        - The steering angle is capped to the maximum turning angle
        - For the real RC car kinematics, since turning angle is a relatively small angle,
            we can approximate the relationship between servo angle and steering angle to be linear
    """
    car_params = load_car_kinematics()
    # Capped the angle to the maximum turning angle
    if angle < 0:
        angle = -car_params["maximum_left_turning_angle"] if abs(angle) > abs(car_params["maximum_left_turning_angle"]) else angle
        steering_value = angle / car_params["maximum_left_turning_angle"] * 50 + 50
        servoControl.set_steering(steering_value)
        servoControl.update_servo()
    elif angle > 0:
        angle = car_params["maximum_right_turning_angle"] if abs(angle) > abs(car_params["maximum_right_turning_angle"]) else angle
        steering_value = angle / car_params["maximum_right_turning_angle"] * 50 + 50
        servoControl.set_steering(steering_value)
        servoControl.update_servo()
    else:
        angle = 10e-3 # Epislon to avoid division by zero
        steering_value = angle * 50 + 50
        servoControl.set_steering(steering_value)
        servoControl.update_servo()


if __name__ == "__main__":
    steer_control = SteeringControl('../conf/car_kinematics.yaml', '../conf/steering.yaml')

    print('Start to test steering control...')
    for i in np.arange(0.0, 20.0, 5):
        print('++++Steering angle:', i)
        steer_control.set_angle_degrees(i)
        print('----Finish')
        sleep(0.2)
    for i in np.arange(0.0, -20.0, -5):
        print('++++Steering angle:', i)
        steer_control.set_angle_degrees(i)
        print('----Finish')
        sleep(0.2)