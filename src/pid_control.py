import numpy as np


class PID:  
    def __init__(self, dt, kp, ki, kd):  
        """  
        Initialize the PID controller.  

        Args:  
            dt (float): Time step (delta time)  
            kp (float): Proportional gain  
            ki (float): Integral gain  
            kd (float): Derivative gain  
        """  
        self.dt = dt    # Delta time  
        self.kp = kp    # Proportional coefficient  
        self.ki = ki    # Integral coefficient  
        self.kd = kd    # Derivative coefficient  

        # Initialize error terms  
        self.prev_error = 0.0    # For storing the previous error (used in derivative term)  
        self.integral = 0.0      # Accumulated integral of the error  

    def update(self, setpoint, measurement):  
        """  
        Compute the control output using the PID formula.  

        Args:  
            setpoint (float): The desired target value  
            measurement (float): The current measured value  

        Returns:  
            float: The control output  
        """  
        # Calculate the error  
        error = setpoint - measurement  

        # Proportional term  
        proportional = self.kp * error  

        # Integral term  
        self.integral += error * self.dt  
        integral = self.ki * self.integral  

        # Derivative term (based on the rate of error change)  
        derivative = self.kd * (error - self.prev_error) / self.dt  

        # Save the current error for the next computation  
        self.prev_error = error  

        # Calculate and return total control output  
        pid_gain = proportional + integral + derivative  
        return pid_gain
    
    def get_steer(self, pid_gain):
        """  
        Convert the control signal to steering angle.  

        Args:  
            pid_gain (float): The control signal  

        Returns:  
            direction (str): The direction of the steering ('left' or 'right')
            angle_value: The abs steering angle in radians.
        """  
        # Assuming a linear mapping from control signal to steering angle  
        # This can be adjusted based on the specific vehicle dynamics  
        max_steering_angle = 1.0
        min_steering_angle = -1.0
        
        angle = np.clip(pid_gain, min_steering_angle, max_steering_angle)
        # Determine the direction based on the sign of the angle.
        # For example, if d-offset is negative (left to the center line), the error is positive, gain is positive.
        # Should steer right.
        direction = 'left' if angle < 0 else 'right'  
        return direction, abs(angle)
