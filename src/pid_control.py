import numpy as np


class PID:  
    def __init__(self, kp, ki, kd):  
        """  
        Initialize the PID controller.  

        Args:  
            kp (float): Proportional gain  
            ki (float): Integral gain  
            kd (float): Derivative gain  
        """  
        self.kp = kp    # Proportional coefficient  
        self.ki = ki    # Integral coefficient  
        self.kd = kd    # Derivative coefficient  

        # Initialize error terms  
        self.prev_error = 0.0    # For storing the previous error (used in derivative term)  
        self.integral = 0.0      # Accumulated integral of the error  

    def update(self, setpoint, measurement, dt):  
        """  
        Compute the control output using the PID formula.  

        Args:  
            setpoint (float): The desired target value  
            measurement (float): The current measured value  
            dt (float): The time interval since the last update in seconds.

        Returns:  
            float: The control output  
        """  
        # Calculate the error  
        error = setpoint - measurement  

        # Proportional term  
        proportional = self.kp * error  

        # Integral term  
        self.integral += error * dt  
        integral = self.ki * self.integral  

        # Derivative term (based on the rate of error change)  
        derivative = self.kd * (error - self.prev_error) / dt  

        # Save the current error for the next computation  
        self.prev_error = error  

        # Calculate and return total control output  
        pid_gain = proportional + integral + derivative  
        return pid_gain
    
