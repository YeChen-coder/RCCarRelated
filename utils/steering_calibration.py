import tkinter as tk
import pigpio
import yaml  # Requires 'pyyaml' package: sudo apt-get install python3-yaml

class ServoControl:
    def __init__(self, master):
        self.master = master
        self.master.title("Servo Steering Control")
        
        # Initialize pigpio and set GPIO pin
        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise Exception("Cannot connect to pigpio daemon")
        self.gpio = 12  # GPIO12 for PWM output
        self.pi.set_mode(self.gpio, pigpio.OUTPUT)

        # Set PWM range (0 to 1000 for finer control)
        self.pi.set_PWM_range(self.gpio, 1000)  # Range of 0-1000 for duty cycle

        # Initial configuration values
        self.f_pwm = 50    # PWM frequency in Hz (default 50 Hz for servos)
        self.pw_center = 1.123   # Center pulse width in ms
        self.pw_delta = 0.1702   # Delta pulse width in ms
        self.slider_value = 50   # Slider initial position (0 to 100)

        # Calculate initial frequency bounds
        self.f_min = 1 / (self.pw_center + self.pw_delta)  # Right (max PW)
        self.f_max = 1 / (self.pw_center - self.pw_delta)  # Left (min PW)
        
        # Set initial PWM frequency
        self.pi.set_PWM_frequency(self.gpio, self.f_pwm)

        # GUI Elements
        # PWM Frequency Input (now base frequency)
        tk.Label(master, text="Base PWM Frequency (Hz):").grid(row=0, column=0, padx=5, pady=5)
        self.entry_f_pwm = tk.Entry(master)
        self.entry_f_pwm.insert(0, str(self.f_pwm))
        self.entry_f_pwm.grid(row=0, column=1, padx=5, pady=5)

        # Center Pulse Width Input
        tk.Label(master, text="Center Pulse Width (ms):").grid(row=1, column=0, padx=5, pady=5)
        self.entry_pw_center = tk.Entry(master)
        self.entry_pw_center.insert(0, str(self.pw_center))
        self.entry_pw_center.grid(row=1, column=1, padx=5, pady=5)

        # Delta Pulse Width Input
        tk.Label(master, text="Delta Pulse Width (ms):").grid(row=2, column=0, padx=5, pady=5)
        self.entry_pw_delta = tk.Entry(master)
        self.entry_pw_delta.insert(0, str(self.pw_delta))
        self.entry_pw_delta.grid(row=2, column=1, padx=5, pady=5)

        # Apply Button
        self.apply_button = tk.Button(master, text="Apply", command=self.apply_settings)
        self.apply_button.grid(row=3, column=0, pady=5)

        # Save Button
        self.save_button = tk.Button(master, text="Save", command=self.save_settings)
        self.save_button.grid(row=3, column=1, pady=5)

        # Slider (0 to 100, mapped to frequency range)
        self.slider = tk.Scale(master, from_=0, to=100, orient=tk.HORIZONTAL, 
                              command=self.update_slider, length=300)
        self.slider.set(self.slider_value)
        self.slider.grid(row=4, column=0, columnspan=2, pady=5)

        # Center Button
        self.center_button = tk.Button(master, text="Center", command=self.center)
        self.center_button.grid(row=5, column=0, columnspan=2, pady=5)

        # Display Labels
        self.label_f = tk.Label(master, text="Current f: 0.00 Hz")
        self.label_f.grid(row=6, column=0, columnspan=2, pady=5)
        self.label_pw = tk.Label(master, text="Current pw: 0.00 ms")
        self.label_pw.grid(row=7, column=0, columnspan=2, pady=5)

        # Set initial servo position
        self.update_servo()

        # Handle window close event
        self.master.protocol("WM_DELETE_WINDOW", self.on_closing)

    def update_slider(self, value):
        """Update servo position when slider is moved."""
        self.slider_value = float(value)
        self.update_servo()

    def update_servo(self):
        """Calculate frequency, set PWM, and update display."""
        # Calculate pulse width based on slider (0 to 100 mapped to -delta to +delta)
        pw_offset = (self.slider_value - 50) / 50 * self.pw_delta
        current_pw = self.pw_center + pw_offset
        
        # Calculate frequency from pulse width (f = 1/pw)
        f = 1000 / current_pw
        
        # Calculate duty cycle for base frequency
        period_ms = (1 / self.f_pwm) * 1000  # Base period in ms
        duty = current_pw / period_ms
        duty_cycle = int(duty * 1000)  # Scale to PWM range (0-1000)
        
        # Set PWM duty cycle
        self.pi.set_PWM_dutycycle(self.gpio, duty_cycle)
        
        # Update display
        self.label_f.config(text=f"Current f: {f:.2f} Hz")
        self.label_pw.config(text=f"Current pw: {current_pw:.2f} ms")

    def apply_settings(self):
        """Apply user-entered configuration settings."""
        try:
            # Get new values from entries
            new_f_pwm = float(self.entry_f_pwm.get())
            new_pw_center = float(self.entry_pw_center.get())
            new_pw_delta = float(self.entry_pw_delta.get())
            
            # Update internal variables
            self.f_pwm = new_f_pwm
            self.pw_center = new_pw_center
            self.pw_delta = new_pw_delta
            
            # Validate inputs
            if self.f_pwm <= 0 or self.pw_center <= self.pw_delta or self.pw_delta < 0:
                raise ValueError("Invalid values: f_pwm > 0, pw_center > pw_delta, pw_delta >= 0")
            
            # Recalculate frequency bounds
            self.f_min = 1 / (self.pw_center + self.pw_delta)
            self.f_max = 1 / (self.pw_center - self.pw_delta)
            
            # Set new base PWM frequency
            self.pi.set_PWM_frequency(self.gpio, int(self.f_pwm))
            
            # Update servo with new settings
            self.update_servo()
        except ValueError as e:
            print(f"Error applying settings: {e}")

    def save_settings(self):
        """Save current settings to steering.yaml."""
        try:
            settings = {
                'steering': {
                    'base_pwm_frequency_hz': self.f_pwm,
                    'center_pulse_width_ms': self.pw_center,
                    'delta_pulse_width_ms': self.pw_delta,
                    'pwm_range': self.pi.get_PWM_range(self.gpio)
                }
            }
            with open('steering.yaml', 'w') as file:
                yaml.dump(settings, file, default_flow_style=False)
            print("Settings saved to steering.yaml")
        except Exception as e:
            print(f"Error saving settings: {e}")

    def center(self):
        """Set slider and servo to center position."""
        self.slider.set(50)
        self.slider_value = 50
        self.update_servo()

    def on_closing(self):
        """Clean up when closing the window."""
        self.pi.set_PWM_dutycycle(self.gpio, 0)  # Stop PWM
        self.pi.stop()  # Stop pigpio
        self.master.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = ServoControl(root)
    root.mainloop()