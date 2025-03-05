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
        self.f_min = 773   # Far right frequency in Hz (arbitrary default)
        self.f_max = 1049   # Far left frequency in Hz (arbitrary default)
        self.slider_value = 50  # Slider initial position (0 to 100)

        # Set initial PWM frequency
        self.pi.set_PWM_frequency(self.gpio, self.f_pwm)

        # GUI Elements
        # PWM Frequency Input
        tk.Label(master, text="PWM Frequency (Hz):").grid(row=0, column=0, padx=5, pady=5)
        self.entry_f_pwm = tk.Entry(master)
        self.entry_f_pwm.insert(0, str(self.f_pwm))
        self.entry_f_pwm.grid(row=0, column=1, padx=5, pady=5)

        # Far Right Frequency Input
        tk.Label(master, text="Far Right Frequency (Hz):").grid(row=1, column=0, padx=5, pady=5)
        self.entry_f_min = tk.Entry(master)
        self.entry_f_min.insert(0, str(self.f_min))
        self.entry_f_min.grid(row=1, column=1, padx=5, pady=5)

        # Far Left Frequency Input
        tk.Label(master, text="Far Left Frequency (Hz):").grid(row=2, column=0, padx=5, pady=5)
        self.entry_f_max = tk.Entry(master)
        self.entry_f_max.insert(0, str(self.f_max))
        self.entry_f_max.grid(row=2, column=1, padx=5, pady=5)

        # Apply Button
        self.apply_button = tk.Button(master, text="Apply", command=self.apply_settings)
        self.apply_button.grid(row=3, column=0, pady=5)

        # Save Button
        self.save_button = tk.Button(master, text="Save", command=self.save_settings)
        self.save_button.grid(row=3, column=1, pady=5)

        # Slider (0 to 100, mapped to 1-2 ms pulse width)
        self.slider = tk.Scale(master, from_=0, to=100, orient=tk.HORIZONTAL, 
                              command=self.update_slider, length=300)
        self.slider.set(self.slider_value)
        self.slider.grid(row=4, column=0, columnspan=2, pady=5)

        # Center Button
        self.center_button = tk.Button(master, text="Center", command=self.center)
        self.center_button.grid(row=5, column=0, columnspan=2, pady=5)

        # Display Labels
        self.label_f = tk.Label(master, text="Current f: 150.00 Hz")
        self.label_f.grid(row=6, column=0, columnspan=2, pady=5)
        self.label_pw = tk.Label(master, text="Current pw: 1.50 ms")
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
        """Calculate pulse width, set PWM duty cycle, and update display."""
       # Calculate current frequency for display (mapped between f_min and f_max)
        f = self.f_min + (self.f_max - self.f_min) * (self.slider_value / 100)
        
        control_ms_out = (1 / f) * 1000
        
        # Get current PWM frequency
        current_freq = self.pi.get_PWM_frequency(self.gpio)
        
        # Calculate period in milliseconds (inverse of frequency)
        period_ms = (1 / current_freq) * 1000  # e.g., 20 ms at 50 Hz
        
        # Calculate duty cycle as fraction of period
        duty = control_ms_out / period_ms  # e.g., 1.5 ms / 20 ms = 0.075
        duty_cycle = int(duty * 1000)  # Scale to PWM range (0-1000)
        
        # Set PWM duty cycle
        self.pi.set_PWM_dutycycle(self.gpio, duty_cycle)
        
        # Update display
        self.label_f.config(text=f"Current f: {f:.2f} Hz")
        self.label_pw.config(text=f"Current pw: {control_ms_out:.2f} ms")

    def apply_settings(self):
        """Apply user-entered configuration settings."""
        try:
            # Get new values from entries
            new_f_pwm = float(self.entry_f_pwm.get())
            new_f_min = float(self.entry_f_min.get())
            new_f_max = float(self.entry_f_max.get())
            
            # Update internal variables
            self.f_pwm = new_f_pwm
            self.f_min = new_f_min
            self.f_max = new_f_max
            
            # Validate inputs
            if self.f_pwm <= 0 or self.f_min >= self.f_max:
                raise ValueError("Invalid values: f_pwm must be > 0, f_min < f_max")
            
            # Set new PWM frequency
            self.pi.set_PWM_frequency(self.gpio, int(self.f_pwm))
            
            # Update servo with new settings
            self.update_servo()
        except ValueError as e:
            # Display error (for simplicity, print to console)
            print(f"Error applying settings: {e}")

    def save_settings(self):
        """Save current settings to steering.yaml."""
        try:
            # Prepare data to save
            settings = {
                'steering': {
                    'pwm_frequency_hz': self.f_pwm,
                    'far_left_frequency_hz': self.f_min,
                    'far_right_frequency_hz': self.f_max,
                    'pwm_range': self.pi.get_PWM_range(self.gpio)
                }
            }
            
            # Write to steering.yaml
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