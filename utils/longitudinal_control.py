import tkinter as tk
import pigpio

class LongitudinalControl:
    def __init__(self, master):
        self.master = master
        self.master.title("Longitudinal Control")
        
        # Initialize pigpio and set GPIO pin
        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise Exception("Cannot connect to pigpio daemon")
        self.gpio = 13  # GPIO13 for PWM output
        self.pi.set_mode(self.gpio, pigpio.OUTPUT)

        # Set PWM range (0 to 1000 for finer control)
        self.pi.set_PWM_range(self.gpio, 1000)  # Range of 0-1000 for duty cycle

        # Initial configuration values (in milliseconds)
        self.f_pwm = 50    # PWM frequency in Hz (default 50 Hz)
        self.neutral_pw = 1.35  # Neutral pulse width in ms
        self.delta_pw = 0.25     # Max speed pulse width in ms
        self.slider_value = 0  # Slider initial position (0 to 100, starts at stop)
        
        # Gear selection: -1 (Backward), 0 (Neutral), 1 (Forward)
        self.gear = 0  # Start in Neutral

        # Set initial PWM frequency
        self.pi.set_PWM_frequency(self.gpio, self.f_pwm)

        # GUI Elements
        # PWM Frequency Input
        tk.Label(master, text="PWM Frequency (Hz):").grid(row=0, column=0, padx=5, pady=5)
        self.entry_f_pwm = tk.Entry(master)
        self.entry_f_pwm.insert(0, str(self.f_pwm))
        self.entry_f_pwm.grid(row=0, column=1, padx=5, pady=5)

        # Neutral Pulse Width Input
        tk.Label(master, text="Neutral Pulse Width (ms):").grid(row=1, column=0, padx=5, pady=5)
        self.entry_neutral = tk.Entry(master)
        self.entry_neutral.insert(0, f"{self.neutral_pw:.2f}")
        self.entry_neutral.grid(row=1, column=1, padx=5, pady=5)

        # Max Speed Pulse Width Input
        tk.Label(master, text="Max Speed Pulse Width (ms):").grid(row=2, column=0, padx=5, pady=5)
        self.entry_max_speed = tk.Entry(master)
        self.entry_max_speed.insert(0, f"{self.delta_pw:.2f}")
        self.entry_max_speed.grid(row=2, column=1, padx=5, pady=5)

        # Gear Selection
        tk.Label(master, text="Gear:").grid(row=3, column=0, padx=5, pady=5)
        self.gear_var = tk.StringVar(value="Neutral")
        self.gear_menu = tk.OptionMenu(master, self.gear_var, "Neutral", "Forward", "Backward", 
                                     command=self.update_gear)
        self.gear_menu.grid(row=3, column=1, padx=5, pady=5)

        # Apply Button
        self.apply_button = tk.Button(master, text="Apply", command=self.apply_settings)
        self.apply_button.grid(row=4, column=0, columnspan=2, pady=5)  # Span both columns

        # Slider (0 to 100, 0 = stop, 100 = max speed)
        self.slider = tk.Scale(master, from_=0, to=100, orient=tk.HORIZONTAL, 
                              command=self.update_slider, length=300)
        self.slider.set(self.slider_value)
        self.slider.grid(row=5, column=0, columnspan=2, pady=5)

        # Stop Button
        self.stop_button = tk.Button(master, text="Stop", command=self.stop)
        self.stop_button.grid(row=6, column=0, columnspan=2, pady=5)

        # Display Labels
        self.label_f = tk.Label(master, text=f"Current f: {1000/self.neutral_pw:.2f} Hz")
        self.label_f.grid(row=7, column=0, columnspan=2, pady=5)
        self.label_pw = tk.Label(master, text=f"Current pw: {self.neutral_pw:.2f} ms")
        self.label_pw.grid(row=8, column=0, columnspan=2, pady=5)

        # Set initial servo position
        self.update_servo()

        # Handle window close event
        self.master.protocol("WM_DELETE_WINDOW", self.on_closing)

    def update_gear(self, value):
        """Update gear selection."""
        gear_map = {"Neutral": 0, "Forward": -1, "Backward": 1}
        self.gear = gear_map[value]
        self.update_servo()

    def update_slider(self, value):
        """Update servo position when slider is moved."""
        self.slider_value = float(value)
        self.update_servo()

    def update_servo(self):
        """Calculate pulse width, set PWM duty cycle, and update display."""
        # Calculate pulse width delta (linear from neutral to max)
        pw_offset = self.delta_pw * (self.slider_value / 100)
        
        # Apply gear direction (Neutral stays at neutral_pw, Forward/Backward adjust from neutral)
        if self.gear == 0:  # Neutral
            final_pw = self.neutral_pw
        else:  # Forward or Backward
            final_pw = self.neutral_pw + (pw_offset * self.gear)
        
        # Get current PWM frequency
        current_freq = self.pi.get_PWM_frequency(self.gpio)
        
        # Calculate period in milliseconds
        period_ms = (1 / current_freq) * 1000
        
        # Calculate duty cycle
        duty = final_pw / period_ms if period_ms > 0 else 0
        duty_cycle = int(duty * 1000)  # Scale to PWM range (0-1000)
        
        # Set PWM duty cycle
        self.pi.set_PWM_dutycycle(self.gpio, duty_cycle)
        
        # Update display
        current_freq = 1000 / final_pw if final_pw > 0 else 0
        self.label_f.config(text=f"Current f: {current_freq:.2f} Hz")
        self.label_pw.config(text=f"Current pw: {final_pw:.2f} ms")

    def apply_settings(self):
        """Apply user-entered configuration settings."""
        try:
            # Get new values from entries
            new_f_pwm = float(self.entry_f_pwm.get())
            new_neutral = float(self.entry_neutral.get())
            new_max_speed = float(self.entry_max_speed.get())
            
            # Update internal variables
            self.f_pwm = new_f_pwm
            self.neutral_pw = new_neutral
            self.delta_pw = new_max_speed
            
            # Validate inputs
            if self.f_pwm <= 0 or self.neutral_pw <= self.delta_pw:
                raise ValueError("Invalid values: f_pwm must be > 0, neutral_pw > delta_pw")
            
            # Set new PWM frequency
            self.pi.set_PWM_frequency(self.gpio, int(self.f_pwm))
            
            # Update servo with new settings
            self.update_servo()
        except ValueError as e:
            print(f"Error applying settings: {e}")

    def stop(self):
        """Set slider and servo to stop position (0)."""
        self.slider.set(0)
        self.slider_value = 0
        self.update_servo()

    def on_closing(self):
        """Clean up when closing the window."""
        self.pi.set_PWM_dutycycle(self.gpio, 0)  # Stop PWM
        self.pi.stop()  # Stop pigpio
        self.master.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = LongitudinalControl(root)
    root.mainloop()