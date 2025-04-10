import cv2
import numpy as np
from picamera2 import Picamera2
import time
import sys
from datetime import datetime

def show_available_modes():
    """Display all available camera sensor modes."""
    camera = Picamera2()
    sensor_modes = camera.sensor_modes
    
    print("Available sensor modes:")
    for i, mode in enumerate(sensor_modes):
        print(f"Mode {i}:")
        # Dictionary of friendly names for known keys
        friendly_names = {
            'size': 'Resolution',
            'format': 'Pixel Format',
            'fps': 'Max Frame Rate',
            'crop': 'Crop Area',
            'bit_depth': 'Bit Depth',
            'packed': 'Packed Format',
            'unpacked': 'Unpacked Format',
            'crop_limits': 'Crop Limits',
            'exposure_limits': 'Exposure Limits (min, max, default)'
            # Add more mappings as needed
        }
        
        # Print all key-value pairs with friendly names if available
        for key, value in mode.items():
            friendly_name = friendly_names.get(key, key.capitalize())
            print(f"  {friendly_name}: {value}")
        print()

def live_capture():
    """Run live camera preview with FPS counter and photo capture."""
    camera = Picamera2()
    # config = camera.create_preview_configuration(main={"size": (640, 480), "format": 'YUV420'})
    config = camera.create_video_configuration(main={"size": (640, 480), "format": 'YUV420'})
    camera.configure(config)
    
    # Print current configuration
    current_config = camera.camera_config
    print("-------------------")
    print(f"Current Resolution: {current_config['main']['size']}")
    print(f"Current Format: {current_config['main']['format']}")
    print("-------------------")
    
    camera.start()
    print("Camera started successfully")
    print("Press 's' to save a photo, 'q' to quit")
    
    # FPS counter
    start_time = time.time()
    frame_count = 0
    
    cv2.namedWindow("Raspberry Pi Camera", cv2.WINDOW_NORMAL)

    FPS = 30
    time_delay = 1.0 / FPS
    
    try:
        while True:
            frame = camera.capture_array()
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            
            # Count frames for FPS
            frame_count += 1
            curr_time = time.time()
            elapsed_time = curr_time - start_time

            if frame_count % 30 == 0:  # Update FPS every 30 frames
                fps = frame_count / elapsed_time
                print(f"Current FPS: {fps:.2f}")
                frame_count = 0
                start_time = curr_time
            
            cv2.imshow("Raspberry Pi Camera", frame)
            key = cv2.waitKey(1) & 0xFF
            
            # Check for 'q' to quit
            if key == ord('q'):
                break
            
            # Check for 's' to save photo
            if key == ord('s'):
                # Generate filename with current timestamp
                timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                filename = f"photo_{timestamp}.jpg"
                cv2.imwrite(filename, frame)
                print(f"Photo saved as {filename}")
            
            time.sleep(time_delay)  # Control the frame rate
    
    except Exception as e:
        print(f"An error occurred: {str(e)}")
    
    finally:
        camera.stop()
        cv2.destroyAllWindows()
        print("Camera stopped and window closed")

def main():
    # Check command-line arguments
    if len(sys.argv) > 1 and sys.argv[1] == "modes":
        show_available_modes()
    else:
        live_capture()

if __name__ == "__main__":
    main()