import cv2
import numpy as np
from picamera2 import Picamera2
import time
import sys
import os
from datetime import datetime
import yaml
import random

def collect_images():
    """Collect calibration images every 2 seconds."""
    camera = Picamera2()
    config = camera.create_preview_configuration(main={"size": (2592, 1944)})
    camera.configure(config)
    
    # Create calibration directory if it doesn't exist
    os.makedirs('calibration', exist_ok=True)
    
    print(f"Current Resolution: {config['main']['size']}")
    print(f"Current Format: {config['main']['format']}")
    print("Collecting calibration images every 2 seconds...")
    print("Press 'q' to quit")
    
    camera.start()
    cv2.namedWindow("Calibration Collection", cv2.WINDOW_NORMAL)
    
    try:
        while True:
            frame = camera.capture_array()
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            
            # Save image every 2 seconds
            timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            filename = f"calibration/calib_{timestamp}.jpg"
            cv2.imwrite(filename, frame)
            print(f"Saved: {filename}")
            
            cv2.imshow("Calibration Collection", frame)
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('q'):
                break
            
            time.sleep(2)  # Wait 2 seconds between captures
    
    except Exception as e:
        print(f"An error occurred: {str(e)}")
    
    finally:
        camera.stop()
        cv2.destroyAllWindows()
        print("Collection stopped")

def calibrate_camera():
    """Calibrate camera using random 30 images from calibration folder."""
    # Calibration pattern settings (adjust based on your checkerboard)
    CHECKERBOARD = (7, 10)  # (rows-1, cols-1) of inner corners
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    
    # Prepare object points
    objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
    
    objpoints = []  # 3d points in real world space
    imgpoints = []  # 2d points in image plane
    
    # Load all images from calibration folder
    images = [os.path.join('calibration', f) for f in os.listdir('calibration') 
             if f.endswith(('.jpg', '.png', '.jpeg'))]
    
    if not images:
        print("No calibration images found in 'calibration/' folder")
        return
    
    # List all images
    print("Images found in calibration folder:")
    for i, img_path in enumerate(images, 1):
        print(f"{i}. {os.path.basename(img_path)}")
    
    # Select random 30 images (or all if less than 30)
    num_images_to_use = min(30, len(images))
    selected_images = random.sample(images, num_images_to_use)
    
    print(f"\nSelected {num_images_to_use} random images for calibration:")
    for i, img_path in enumerate(selected_images, 1):
        print(f"{i}. {os.path.basename(img_path)}")
    
    for fname in selected_images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        # Find chessboard corners
        ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)
        
        if ret:
            objpoints.append(objp)
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(corners2)
            
            # Draw and display corners (optional visualization)
            cv2.drawChessboardCorners(img, CHECKERBOARD, corners2, ret)
            cv2.imshow('Calibration', img)
            cv2.waitKey(500)
        else:
            print(f"Chessboard not found in {os.path.basename(fname)}")
    
    cv2.destroyAllWindows()
    
    if not objpoints or not imgpoints:
        print("No valid calibration data found")
        return
    
    # Perform calibration
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, gray.shape[::-1], None, None
    )
    
    # Save calibration data to YAML file
    calibration_data = {
        'camera_matrix': mtx.tolist(),
        'dist_coeff': dist.tolist()
    }
    
    with open('camera_calibration.yaml', 'w') as f:
        yaml.dump(calibration_data, f)
    
    print(f"\nCalibration completed. RMS error: {ret}")
    print("Calibration data saved to camera_calibration.yaml")
    
    # Print calibration results
    print("\nCamera Matrix:")
    print(mtx)
    print("\nDistortion Coefficients:")
    print(dist)

def undistort_image(image_path):
    """Load calibration data and undistort the specified image with scaled parameters."""
    # Check if calibration file exists
    if not os.path.exists('camera_calibration.yaml'):
        print("Error: camera_calibration.yaml calibration file not found")
        return
    
    # Load calibration data
    with open('camera_calibration.yaml', 'r') as f:
        calibration_data = yaml.safe_load(f)
    
    camera_matrix = np.array(calibration_data['camera_matrix'])
    dist_coeffs = np.array(calibration_data['dist_coeff'])
    
    # Load the image
    if not os.path.exists(image_path):
        print(f"Error: Image file not found at {image_path}")
        return
    
    img = cv2.imread(image_path)
    if img is None:
        print(f"Error: Could not load image at {image_path}")
        return
    
    # Get image dimensions
    h, w = img.shape[:2]
    
    # Check 4:3 aspect ratio (within a small tolerance)
    aspect_ratio = w / h
    expected_ratio = 4 / 3  # 2592/1944 = 4:3
    tolerance = 0.05  # 5% tolerance
    
    if not (expected_ratio - tolerance <= aspect_ratio <= expected_ratio + tolerance):
        print(f"Error: Image must have 4:3 aspect ratio. Got {w}x{h} (ratio: {aspect_ratio:.3f})")
        return
    
    # Calculate scaling factors based on original calibration resolution (2592x1944)
    orig_w, orig_h = 2592, 1944
    width_scale = w / orig_w
    height_scale = h / orig_h
    
    # Scale camera matrix parameters
    scaled_camera_matrix = camera_matrix.copy()
    scaled_camera_matrix[0, 0] *= width_scale  # fx
    scaled_camera_matrix[1, 1] *= height_scale  # fy
    scaled_camera_matrix[0, 2] *= width_scale  # cx
    scaled_camera_matrix[1, 2] *= height_scale  # cy
    
    # Get optimal new camera matrix with scaled parameters
    new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(
        scaled_camera_matrix, dist_coeffs, (w, h), 1, (w, h)
    )
    
    # Undistort the image
    undistorted_img = cv2.undistort(
        img, scaled_camera_matrix, dist_coeffs, None, new_camera_matrix
    )
    
    # Crop the image based on ROI
    x, y, w, h = roi
    undistorted_img = undistorted_img[y:y+h, x:x+w]

    return undistorted_img, img

def undistort_image_and_display(image_path):
    undistorted_img, img = undistort_image(image_path)

    h, w = img.shape[:2]
    
    # Calculate scaling factors based on original calibration resolution (2592x1944)
    orig_w, orig_h = 2592, 1944
    width_scale = w / orig_w
    height_scale = h / orig_h

    # Display original and undistorted images
    cv2.namedWindow("Original Image", cv2.WINDOW_NORMAL)
    cv2.namedWindow("Undistorted Image", cv2.WINDOW_NORMAL)
    
    cv2.imshow("Original Image", img)
    cv2.imshow("Undistorted Image", undistorted_img)
    
    print(f"Image size: {w}x{h}")
    print(f"Scaling factors - Width: {width_scale:.4f}, Height: {height_scale:.4f}")
    print("Showing original and undistorted images")
    print("Press any key to close windows")
    
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def main():
    if len(sys.argv) < 2:
        print("Usage: python script.py [command] [options]")
        print("  collect: Collect calibration images every 2 seconds")
        print("  calibr/cali: Perform camera calibration using collected images")
        print("  undistort <image_path>: Undistort specified image using calibration data")
        return
    
    command = sys.argv[1].lower()
    
    if command == "collect":
        collect_images()
    elif command in ["calibr", "cali"]:
        calibrate_camera()
    elif command == "undistort":
        if len(sys.argv) < 3:
            print("Error: Please provide an image path after 'undistort'")
            print("Usage: python script.py undistort <image_path>")
            return
        image_path = sys.argv[2]
        undistort_image_and_display(image_path)
    else:
        print(f"Unknown command: {command}")
        print("Available commands: collect, calibr, cali, undistort")

if __name__ == "__main__":
    main()