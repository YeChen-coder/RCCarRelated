import cv2
import numpy as np
import yaml
import os
import sys
from camera_calib import undistort_image

def warp(img, src, dst):
    """
    Apply perspective transform to get a bird's eye view.
    
    Args:
        img: Input image
        src: Source points for perspective transform
        dst: Destination points for perspective transform
        
    Returns:
        warped: Warped image (bird's eye view)
        M: Transformation matrix
        Minv: Inverse transformation matrix
    """
    img_size = (img.shape[1], img.shape[0])
    M = cv2.getPerspectiveTransform(src, dst)
    # Inverse
    Minv = cv2.getPerspectiveTransform(dst, src)
    warped = cv2.warpPerspective(img, M, img_size)
    return warped, M, Minv

def load_warp_points(yaml_path='camera_warp.yaml'):
    """
    Load source and destination points from YAML file.
    
    Args:
        yaml_path: Path to YAML file containing camera warp points
        
    Returns:
        src_points: Source points array
        dst_points: Destination points array
    """
    with open(yaml_path, 'r') as file:
        warp_data = yaml.safe_load(file)
    
    src_points = np.array(warp_data['camera_src_points_matrix'], dtype=np.float32)
    dst_points = np.array(warp_data['camera_dst_points_matrix'], dtype=np.float32)
    
    return src_points, dst_points

def warp_image_path(image_path, undisorted=False):
    """
    Load image from file, undistort it if necessary, and warp it to bird's eye view.
    
    Args:
        image_path: Path to the input image
        undisorted: The image is already undistorted
    """
    if not undisorted:
        undistorted_img, _ = undistort_image(image_path)
    else:
        undistorted_img = cv2.imread(image_path)

    return warp_image_undistorted(undistorted_img)

def warp_image_undistorted(img):
    """
    Warp an undistorted image to bird's eye view.
    
    Args:
        img: Undistorted input image (ndarray)
    """
    # Get image dimensions
    height, width = img.shape[:2]
    
    # Load warp points
    src_points_norm, dst_points_norm = load_warp_points()
    
    # Scale normalized points to image dimensions
    src_points = src_points_norm * np.array([width, height], dtype=np.float32)
    dst_points = dst_points_norm * np.array([width, height], dtype=np.float32)
    
    # Warp the image to bird's eye view
    warped_img, M, Minv = warp(img, src_points, dst_points)
    
    return warped_img, M, Minv

def warp_image_and_display(image_path, undisorted=False, save=False):
    """
    Main function to display the original and warped images.
    Undistort the image if necessary.
    
    Args:
        image_path: Path to the input image
        undisorted: The image is already undistorted
        save: Save the output image
    """
    warped_img, M, Minv = warp_image_path(image_path, undisorted)
    
    # Display images
    original_img = cv2.imread(image_path)
    
    # Step 4: Display results
    cv2.imshow('Original Image', original_img)
    cv2.imshow('Bird\'s Eye View', warped_img)
    
    if save:
        output_path = os.path.join('warped_' + os.path.basename(image_path))
        cv2.imwrite(output_path, warped_img)
        print("Warped image saved at:", output_path)

    cv2.waitKey(0)
    cv2.destroyAllWindows()

def visualize_warp_points(image_path, undistorted=False):
    """
    Display the original image with source points for warp.
    
    Args:
        image_path: Path to the input image
        undistorted: The image is already undistorted
    """
    if not undistorted:
        img, _ = undistort_image(image_path)
    else:
        img = cv2.imread(image_path)
    
    # Load warp points
    src_points_norm, dst_points_norm = load_warp_points()

    height, width = img.shape[:2]

    src_points = src_points_norm * np.array([width, height], dtype=np.float32)
    dst_points = dst_points_norm * np.array([width, height], dtype=np.float32)
    
    # Draw source points on the image
    vis_img = img.copy()
    for pt in src_points:
        x, y = int(pt[0]), int(pt[1])
        cv2.circle(vis_img, (x, y), 5, (0, 0, 255), -1)
    cv2.polylines(vis_img, [src_points.astype(np.int32)], True, (0, 255, 0), 2)

    # Draw destination points on the image
    for pt in dst_points:
        x, y = int(pt[0]), int(pt[1])
        cv2.circle(vis_img, (x, y), 5, (255, 0, 0), -1)
    cv2.polylines(vis_img, [dst_points.astype(np.int32)], True, (255, 255, 0), 2)

    cv2.imshow('Warp Points Visualization', vis_img)
    
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def main():
    if len(sys.argv) < 2:
        print("Usage: python camera_warp.py [command] [options]")
        print("Commands:")
        print("    warp_image [image_path] [--undistorted] [--save]")  # Warp an image to bird's eye view
        print("    visualize_points [image_path] [--undistorted]")  # Visualize source points for warp
        return

    command = sys.argv[1].lower()

    if command == 'warp_image':
        image_path = sys.argv[2]
        undisorted = '--undistorted' in sys.argv
        save = '--save' in sys.argv
        warp_image_and_display(image_path, undisorted, save)
    elif command == 'visualize_points':
        image_path = sys.argv[2]
        undisorted = '--undistorted' in sys.argv
        visualize_warp_points(image_path, undisorted)
    else:
        print("Error: Invalid command")
        return

if __name__ == "__main__":
    main()