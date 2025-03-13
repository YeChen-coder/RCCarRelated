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

def warp_image(image_path):
    """
    Main function to process image: undistort and warp to bird's eye view.
    
    Args:
        image_path: Path to the input image
    """
    # Step 1: Undistort the image
    undistorted_img, original_img = undistort_image(image_path)
    
    # Get image dimensions
    height, width = undistorted_img.shape[:2]
    
    # Step 2: Load warp points
    src_points_norm, dst_points_norm = load_warp_points()
    
    # Scale normalized points to image dimensions
    src_points = src_points_norm * np.array([width, height], dtype=np.float32)
    dst_points = dst_points_norm * np.array([width, height], dtype=np.float32)
    
    # Step 3: Warp the image to bird's eye view
    warped_img, M, Minv = warp(undistorted_img, src_points, dst_points)
    
    # Step 4: Display results
    cv2.imshow('Original Image', original_img)
    cv2.imshow('Undistorted Image', undistorted_img)
    cv2.imshow('Bird\'s Eye View', warped_img)
    
    # Draw source points on the undistorted image for visualization
    vis_img = undistorted_img.copy()
    for pt in src_points:
        x, y = int(pt[0]), int(pt[1])
        cv2.circle(vis_img, (x, y), 5, (0, 0, 255), -1)
    
    # Connect the points to show the region being transformed
    cv2.polylines(vis_img, [src_points.astype(np.int32)], True, (0, 255, 0), 2)
    cv2.imshow('Source Points Visualization', vis_img)
    
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    
    return warped_img, M, Minv

def main():
    if len(sys.argv) < 1:
        print("Usage: python camera_warp.py <image_path>")
        return
    image_path = sys.argv[1]
    warp_image(image_path)

if __name__ == "__main__":
    main()