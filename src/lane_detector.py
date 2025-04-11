import os
import sys
import cv2
import numpy as np

from lane_line import LaneLine, predict_other_lane_line, middle_lane_fit, center_offset

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../utils")
from camera_warp import load_warp_points, warp_image_undistorted, warp_image_path
from camera_calib import load_calibration_data, undistort_image, undistort_image_path

def _check_overlap(window1, window2):
    """
    Check if two windows overlap.
    Each window is represented by its top-left (x1, y1) and bottom-right (x2, y2) coordinates.
    
    Args:
        window1: tuple of ((top_left_x, top_left_y), (bottom_right_x, bottom_right_y))
        window2: tuple of ((top_left_x, top_left_y), (bottom_right_x, bottom_right_y))
    
    Returns:
        True if windows overlap, False otherwise
    """
    # Extract coordinates
    (x1_tl, y1_tl), (x1_br, y1_br) = window1
    (x2_tl, y2_tl), (x2_br, y2_br) = window2
    
    # Check if one window is to the left of the other
    if x1_br < x2_tl or x2_br < x1_tl:
        return False
    
    # Check if one window is above the other
    if y1_br < y2_tl or y2_br < y1_tl:
        return False
    
    # If neither of the above conditions is true, windows overlap
    return True

# Color space transformation
def _color_space_transform(image):
    # Convert to HSV
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    # Define lower and upper thresholds for white
    lower_white = np.array([0, 0, 180])   # Minimum white (high Value)
    upper_white = np.array([180, 50, 255]) # Maximum white (low Saturation)
    
    # Thresholding to extract white
    mask = cv2.inRange(hsv, lower_white, upper_white)
    
    kernel = np.ones((3,3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    
    edges = cv2.Canny(mask, 50, 150)  # Adjust thresholds as needed
    
    return mask, edges

def _remove_overlapping_windows(windows_a, windows_b, mode='remove_a_keep_b'):
    """
    Remove windows that overlap between two sets based on the specified mode.
    
    Args:
        windows_a: numpy array of windows, each window is ((top_left_x, top_left_y), (bottom_right_x, bottom_right_y))
        windows_b: numpy array of windows, each window is ((top_left_x, top_left_y), (bottom_right_x, bottom_right_y))
        mode: string that controls which windows to remove:
              'remove_a_keep_b' - remove overlapping windows from set A, keep all windows in set B
              'remove_b_keep_a' - remove overlapping windows from set B, keep all windows in set A
    
    Returns:
        If mode is 'remove_a_keep_b': filtered_windows_a, windows_b
        If mode is 'remove_b_keep_a': windows_a, filtered_windows_b
    """

    # Ensure windows_a and windows_b are numpy arrays
    windows_a = np.array(windows_a)
    windows_b = np.array(windows_b)

    if mode not in ['remove_a_keep_b', 'remove_b_keep_a']:
        raise ValueError("Mode must be either 'remove_a_keep_b' or 'remove_b_keep_a'")
    
    if mode == 'remove_a_keep_b':
        # Remove overlapping windows from set A
        non_overlapping_a = []
        
        for window_a in windows_a:
            overlaps = False
            
            for window_b in windows_b:
                if _check_overlap(window_a, window_b):
                    overlaps = True
                    break
            
            if not overlaps:
                non_overlapping_a.append(window_a)
        
        # Convert list back to numpy array
        if non_overlapping_a:
            filtered_a = np.array(non_overlapping_a)
        else:
            filtered_a = np.empty((0, 2, 2), dtype=windows_a.dtype)
            
        return filtered_a, windows_b
    
    else:  # mode == 'remove_b_keep_a'
        # Remove overlapping windows from set B
        non_overlapping_b = []
        
        for window_b in windows_b:
            overlaps = False
            
            for window_a in windows_a:
                if _check_overlap(window_b, window_a):
                    overlaps = True
                    break
            
            if not overlaps:
                non_overlapping_b.append(window_b)
        
        # Convert list back to numpy array
        if non_overlapping_b:
            filtered_b = np.array(non_overlapping_b)
        else:
            filtered_b = np.empty((0, 2, 2), dtype=windows_b.dtype)
            
        return windows_a, filtered_b

def _select_points_in_windows_vectorized(masked_image, search_windows, weighted=False):
    # Get all white points (nonzero pixels) from the binary image
    white_points = np.column_stack(np.nonzero(masked_image))
    
    # Initialize an empty array to collect all points in windows
    all_points = []
    
    # Process each window
    for window in search_windows:
        if weighted:
            top_left, bottom_right, _ = window
        else:
            top_left, bottom_right = window
            
        min_col, min_row = top_left
        max_col, max_row = bottom_right
        
        # Create mask for points within this window
        mask = ((white_points[:, 0] >= min_row) & 
                (white_points[:, 0] <= max_row) & 
                (white_points[:, 1] >= min_col) & 
                (white_points[:, 1] <= max_col))
        
        # Add points from this window
        points_in_this_window = white_points[mask]
        if len(points_in_this_window) > 0:
            all_points.append(points_in_this_window)
    
    # Combine all points and remove duplicates if needed
    if all_points:
        all_points = np.unique(np.vstack(all_points), axis=0)
        return all_points
    else:
        return np.array([])

def _select_points_and_fit_curve(masked_image, search_windows, weighted=False, degree=2):
    all_points = _select_points_in_windows_vectorized(masked_image, search_windows, weighted)

    if len(all_points) > 0:
        points_in_windows = np.unique(np.vstack(all_points), axis=0)
        
        # Extract y and x coordinates for the polynomial fit
        # Note: y is typically the row (first coordinate) and x is the column (second coordinate)
        lefty = points_in_windows[:, 0]  # rows
        leftx = points_in_windows[:, 1]  # columns
        
        # Fit polynomial
        if len(lefty) >= degree + 1:  # Need at least degree+1 points to fit a polynomial of degree
            coefficients = np.polyfit(lefty, leftx, degree)
            return points_in_windows, coefficients
        else:
            return points_in_windows, None
    else:
        return np.array([]), None


# Copy from https://github.com/laavanyebahl/Advance-Lane-Detection-and-Keeping-
# Modified using claude.ai to work with newer NumPy versions
def _sliding_window_polyfit(image):
    # Take a histogram of the bottom half of the image
    histogram = np.sum(image[image.shape[0]//2:,:], axis=0)
    # Find the peak of the left and right halves of the histogram
    # These will be the starting point for the left and right lines
    midpoint = int(histogram.shape[0]//2)
    # Previously the left/right base was the max of the left/right half of the histogram
    # FIXME!! NOT SUITABLE FOR OUR  CONDITION
    leftx_base = np.argmax(histogram[0:midpoint])
    rightx_base = np.argmax(histogram[midpoint:]) + midpoint
    
    # Choose the number of sliding windows
    nwindows = 10
    # Set height of windows
    window_height = int(image.shape[0]/nwindows)
    # Identify the x and y positions of all nonzero pixels in the image
    nonzero = image.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])
    # Current positions to be updated for each window
    leftx_current = leftx_base
    rightx_current = rightx_base
    # Set the width of the windows +/- margin
    margin = 30
    # Set minimum number of pixels found to recenter window
    minpix = 60
    # Create empty lists to receive left and right lane pixel indices
    left_lane_inds = []
    right_lane_inds = []
    # Rectangle data for visualization
    rectangle_data = []

    # Step through the windows one by one
    for window in range(nwindows):
        # Identify window boundaries in x and y (and right and left)
        win_y_low = image.shape[0] - (window+1)*window_height
        win_y_high = image.shape[0] - window*window_height
        win_xleft_low = leftx_current - margin
        win_xleft_high = leftx_current + margin
        win_xright_low = rightx_current - margin
        win_xright_high = rightx_current + margin
        rectangle_data.append((win_y_low, win_y_high, win_xleft_low, win_xleft_high, win_xright_low, win_xright_high))
        # Identify the nonzero pixels in x and y within the window
        good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
        good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]
        # Append these indices to the lists
        left_lane_inds.append(good_left_inds)
        right_lane_inds.append(good_right_inds)
        # If you found > minpix pixels, recenter next window on their mean position
        if len(good_left_inds) > minpix:
            leftx_current = int(np.mean(nonzerox[good_left_inds]))
        if len(good_right_inds) > minpix:        
            rightx_current = int(np.mean(nonzerox[good_right_inds]))

    # Concatenate the arrays of indices
    left_lane_inds = np.concatenate(left_lane_inds)
    right_lane_inds = np.concatenate(right_lane_inds)

    # Extract left and right line pixel positions
    leftx = nonzerox[left_lane_inds]
    lefty = nonzeroy[left_lane_inds] 
    rightx = nonzerox[right_lane_inds]
    righty = nonzeroy[right_lane_inds] 

    left_fit, right_fit = (None, None)
    # Fit a second order polynomial to each
    if len(leftx) != 0:
        left_fit = np.polyfit(lefty, leftx, 2)
    if len(rightx) != 0:
        right_fit = np.polyfit(righty, rightx, 2)
    
    display_data = (rectangle_data, histogram)
    
    return left_fit, right_fit, left_lane_inds, right_lane_inds, display_data

def _draw_lane(und_image, warped_img, lane_or_pts, Minv):
    """
    Draws a lane or a list of points onto the original image using the inverse perspective matrix.
    
    Args:
        und_image: Original undistorted image.
        warped_img: Warped image (used for sizing and warping back).
        lane_or_pts: Either a LaneLine object with an .evaluate(y) method or a numpy array of points.
        Minv: Inverse perspective transform matrix.
        
    Returns:
        Image with lane or points drawn on it.
    """
    image_copy = np.copy(und_image)
    if lane_or_pts is None:
        return image_copy

    # Create an image to draw the lines on
    warp_zero = np.zeros_like(warped_img).astype(np.uint8)

    # Handle grayscale vs. color
    if len(warped_img.shape) == 2:
        color_warp = np.dstack((warp_zero, warp_zero, warp_zero))
    else:
        color_warp = np.zeros_like(warped_img).astype(np.uint8)

    h, w = warped_img.shape[:2]

    # Determine how to process lane_or_pts
    if isinstance(lane_or_pts, LaneLine) and hasattr(lane_or_pts, "evaluate"):
        # Assume it's a LaneLine object
        ploty = np.linspace(0, h - 1, num=h)
        fitx = lane_or_pts.evaluate(ploty)
        pts = np.array([np.transpose(np.vstack([fitx, ploty]))], dtype=np.int32)
        color = (255, 0, 0)
        thickness = 10
    else:
        # Assume it's an ndarray of points
        pts = np.array(lane_or_pts, dtype=np.int32)
        if pts.ndim == 2 and pts.shape[1] == 2:
            pts = np.stack([pts[:, 1], pts[:, 0]], axis=1)  # Swap x and y
            pts = pts.reshape((-1, 1, 2))
        pts = [pts]
        color = (0, 0, 255)
        thickness = 5

    # Draw the lane or points
    cv2.polylines(color_warp, pts, isClosed=False, color=color, thickness=thickness)

    # Warp the blank back to original image space
    newwarp = cv2.warpPerspective(color_warp, Minv, (w, h))

    # Overlay the result
    result = cv2.addWeighted(image_copy, 1, newwarp, 0.3, 0)
    return result

class LaneDetector:
    def __init__(self, calibration_conf_path, warp_conf_path):
        self.camera_matrix, self.dist_coeffs = load_calibration_data(calibration_conf_path)
        self.src_point, self.dst_point = load_warp_points(warp_conf_path)
        self.initialized = False
        self.previous_left_line = None
        self.previous_right_line = None

    def process_frame(self, frame):
        """
        Process the input frame to detect lanes and return offset and heading angle.
        Args:
            frame: The input frame from the camera.
        Returns:
            offset (pixel) and heading angle (degree).
        """
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        img, _ = undistort_image(frame, self.camera_matrix, self.dist_coeffs)
        warped, M, Minv = warp_image_undistorted(img, self.src_point, self.dst_point)

        masked, edges = _color_space_transform(warped)

        nonzero = masked.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        out_img = np.uint8(np.dstack((masked, masked, masked))*255)
        
        if self.initialized:
            # For the polyfit we can actually assign larger weight to
            # near (a.k.a bottom region) points and assign lighter weight to far points in order to eliminate noise
            # Currently not implemented so keep it False
            weighted=False
            
            left_search_windows = self.previous_left_line.sliding_window_loc(masked, margin=30, weighted=weighted)
            right_search_windows = self.previous_right_line.sliding_window_loc(masked, margin=30, weighted=weighted)

            # if len(left_search_windows) < len(right_search_windows) and previous_right_line.is_turning_left(masked.shape[0]):
            if len(left_search_windows) != len(right_search_windows) and self.previous_right_line.is_turning_left(masked.shape[0]):
                left_search_windows, right_search_windows = _remove_overlapping_windows(left_search_windows, right_search_windows, mode='remove_a_keep_b')
            elif len(left_search_windows) != len(right_search_windows) and self.previous_left_line.is_turning_right(masked.shape[0]):
                left_search_windows, right_search_windows = _remove_overlapping_windows(left_search_windows, right_search_windows, mode='remove_b_keep_a')

            # Draw the windows after overlap removal, for better visualization and debug
            for window in left_search_windows:
                (top_left_x, top_left_y), (bottom_right_x, bottom_right_y) = window
                cv2.rectangle(out_img,(top_left_x,top_left_y),(bottom_right_x,bottom_right_y),(0,255,0), 2) 

            for window in right_search_windows:
                (top_left_x, top_left_y), (bottom_right_x, bottom_right_y) = window
                cv2.rectangle(out_img,(top_left_x,top_left_y),(bottom_right_x,bottom_right_y),(255,0,0), 2)

            # Try with polyfit first
            points_in_left, left_fit_cos = _select_points_and_fit_curve(masked, left_search_windows, weighted)
            points_in_rigth, right_fit_cos = _select_points_and_fit_curve(masked, right_search_windows, weighted)

            if left_fit_cos is not None and right_fit_cos is not None:
                # All two line are visiable
                #print('Condition: 1')
                left_fit = LaneLine('left', 'measured', (left_fit_cos[0], left_fit_cos[1], left_fit_cos[2]))
                right_fit = LaneLine('right', 'measured', (right_fit_cos[0], right_fit_cos[1], right_fit_cos[2]))
                if (len(left_search_windows) <= 7):
                    left_fit = predict_other_lane_line(right_fit, masked)
                    print('Using predict only: Left', left_fit);
                elif (len(right_search_windows) <= 7):
                    right_fit = predict_other_lane_line(left_fit, masked)
                    print('Using predict only: Right', right_fit);
            elif right_fit_cos is not None and left_fit_cos is None:
                # Only right line is visible
                print('Condition: 2')
                right_fit = LaneLine('right', 'measured', (right_fit_cos[0], right_fit_cos[1], right_fit_cos[2]))
                left_fit = predict_other_lane_line(right_fit, masked)
            elif left_fit_cos is not None and right_fit_cos is None:
                # Only left line is visible
                print('Condition: 3')
                left_fit = LaneLine('left', 'measured', (left_fit_cos[0], left_fit_cos[1], left_fit_cos[2]))
                right_fit = predict_other_lane_line(left_fit, masked)
            else:
                # Two lines are both invisible
                print('Condition: 4')
                # Fall back to sliding_window_polyfit
                self.initialized = False
        else:
            # First time, initialized with sliding_window_method
            left_fit_cos, right_fit_cos, left_lane_inds, right_lane_inds, _ = _sliding_window_polyfit(masked)
            if left_fit_cos is not None and right_fit_cos is not None:
                # Sliding window must ensure two lines are visible
                left_fit = LaneLine('left', 'measured', (left_fit_cos[0], left_fit_cos[1], left_fit_cos[2]))
                right_fit = LaneLine('right', 'measured', (right_fit_cos[0], right_fit_cos[1], right_fit_cos[2]))
                self.initialized = True
            
        if self.initialized:
            self.previous_left_line = left_fit
            self.previous_right_line = right_fit
            middle_line = middle_lane_fit(left_fit, right_fit)
            offset = center_offset(left_fit, right_fit, masked, car_camera_offset=-5)
            heading = middle_line.heading_angle
            heading_deg = heading / np.pi * 180

            # Draw polyfit lines for better visualization and debugging
            ploty = np.linspace(0, masked.shape[0]-1, masked.shape[0])
            for idx, line in enumerate([left_fit, middle_line, right_fit]):
                plotx = line.evaluate(ploty)
                # Convert floating point values to integers for indexing
                plotx = np.round(plotx).astype(np.int64)
                
                # Filter out any points that might be outside the image bounds
                valid_points = (plotx >= 0) & (plotx < masked.shape[1])
                ploty_valid = ploty[valid_points].astype(np.int64)
                plotx_valid = plotx[valid_points]
                
                # Draw the line
                out_img[ploty_valid, plotx_valid] = [100, 200, 255] if idx == 1 else [255, 100, 200]

            drawed = _draw_lane(img, warped, middle_line, Minv)
        else:
            drawed = _draw_lane(img, warped, None, Minv)
        
        combin = np.hstack((warped, out_img, drawed))
        cv2.imshow("Combined", combin)

        return offset, heading_deg


    def draw_lanes(self, frame):
        # Placeholder for drawing detected lanes on the frame
        pass