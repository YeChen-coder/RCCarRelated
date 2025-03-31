import numpy as np

class LaneLine:
    """
    A class representing a lane line in a driving scenario.
    
    Attributes:
        position (str): Position of the lane line ('left', 'middle' or 'right')
        type (str): Type of lane line ('measured' or 'calculated')
        coefficients (tuple): Coefficients of the polynomial fit (a, b, c) for y = ax^2 + bx + c
    """
    
    def __init__(self, position, line_type, coefficients):
        """
        Initialize a LaneLine object.
        
        Args:
            position (str): Position of the lane line ('left', 'middle' or 'right')
            line_type (str): Type of lane line ('measured' or 'calculated')
            coefficients (tuple): Coefficients of the polynomial fit (a, b, c) for y = ax^2 + bx + c
        
        Raises:
            ValueError: If position is not 'left', 'middle' or 'right'
            ValueError: If line_type is not 'measured' or 'calculated'
            ValueError: If coefficients is not a tuple of length 3
        """
        # Validate position
        if position not in ['left', 'middle', 'right']:
            raise ValueError("Position must be 'left', 'middle' or 'right'")
        self.position = position
        
        # Validate type
        if line_type not in ['measured', 'calculated']:
            raise ValueError("Type must be 'measured' or 'calculated'")
        self.type = line_type
        
        # Validate coefficients
        if not isinstance(coefficients, tuple) or len(coefficients) != 3:
            raise ValueError("Coefficients must be a tuple of length 3 (a, b, c)")
        self.coefficients = coefficients
        
    @property
    def a(self):
        """Quadratic coefficient of the polynomial fit."""
        return self.coefficients[0]
    
    @property
    def b(self):
        """Linear coefficient of the polynomial fit."""
        return self.coefficients[1]
    
    @property
    def c(self):
        """Constant coefficient of the polynomial fit."""
        return self.coefficients[2]
    
    def evaluate(self, x):
        """
        Evaluate the polynomial at a given x value.
        
        Args:
            x (float or numpy.array): The x-coordinate(s) to evaluate
            
        Returns:
            float or numpy.array: The y-coordinate(s) of the polynomial at x
        """
        a, b, c = self.coefficients
        return a * x**2 + b * x + c

    def compute_curvature(self, x0):
        """
        Calculate the radius (pixel) of curvature at a given y value.
        
        Args:
            y (float): The y-coordinate to evaluate
        
        Returns:
            float: The radius of curvature at the given y value
        """
        a = self.a
        b = self.b
        c = self.c

        # Compute first and second derivatives directly
        y_prime_val = 2 * a * x0 + b  # First derivative
        y_double_prime_val = 2 * a  # Second derivative (constant for a quadratic function)
        
        # Compute y value at x0
        y_val = a * x0**2 + b * x0 + c
        
        # Compute curvature
        curvature = abs(y_double_prime_val) / (1 + y_prime_val**2)**(3/2)
        
        # Compute radius of curvature
        radius_curvature = 1 / curvature if curvature != 0 else float('inf')
        
        # Compute center of curvature
        X_c = x0 - (y_prime_val * (1 + y_prime_val**2)) / y_double_prime_val
        Y_c = y_val + (1 + y_prime_val**2) / y_double_prime_val

        return curvature, radius_curvature, (X_c, Y_c)

    def is_turing_left_rad_center(self, x0, rad, center, threshold_rad=2000):
        X_c, Y_c = center
        return Y_c < self.evaluate(x0) and rad < threshold_rad

    def is_turing_right_rad_center(self, x0, rad, center, threshold_rad=2000):
        X_c, Y_c = center
        return Y_c > self.evaluate(x0) and rad < threshold_rad
    
    @property
    def heading_angle(self):
        """
        Calculate the heading angle of the lane line.
        
        Returns:
            float: The heading angle in radians
        """
        return np.arctan2(2 * self.a, 1)

    def is_turning_left(self, x0, threshold_rad=2000):
        """
        Determine if the lane line is turning left at a given y value.
        
        Args:
            x0 (float): The x-coordinate to evaluate
        
        Returns:
            bool: True if the lane line is turning left, False otherwise
        """
        _, rad, (X_c, Y_c) = self.compute_curvature(x0)

        return self.is_turing_left_rad_center(x0, rad, (X_c, Y_c), threshold_rad)

    def is_turning_right(self, x0, threshold_rad=2000):
        """
        Determine if the lane line is turning right at a given y value.
        
        Args:
            x0 (float): The x-coordinate to evaluate
        
        Returns:
            bool: True if the lane line is turning right, False otherwise
        """
        _, rad, (X_c, Y_c) = self.compute_curvature(x0)

        return self.is_turing_right_rad_center(x0, rad, (X_c, Y_c), threshold_rad)

    def sliding_window_loc(self, image, n_windows=10, margin=50, weighted=False):
        """
        Return the sliding windows locations for next frame prediction based on current fit.
        """
        image_height, image_width = image.shape[:2]
        window_height = int(image_height / n_windows)

        windows = []

        plot_y = np.linspace(image_height-1, 0, num=n_windows, endpoint=False, dtype=int)
        for idx, y_base in enumerate(plot_y):
            x_base = self.evaluate(y_base)
            x_left = max(0, min(int(x_base - margin), image_width - 1))
            x_right = max(0, min(int(x_base + margin), image_width - 1))
            if (x_right - x_left) > 0:
                # (x_left, int(y_base) - window_height, x_right, int(y_base)
                top_left = (x_left, int(y_base) - window_height)
                bottom_right = (x_right, int(y_base))
                if weighted:
                    windows.append((top_left, bottom_right, idx))
                else:
                    windows.append((top_left, bottom_right))
        
        return windows

    def __str__(self):
        """String representation of the lane line."""
        return f"{self.position.capitalize()} lane line ({self.type}): y = {self.a}x^2 + {self.b}x + {self.c}"
    
    def __repr__(self):
        """Official string representation of the lane line."""
        return f"LaneLine(position='{self.position}', type='{self.type}', coefficients={self.coefficients})"

def center_offset(lane_line_left, lane_line_right, image, car_camera_offset=0):
    """
    Calculate the center offset of the lane lines from the center of the image.
    
    Args:
        lane_line_left (LaneLine): The left lane line
        lane_line_right (LaneLine): The right lane line
        image (numpy.array): The image containing the lane lines
        car_camera_offset (float): The offset of the camera from the center of the car (default is 0)
        
    Returns:
        float: The center offset in pixels
    """

    # Get the image dimensions
    image_height, image_width = image.shape[:2]

    # Calculate the x-coordinates of the left and right lane lines at the bottom of the image
    y_bottom = image_height
    x_left = lane_line_left.evaluate(y_bottom)
    x_right = lane_line_right.evaluate(y_bottom)

    car_position = image_width / 2 + car_camera_offset
    
    # Calculate the center of the lane lines
    center_lane_x = (x_left + x_right) / 2
    
    # Calculate the center offset from the image center
    offset = car_position - center_lane_x # positive value means the car is to the right of the lane center
    
    return offset


def middle_lane_fit_weighted(lane_line_left, lane_line_right, weight=(0.5, 0.5)):
    return LaneLine(
        position='middle',
        line_type='calculated',
        coefficients=(
            (lane_line_left.a * weight[0] + lane_line_right.a * weight[1]) / (weight[0] + weight[1]),
            (lane_line_left.b * weight[0] + lane_line_right.b * weight[1]) / (weight[0] + weight[1]),
            (lane_line_left.c * weight[0] + lane_line_right.c * weight[1]) / (weight[0] + weight[1]),
        )
    )

def middle_lane_fit(lane_line_left, lane_line_right):
    return middle_lane_fit_weighted(lane_line_left, lane_line_right, weight=(1, 1))


def predict_other_lane_line_deprecated(lane_line, image, offset_rad_pixels=117):
    h, w = image.shape[:2]
    _, rad, center = lane_line.compute_curvature(h-1)
    X_c, Y_c = center

    if lane_line.position == 'left' and lane_line.is_turing_left_rad_center(h-1, rad, center):
        x0 = np.float64(0)
        y0 = Y_c + np.sqrt(np.float64(rad ** 2 - (x0 - X_c) ** 2))
        x1 = np.float64(h - 1)
        y1 = Y_c + np.sqrt(np.float64(rad ** 2 - (x1 - X_c) ** 2))
        thetas = []

        for (x, y) in zip([x0, y0], [x1, y1]):
            dx = np.float64(x - X_c)
            dy = np.float64(y - Y_c)
            if dx == 0:
                theta = np.pi / 2 if dy > 0 else 3 * np.pi / 2
            else:
                theta = np.arctan2(dy, dx)
            thetas.append(theta)

        rad2 = np.float64(rad + offset_rad_pixels)
        thetas = np.array(thetas, dtype=np.float64)
        new_Xs = X_c + rad2 * np.cos(thetas)
        plotx = np.linspace(new_Xs[0], new_Xs[1], num=10)

        # Make sure all values in computation are numpy arrays
        X_c_array = np.full_like(plotx, X_c)
        rad2_array = np.full_like(plotx, rad2)
        Y_c_array = np.full_like(plotx, Y_c)

        # Calculate using broadcasted arrays
        ploty = Y_c_array + np.sqrt(rad2_array ** 2 - (plotx - X_c_array) ** 2)
        
        new_coeffs = np.polyfit(plotx, ploty, 2)
        return LaneLine(
            position='left',
            line_type='calculated',
            coefficients=(
                new_coeffs[0],
                new_coeffs[1],
                new_coeffs[2],
            )
        )
    elif lane_line.position == 'right' and lane_line.is_turing_left_rad_center(h-1, rad, center):
        x0 = np.float64(0)
        y0 = Y_c + np.sqrt(np.float64(rad ** 2 - (x0 - X_c) ** 2))
        x1 = np.float64(h - 1)
        y1 = Y_c + np.sqrt(np.float64(rad ** 2 - (x1 - X_c) ** 2))
        thetas = []

        for (x, y) in zip([x0, y0], [x1, y1]):
            dx = np.float64(x - X_c)
            dy = np.float64(y - Y_c)
            if dx == 0:
                theta = np.pi / 2 if dy > 0 else 3 * np.pi / 2
            else:
                theta = np.arctan2(dy, dx)
            thetas.append(theta)

        rad2 = np.float64(rad - offset_rad_pixels)
        thetas = np.array(thetas, dtype=np.float64)
        new_Xs = X_c + rad2 * np.cos(thetas)
        plotx = np.linspace(new_Xs[0], new_Xs[1], num=10)

        # Make sure all values in computation are numpy arrays
        X_c_array = np.full_like(plotx, X_c)
        rad2_array = np.full_like(plotx, rad2)
        Y_c_array = np.full_like(plotx, Y_c)

        # Calculate using broadcasted arrays
        ploty = Y_c_array + np.sqrt(rad2_array ** 2 - (plotx - X_c_array) ** 2)
        
        new_coeffs = np.polyfit(plotx, ploty, 2)
        return LaneLine(
            position='left',
            line_type='calculated',
            coefficients=(
                new_coeffs[0],
                new_coeffs[1],
                new_coeffs[2],
            )
        )
    elif lane_line.position == 'left' and lane_line.is_turing_right_rad_center(h-1, rad, center):
        x0 = np.float64(0)
        y0 = Y_c - np.sqrt(np.float64(rad ** 2 - (x0 - X_c) ** 2))
        x1 = np.float64(h - 1)
        y1 = Y_c - np.sqrt(np.float64(rad ** 2 - (x1 - X_c) ** 2))
        thetas = []

        for (x, y) in zip([x0, y0], [x1, y1]):
            dx = np.float64(x - X_c)
            dy = np.float64(y - Y_c)
            if dx == 0:
                theta = np.pi / 2 if dy > 0 else 3 * np.pi / 2
            else:
                theta = np.arctan2(dy, dx)
            thetas.append(theta)

        rad2 = np.float64(rad - offset_rad_pixels)
        thetas = np.array(thetas, dtype=np.float64)
        new_Xs = X_c + rad2 * np.cos(thetas)
        plotx = np.linspace(new_Xs[0], new_Xs[1], num=10)

        # Make sure all values in computation are numpy arrays
        X_c_array = np.full_like(plotx, X_c)
        rad2_array = np.full_like(plotx, rad2)
        Y_c_array = np.full_like(plotx, Y_c)

        # Calculate using broadcasted arrays
        ploty = Y_c_array - np.sqrt(rad2_array ** 2 - (plotx - X_c_array) ** 2)
        
        new_coeffs = np.polyfit(plotx, ploty, 2)
        return LaneLine(
            position='left',
            line_type='calculated',
            coefficients=(
                new_coeffs[0],
                new_coeffs[1],
                new_coeffs[2],
            )
        )
    elif lane_line.position == 'right' and lane_line.is_turing_right_rad_center(h-1, rad, center):
        x0 = np.float64(0)
        y0 = Y_c - np.sqrt(np.float64(rad ** 2 - (x0 - X_c) ** 2))
        x1 = np.float64(h - 1)
        y1 = Y_c - np.sqrt(np.float64(rad ** 2 - (x1 - X_c) ** 2))
        thetas = []

        for (x, y) in zip([x0, y0], [x1, y1]):
            dx = np.float64(x - X_c)
            dy = np.float64(y - Y_c)
            if dx == 0:
                theta = np.pi / 2 if dy > 0 else 3 * np.pi / 2
            else:
                theta = np.arctan2(dy, dx)
            thetas.append(theta)

        rad2 = np.float64(rad + offset_rad_pixels)
        thetas = np.array(thetas, dtype=np.float64)
        new_Xs = X_c + rad2 * np.cos(thetas)
        plotx = np.linspace(new_Xs[0], new_Xs[1], num=10)

        # Make sure all values in computation are numpy arrays
        X_c_array = np.full_like(plotx, X_c)
        rad2_array = np.full_like(plotx, rad2)
        Y_c_array = np.full_like(plotx, Y_c)

        # Calculate using broadcasted arrays
        ploty = Y_c_array - np.sqrt(rad2_array ** 2 - (plotx - X_c_array) ** 2)
        
        new_coeffs = np.polyfit(plotx, ploty, 2)
        return LaneLine(
            position='left',
            line_type='calculated',
            coefficients=(
                new_coeffs[0],
                new_coeffs[1],
                new_coeffs[2],
            )
        )
    elif lane_line.position == 'left' and not lane_line.is_turing_left_rad_center(h-1, rad, center) and not lane_line.is_turing_right_rad_center(h-1, rad, center):
        return LaneLine(
            position='right',
            line_type='calculated',
            coefficients=(
                lane_line.a,
                lane_line.b,
                lane_line.c + offset_rad_pixels,
            )
        )
    elif lane_line.position == 'right' and not lane_line.is_turing_left_rad_center(h-1, rad, center) and not lane_line.is_turing_right_rad_center(h-1, rad, center):
        return LaneLine(
            position='left',
            line_type='calculated',
            coefficients=(
                lane_line.a,
                lane_line.b,
                lane_line.c - offset_rad_pixels,
            )
        )
    else:
        return None

def predict_other_lane_line(lane_line, image, offset_rad_pixels=117):
    """
    Predicts the other lane line based on a detected lane line.
    
    Args:
        lane_line: The detected lane line
        image: The source image
        offset_rad_pixels: The offset in pixels between lane lines
        
    Returns:
        A new LaneLine object representing the predicted other lane
    """
    h, w = image.shape[:2]
    _, rad, center = lane_line.compute_curvature(h-1)
    X_c, Y_c = center
    
    # Handle straight lines (or lines with very small curvature)
    if not lane_line.is_turing_left_rad_center(h-1, rad, center) and not lane_line.is_turing_right_rad_center(h-1, rad, center):
        # For a straight line, just offset the c coefficient (y-intercept)
        offset = -offset_rad_pixels if lane_line.position == 'right' else offset_rad_pixels
        return LaneLine(
            position='left' if lane_line.position == 'right' else 'right',
            line_type='calculated',
            coefficients=(lane_line.a, lane_line.b, lane_line.c + offset)
        )
    
    # Handle curved lines
    turning_left = lane_line.is_turing_left_rad_center(h-1, rad, center)
    
    y0 = 0 if turning_left else w - 1
    # Determine radius adjustment based on lane position and turning direction
    if (lane_line.position == 'left' and turning_left) or (lane_line.position == 'right' and not turning_left):
        rad2 = np.float64(rad + offset_rad_pixels)
        sign = 1  # For determining square root sign (+ or -)
    else:  # (lane_line.position == 'right' and turning_left) or (lane_line.position == 'left' and not turning_left)
        rad2 = np.float64(rad - offset_rad_pixels)
        sign = 1 if turning_left else -1
        
    # Calculate endpoints of the curve
    # If the circle arc will not cross x=0
    x0, x1 = np.float64(X_c - np.sqrt(np.float64(rad ** 2 - (y0 - Y_c) ** 2))), np.float64(h - 1)
    
    # Calculate theta values for the curve points
    thetas = []
    for x in [x0, x1]:
        # For turning left, y = Y_c + sqrt(...)
        # For turning right, y = Y_c - sqrt(...)
        y = Y_c + sign * np.sqrt(np.float64(rad ** 2 - (x - X_c) ** 2))
        
        # Calculate angle
        dx, dy = np.float64(x - X_c), np.float64(y - Y_c)
        if dx == 0:
            theta = np.pi / 2 if dy > 0 else 3 * np.pi / 2
        else:
            theta = np.arctan2(dy, dx)
        thetas.append(theta)
    
    # Convert to numpy array for calculations
    thetas = np.array(thetas, dtype=np.float64)
    
    # Calculate new x-coordinates with the adjusted radius
    new_Xs = X_c + rad2 * np.cos(thetas)
    plotx = np.linspace(new_Xs[0], new_Xs[1], num=10)
    
    # Create arrays for broadcasting
    X_c_array = np.full_like(plotx, X_c)
    rad2_array = np.full_like(plotx, rad2)
    Y_c_array = np.full_like(plotx, Y_c)
    
    # Calculate y-coordinates with the adjusted radius
    ploty = Y_c_array + sign * np.sqrt(rad2_array ** 2 - (plotx - X_c_array) ** 2)
    
    # Fit a quadratic polynomial to the points
    new_coeffs = np.polyfit(plotx, ploty, 2)
    
    return LaneLine(
        position='left',  # This seems to always be 'left' in the original code
        line_type='calculated',
        coefficients=(new_coeffs[0], new_coeffs[1], new_coeffs[2])
    )             

def inverse_transform_polyfit(lane_line, image):
    """
    The polynomial fit of the lane is re-calculated based with inversed x-y transformation.
    """
    # Get the image dimensions
    h, w = image.shape[:2]
    # Generate y values for the entire image height
    ploty = np.linspace(0, h-1, h)
    # Generate x values using the polynomial coefficients
    xvals = lane_line.evaluate(ploty)
    # Re-polyfit the x values to get the new coefficients
    new_coeffs = np.polyfit(xvals, ploty, 2)
    # Create a new LaneLine object with the transformed coefficients
    transformed_lane_line = LaneLine(
        position=lane_line.position,
        line_type=lane_line.type,
        coefficients=(new_coeffs[0], new_coeffs[1], new_coeffs[2])
    )
    return transformed_lane_line


if __name__ == "__main__":
    # Example usage
    lane_left = LaneLine('left', 'measured', (-1.49676934e-03,  1.04803588e+00, -1.13919888e+02))
    lane_right = LaneLine('right', 'measured', (-0.001956203891633444,  1.0222245584774046,  29.651299568618928))
    # lane_left = LaneLine('left', 'measured', (6.93358831e-05, -2.30505345e-02,  5.12289346e+01))
    # lane_right = LaneLine('right', 'measured', (-1.39013639e-04,  1.81408446e-02,  1.63495409e+02))

    print(lane_left)
    print(lane_right)

    # Calculate the middle lane line
    middle_lane = middle_lane_fit(lane_left, lane_right)
    print(middle_lane)

    # Example image (dummy array for demonstration purposes)
    image = np.zeros((232, 218, 3), dtype=np.uint8)

    _, left_rad, left_rad_center = lane_left.compute_curvature(image.shape[0])
    _, right_rad, right_rad_center = lane_right.compute_curvature(image.shape[0])
    print(left_rad)
    print(right_rad)

    print(left_rad_center)
    print(right_rad_center)

    print(lane_left.is_turning_left(image.shape[0]))
    print(lane_left.is_turning_right(image.shape[0]))

    # Calculate center offset
    offset = center_offset(lane_left, lane_right, image)
    print(f"Center offset: {offset} pixels")

    xm_per_pix = 0.36 /117
    ym_per_pix = 0.2 / 37
    print(f"Center offset in meters: {offset * xm_per_pix} m")

    predict_left = predict_other_lane_line(lane_right, image, offset_rad_pixels=117)
    print(predict_left)
