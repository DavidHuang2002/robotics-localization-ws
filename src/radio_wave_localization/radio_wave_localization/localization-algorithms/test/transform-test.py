
import sys
import os


# set up for test - allow for import from parent directory
# so that I can import modules from the parent directory (like calc_orientation.py)
def add_parent_dir_to_path():
    """Add the parent directory of the script to sys.path."""
    script_dir = os.path.dirname(os.path.abspath(__file__))  # Directory of the script
    parent_dir = os.path.dirname(script_dir)  # Parent directory
    sys.path.append(parent_dir)

add_parent_dir_to_path()

from coord_frame_transformation import coord_frame_transformation
import numpy as np

# Test for Coordinate Transformation
def test_coordinate_transformation():
    coord_from_home = (4, 5, 0)  # Translation
    angle_from_home = np.pi / 4  # Rotation (45 degrees)

    # Original point
    original_point = np.array([1, 2, 0, 1]).reshape(-1, 1)  # Add homogeneous coordinate for matrix multiplication

    # Calculate transformation
    rotation_matrix, translation_matrix = coord_frame_transformation(coord_from_home, angle_from_home)
    transformed_point = np.dot(translation_matrix, np.dot(rotation_matrix, original_point))

    print(f"Original Point: {original_point.T}, Transformed Point: {transformed_point.T}")

if __name__ == "__main__":
    test_coordinate_transformation()