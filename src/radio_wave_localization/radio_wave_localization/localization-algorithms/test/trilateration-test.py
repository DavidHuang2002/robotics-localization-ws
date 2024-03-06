
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


import numpy as np
from trilateriation import trilaterate_based_on_berm
from utils import float_equal

# Helper function to calculate distance between two points
def calculate_distance(p1, p2):
    return np.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

# Test for Trilateration
def test_trilateration():
    berm_w = 10  # Width of the berm
    berm_h = 10  # Height of the berm
    # Simulated point (expected result)
    for i in range(100):
        for j in range(100):
            simulated_point = (i, j)

            # Calculate distances from the simulated point to the beacons
            r1 = calculate_distance(simulated_point, (-berm_w / 2, berm_h / 2))
            r2 = calculate_distance(simulated_point, (berm_w / 2, berm_h / 2))
            r3 = calculate_distance(simulated_point, (-berm_w / 2, -berm_h / 2))

            # Calculate trilateration
            calculated_point = trilaterate_based_on_berm(berm_w, berm_h, r1, r2, r3)
            print(f"Expected Point: {simulated_point}, Calculated Point: {calculated_point}")
            assert float_equal(simulated_point[0], calculated_point[0]) and float_equal(simulated_point[1], calculated_point[1])


if __name__ == "__main__":
    test_trilateration()
