from trilateriation import trilaterate_based_on_berm
from calc_orientation import calc_orientation
from coord_frame_transformation import coord_frame_transformation
import numpy as np

def radio_wave_find_transform(head_radio_distances, tail_radio_distances, berm_length, berm_width, robot_width, robot_length):
    # trilaterate head
    head_poition = trilaterate_based_on_berm(berm_length, berm_width, *head_radio_distances)

    # trilaterate tail
    tail_position = trilaterate_based_on_berm(berm_length, berm_width, *tail_radio_distances)

    robot_position = (head_poition + tail_position) / 2

    robot_angle = calc_orientation(head_poition, tail_position, robot_width, robot_length)

    rotation_matrix, translation_matrix = coord_frame_transformation(robot_position, robot_angle)

    return np.dot(rotation_matrix, translation_matrix)