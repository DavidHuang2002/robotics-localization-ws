from trilateriation import trilaterate_based_on_berm
from calc_orientation import calc_orientation
from coord_frame_transformation import coord_frame_transformation

def radio_wave_find_transform(head_radio_distances, tail_radio_distances, berm_w, berm_h, robot_width, robot_length):
    # trilaterate head
    head_poition = trilaterate_based_on_berm(berm_w, berm_h, *head_radio_distances)

    # trilaterate tail
    tail_position = trilaterate_based_on_berm(berm_w, berm_h, *tail_radio_distances)

    robot_position = (head_poition + tail_position) / 2

    robot_angle = calc_orientation(head_poition, tail_position, robot_width, robot_length)

    return coord_frame_transformation(robot_position, robot_angle)
