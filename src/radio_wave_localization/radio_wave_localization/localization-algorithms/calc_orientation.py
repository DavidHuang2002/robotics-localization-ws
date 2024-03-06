import numpy as np
import math
# assuming diagonal placement of the receiver.
# 
# head ------- 
# |           |
# |           |
#   --------- tail
# return the angle of the robot in radian
# maybe not use NDArray

def normalize_angle(angle):
    return np.mod(angle, 2 * np.pi)

def calc_orientation(head_pos, tail_pos, robot_width, robot_length):
    # diagonal angle offset: the diff. in angle between angle of diagonal of robot
    # calculated from the actual orientation of the robot
    dia_angle_offset = np.arctan(robot_width / robot_length)
    
    # vectorize
    head_vec = np.array(head_pos)
    tail_vec = np.array(tail_pos)
    # vector from tail to head
    t_h_vec = head_vec - tail_vec
    # note that for arctan2 the position of y and x are swapped
    # return the angle from x axis in radian
    diagonal_orientation = np.arctan2(t_h_vec[1], t_h_vec[0])

    return normalize_angle(diagonal_orientation - dia_angle_offset)


