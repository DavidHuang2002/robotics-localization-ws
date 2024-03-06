# coord_from_home - [x, y, z]
# angle_from_home - a float - radian format
# def coord_frame_tranformation(coord_from_home, angle_from_home):
#     pass

# def find_translation_matrix(coord_from_home):
#     pass


# def find_rotation_matrix(angle_from_home):
#     pass

import numpy as np

def find_translation_matrix(coord_from_home):
    x, y, z = coord_from_home
    return np.array([[1, 0, 0, x],
                     [0, 1, 0, y],
                     [0, 0, 1, z],
                     [0, 0, 0, 1]])

def find_rotation_matrix(angle_from_home):
    cos_theta = np.cos(angle_from_home)
    sin_theta = np.sin(angle_from_home)
    return np.array([[cos_theta, -sin_theta, 0, 0],
                     [sin_theta, cos_theta,  0, 0],
                     [0,         0,          1, 0],
                     [0,         0,          0, 1]])

def coord_frame_transformation(coord_from_home, angle_from_home):
    translation_matrix = find_translation_matrix(coord_from_home)
    rotation_matrix = find_rotation_matrix(angle_from_home)

    # The combined transformation matrix
    return (rotation_matrix, translation_matrix)

# # Example usage
# coord_from_home = [1, 2, 3]
# angle_from_home = np.pi / 4  # 45 degrees in radians

# r, t = coord_frame_transformation(coord_from_home, angle_from_home)

# print(t* coord_from_home)

# # testing
# # given coordinate in robot coord. turn into home coord.

