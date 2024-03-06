# assuming the placement of beacon is in the following order
# r1 ------- r2
# |           |
# |           |
# r3 --------- 
# and the coordinate is 
# ^ positive y
# |
# |
#  ---> positive x
# the middle of berm signifies origin
# r1, r2, r3, signifies the distance detected by each beacon
def trilaterate_based_on_berm(berm_w, berm_h, r1, r2, r3):
  half_w = berm_w / 2
  half_h = berm_h / 2
  r1_coord = (- half_w, half_h)
  r2_coord = (half_w, half_h)
  r3_coord = (- half_w, - half_h)
  return trilaterate(r1_coord, r1, r2_coord, r2, r3_coord, r3)

def trilaterate(r1_coord, r1, r2_coord, r2, r3_coord, r3):
  x1, y1 = r1_coord
  x2, y2 = r2_coord
  x3, y3 = r3_coord
  A = 2*x2 - 2*x1
  B = 2*y2 - 2*y1
  C = r1**2 - r2**2 - x1**2 + x2**2 - y1**2 + y2**2
  D = 2*x3 - 2*x2
  E = 2*y3 - 2*y2
  F = r2**2 - r3**2 - x2**2 + x3**2 - y2**2 + y3**2
  x = (C*E - F*B) / (E*A - B*D)
  y = (C*D - A*F) / (B*D - A*E)
  return x,y
