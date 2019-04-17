import matplotlib.pyplot as plt

# p1, q1 = [0, 0], [1, 0]
# p2, q2 = [0.5, 0.5], [0.5, -0.5]

# p1, q1 = [0, 0], [1, 0]
# p2, q2 = [0.5, 0.5], [0.8, 0.4]

# p1, q1 = [0.2, 1], [1, 1]
# p2, q2 = [0, 0], [1, 0]

# p1, q1 = [3.43, 3.91], [4.98, 4.08]
# p2, q2 = [4, 5], [4, 4]

# p1, q1 = [3.43, 3.91], [4.98, 4.08]
# p2, q2 = [5, 5], [3, 3]

p1, q1 = [1, 0], [5, 5]
p2, q2 = [0, 4], [2, 0]

x_max, y_max = max(p1[0], q1[0]), max(p1[1], q1[1])
x_min, y_min = min(p1[0], q1[0]), min(p1[1], q1[1])

den = (p1[0]-q1[0])*(p2[1]-q2[1])-(p1[1]-q1[1])*(p2[0]-q2[0])
if den != 0:
    num = (p2[0] - p1[0]) * (p2[1] - q2[1]) - (p2[0] - q2[0]) * (p2[0] - p1[0])
    print("num: {}, den: {}".format(num, den))
    intersect_coord = [p1[0] + (num / den) * (p1[0] - q1[0]), p1[1] + (num / den) * (p1[1] - q1[1])]
    if intersect_coord[0] >= x_min and intersect_coord[0] <= x_max and intersect_coord[1] >= y_min and intersect_coord[1] <= y_max:
        x_max, y_max = max(p2[0], q2[0]), max(p2[1], q2[1])
        x_min, y_min = min(p2[0], q2[0]), min(p2[1], q2[1])
        if intersect_coord[0] >= x_min and intersect_coord[0] <= x_max and intersect_coord[1] >= y_min and intersect_coord[1] <= y_max:
            print("Coordinate of intersecting-point: {}".format(intersect_coord))
        else:
            print("Lines are non parallel but does not intersect in the range")
    else:
        print("Lines are non parallel but does not intersect in the range")
else:
    print("Line segments are parallel")

plt.plot([p1[0], q1[0]], [p1[1], q1[1]], 'b')
plt.plot([p2[0], q2[0]], [p2[1], q2[1]], 'r')
plt.show()

