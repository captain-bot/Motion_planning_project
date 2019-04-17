import matplotlib.pyplot as plt

p1, q1 = [0, 0], [5, 5]
p2, q2 = [0, 4], [2, 0]

x_max, y_max = max(p1[0], q1[0]), max(p1[1], q1[1])
x_min, y_min = min(p1[0], q1[0]), min(p1[1], q1[1])

den = (p1[0]-q1[0])*(p2[1]-q2[1])-(p1[1]-q1[1])*(p2[0]-q2[0])
if den != 0:
    num = (p2[0] - p1[0]) * (p2[1] - q2[1]) - (p2[0] - q2[0]) * (p2[1] - p1[1])
    print("num: {}, den: {}".format(num, den))
    intersect_coord = [p1[0] + (num / den) * (p1[0] - q1[0]), p1[1] + (num / den) * (p1[1] - q1[1])]
    print(intersect_coord)

plt.plot([p1[0], q1[0]], [p1[1], q1[1]], 'b')
plt.plot([p2[0], q2[0]], [p2[1], q2[1]], 'r')
plt.show()
