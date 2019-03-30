import random
import math
import matplotlib.pyplot as plt

_BIG = 10000
_STRIDE = 0.4


class Node(object):
    def __init__(self):
        self.coord = None
        self.cost = None
        self.parent = None

    def __repr__(self):
        return "------- \nNode object \ncoordinate: {} \ncost: {} \nparent: {} \n-------".format(self.coord, self.cost, self.parent)


class RRT(object):
    def __init__(self, start, goal):
        self.tree = list()
        self.start = start
        self.goal = goal

        start_node = Node()
        start_node.coord = start
        start_node.cost = 0
        start_node.parent = 0
        self.tree.append(start_node)
        self.num_nodes = 1

    def AddNode(self, x):
        self.tree.append(x)
        self.num_nodes += 1

    def DelNode(self, idx):
        self.tree = self.tree[:idx] + self.tree[idx+1:]
        self.num_nodes -= 1

    def __repr__(self):
        return "Tree has {} nodes".format(len(self.tree))

    def GrowTree(self, max_itr=100, seed_val=100):
        random.seed(seed_val)
        for i in range(max_itr):
            # Generate X_rand
            x_rand = [random.uniform(self.start[0], self.goal[0]),
                      random.uniform(self.start[1], self.goal[1])]
            print(x_rand)

            # Get x_nearest
            nearest_idx, local_cost = self.GetNearestNode(x_rand)
            print(nearest_idx)

            # Steer towards x_rand from x_nearest
            x_new = Node()
            x_new.coord = self.Steer(self.tree[nearest_idx].coord, x_rand)
            x_new.cost = self.tree[nearest_idx].cost + local_cost
            x_new.parent = nearest_idx

            # Add x_new to the tree
            self.AddNode(x_new)

    def Steer(self, q1, q2):
        d_current = math.sqrt((q1[0] - q2[0])**2 + (q1[1] - q2[1])**2)
        if d_current > _STRIDE:
            x = q1[0] + _STRIDE * (q2[0] - q1[0])
            y = q1[1] + _STRIDE * (q2[1] - q1[1])
            return [x, y]
        else:
            return q2

    def GetNearestNode(self, n):
        d_min = _BIG
        for i, j in enumerate(self.tree):
            d_current = math.sqrt((j.coord[0]-n[0])**2 + (j.coord[1]-n[1])**2)
            if d_current < d_min:
                d_min_idx = i
                d_min = d_current
        return d_min_idx, d_min

    def VisualizeTree(self):
        for i, j in enumerate(self.tree):
            # plt.plot(j.coord[0], j.coord[1], 'bo', markersize=2)
            if i > 0:
                plt.plot([self.tree[j.parent].coord[0], j.coord[0]], [self.tree[j.parent].coord[1], j.coord[1]], 'r', linewidth=0.5)
        plt.grid()
        plt.title(self.__class__.__name__ + " 2D implementation")
        plt.xlabel("x")
        plt.ylabel("y")
        plt.xlim([self.start[0], self.goal[0]])
        plt.ylim([self.start[1], self.goal[1]])
        plt.axis('square')
        plt.show()


class RRTstar(RRT):
    def __init__(self, start, goal):
        RRT.__init__(self, start=start, goal=goal)

    def GrowTree(self, max_itr=100, seed_val=100):
        random.seed(seed_val)
        for i in range(max_itr):
            # Generate X_rand
            x_rand = [random.uniform(self.start[0], self.goal[0]),
                      random.uniform(self.start[1], self.goal[1])]
            print(x_rand)

            # Get x_nearest
            nearest_idx, local_cost = self.GetNearestNode(x_rand)
            print(nearest_idx)

            # Steer towards x_rand from x_nearest
            x_new = Node()
            x_new.coord = self.Steer(self.tree[nearest_idx].coord, x_rand)
            x_new.cost = self.tree[nearest_idx].cost + local_cost
            x_new.parent = nearest_idx

            # Select best parent : K nearest nodes
            k_rad = 0.25
            near_indx_list = list()
            for node_idx in range(self.num_nodes):
                current_node = self.tree[node_idx]
                if math.sqrt((current_node.coord[0]-x_new.coord[0])**2
                             + (current_node.coord[1]-x_new.coord[1])**2) <= k_rad:
                    near_indx_list.append(node_idx)

            x_min_idx = nearest_idx
            C_min = x_new.cost
            for k in near_indx_list:
                alter_cost = self.tree[k].cost + math.sqrt(
                    (self.tree[k].coord[0] - x_new.coord[0])**2 + (self.tree[k].coord[1] - x_new.coord[1])**2)
                if alter_cost < C_min:
                    C_min = alter_cost
                    x_min_idx = k

            x_new.parent = x_min_idx
            x_new.cost = C_min

            # Rewiring
            for k2 in near_indx_list:
                new_cost = x_new.cost + math.sqrt(
                    (self.tree[k2].coord[0] - x_new.coord[0])**2 + (self.tree[k2].coord[1] - x_new.coord[1])**2)
                if new_cost < self.tree[k2].cost:
                    self.tree[k2].parent = self.num_nodes

            # Add x_new to the tree
            self.AddNode(x_new)
        print(self.__class__.__name__)


# # Node 1
# new_node1 = Node()
# new_node1.coord = [0.1, 0.6]
# new_node1.cost = 15
# new_node1.parent = 1
# print(new_node1)
#
# # Node 2
# new_node2 = Node()
# new_node2.coord = [0.1, 0.2]
# new_node2.cost = 10
# new_node2.parent = 2
# print(new_node2)
#
# # Node 1
# new_node3 = Node()
# new_node3.coord = [0.1, 0.3]
# new_node3.cost = 5
# new_node3.parent = 3
# print(new_node3)

# st, gl = [0, 0], [50, 50]
# tree = RRTstar(st, gl)
# tree.AddNode(new_node1)
# tree.AddNode(new_node2)
# tree.AddNode(new_node3)
# print(tree)

# tree.DelNode(1)
# print(tree)
#
# tree.GrowTree(1)

# # RRT Test: Initiate and grow tree
# st, gl = [0, 0], [10, 10]
# tree = RRT(st, gl)
# tree.GrowTree(max_itr=5000)
# print(tree)
#
# # Visualize tree
# tree.VisualizeTree()

# RRT-Star Test: Initiate and grow tree
st, gl = [0, 0], [10, 10]
tree = RRTstar(st, gl)
tree.GrowTree(max_itr=5000)
print(tree)

# Visualize tree
tree.VisualizeTree()
