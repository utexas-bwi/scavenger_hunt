import random
import math
import numpy as np, numpy.random

def sample(ls):
    item = random.choice(ls)
    ls.remove(item)
    return item

# Distribute probabilities uniformly with Dirichlet distribution
def distribute(count):
    return np.random.dirichlet(np.ones(count),size=1)[0]


def generate(fname, nodes_range, cost_range, objects_range, occurrences_range):
    out = open(fname, "w")

    # Map
    out.write("[map]\n")
    start_marked = False
    c_nodes = random.randint(nodes_range[0], nodes_range[1])
    nodes = []
    points = []
    first = True
    second = True

    for i in range(c_nodes):
        node = "l%s" % i

        # Generate a random point
        if first: #Robot in middle
            point = (random.randrange(200, 300, 1), random.randrange(200, 300, 1))
            first = False
        elif second: # One point on one side
            point = (random.randrange(100, 150, 1), random.randrange(100, 150, 1))
            second = False
        else: # All other points on different side
            point = (random.randrange(500, 700, 1), random.randrange(500, 700, 1))

        # Original
        # point = (random.randrange(0, 500, 1), random.randrange(0, 500, 1))

        index = 0

        # Get distances between each point
        for point2 in points:
            cost = math.sqrt((point2[0] - point[0])**2 + (point2[1] - point[1])**2)
            start = "*" if not start_marked else ""
            start_marked = True
            out.write("%s%s l%s %s\n" %(node, start, index, cost))
            index = index + 1

        nodes.append(node)
        points.append(point)

    # Distribution
    out.write("\n[distr]\n")
    c_objs = random.randint(objects_range[0], objects_range[1])
    for obj in range(c_objs):
        out.write("o%s " % obj)
        occurrences = random.randint(occurrences_range[0], occurrences_range[1])
        locs = nodes.copy()
        p = distribute(occurrences)
        for j in range(len(p)):
            out.write("%s %s " % (sample(locs), p[j]))
        out.write("\n")


if __name__ == "__main__":
    nodes_range = [5, 5]
    cost_range = [50, 200]
    objects_range = [4, 4]
    occurrences_range = [1, 3]

    import sys
    generate(sys.argv[1],
        nodes_range, cost_range, objects_range, occurrences_range
    )
