import random


def sample(ls):
    item = random.choice(ls)
    ls.remove(item)
    return item


def distribute(count):
    if count == 1:
        return [100]
    p = None
    p_total = 0
    d = []
    for i in range(count):
        if p is None:
            p = random.randint(0, 100)
        elif i < count - 1:
            p = random.randint(0, 100 - p_total)
        else:
            p = 100 - p_total
        p_total += p
        d.append(p)
    return d


def generate(fname, nodes_range, cost_range, objects_range, occurrences_range):
    out = open(fname, "w")

    # Map
    out.write("[map]\n")
    start_marked = False
    c_nodes = random.randint(nodes_range[0], nodes_range[1])
    nodes = []
    for i in range(c_nodes):
        node = "l%s" % i
        for j in range(i + 1, c_nodes):
            cost = random.randint(cost_range[0], cost_range[1])
            start = "*" if not start_marked else ""
            out.write("%s%s l%s %s\n" % (node, start, j, cost))
            start_marked = True
        nodes.append(node)

    # Distribution
    out.write("\n[distr]\n")
    c_objs = random.randint(objects_range[0], objects_range[1])
    for obj in range(c_objs):
        out.write("o%s " % obj)
        occurrences = random.randint(occurrences_range[0], occurrences_range[1])
        locs = nodes.copy()
        p = distribute(occurrences)
        for j in range(occurrences):
            out.write("%s %s " % (sample(locs), p[j] / 100))
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
