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
    c_nodes = random.randint(nodes_range[0], nodes_range[1])
    nodes = []
    for i in range(c_nodes):
        node = "l%s" % i
        for j in range(i + 1, c_nodes):
            cost = random.randint(cost_range[0], cost_range[1])
            out.write("%s l%s %s !\n" % (node, j, cost))
        nodes.append(node)

    # Start
    out.write("\n[start]\nl0\n")

    # Object
    out.write("\n[obj]\n")
    c_objs = random.randint(objects_range[0], objects_range[1])
    insts = []
    objs = []
    for i in range(c_objs):
        inst = "i%s" % i
        obj = "o%s" % i
        out.write("%s %s\n" % (inst, obj))
        insts.append(inst)
        objs.append(obj)

    # Distribution
    out.write("\n[distr]\n")
    for inst in insts:
        out.write(inst + " ")
        occurrences = random.randint(occurrences_range[0], occurrences_range[1])
        locs = nodes.copy()
        p = distribute(occurrences)
        for j in range(occurrences):
            out.write("%s %s " % (sample(locs), p[j] / 100))
        out.write("\n")

    # Hunt
    out.write("\n[hunt]\n")
    for obj in objs:
        out.write(obj + "\n")


if __name__ == "__main__":
    nodes_range = [5, 5]
    cost_range = [50, 200]
    objects_range = [4, 4]
    occurrences_range = [5, 5]

    import sys
    generate(sys.argv[1],
        nodes_range, cost_range, objects_range, occurrences_range
    )
