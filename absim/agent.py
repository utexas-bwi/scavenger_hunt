import world


class Agent:
    def __init__(self, map, hunt, start):
        self.map = map
        self.hunt = hunt
        self.travel_distance = 0
        self.current_node = start

    def traverse(self, dest):
        # Collect objects at current location
        for object in self.current_node.objects:
            if object in self.hunt:
                self.hunt.remove(object)
        if self.is_done():
            return
        # Find edge
        edge = None
        for e in self.map.edges:
            if e.n0 == self.current_node and e.n1 == dest:
                edge = e
                break
        if edge is None:
            raise RuntimeError(
                "agent tried to move along nonexistent edge %s -> %s" % (
                    str(self.current_node),
                    str(dest)
                )
            )
        # Travel edge
        self.travel_distance += edge.cost
        self.current_node = edge.n1

    def run(self):
        raise RuntimeError("should be overridden")

    def is_done(self):
        return len(self.hunt) == 0
