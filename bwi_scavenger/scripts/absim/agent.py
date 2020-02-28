import absim.world as world


class Agent:
    """Framework for a scavenger hunt-solving algorithm.
    """
    def __init__(self, world, hunt, start_loc):
        """Agent will solve HUNT in WORLD starting at START_LOC.
        """
        self.world = world
        self.hunt = hunt
        self.start_loc = start_loc

    def epoch(self):
        """Optional once-per-simulation logic.
        """
        pass

    def setup(self):
        """Once-per-hunt logic. If overriding, make sure to invoke
        super().setup().
        """
        self.loc = self.start_loc
        self.travel_distance = 0
        self.visited_count = [0] * len(self.world.graph.nodes)
        self.visited_count[self.loc] = 1
        self.found = [False] * len(self.hunt)
        self.path = [self.loc]
        self.arrangement_space = world.ArrangementSpace(self.world)
        self.objs_at_loc = []

    def done(self):
        """Returns if the hunt has been completed.
        """
        for f in self.found:
            if not f:
                return False
        return True

    def visited(self, loc):
        """Returns if LOC has been visited.
        """
        return self.visited_count[loc] > 0

    def collect(self):
        """Collects objects at the current location and removes them from
        the hunt list.
        """
        for i in range(len(self.hunt)):
            if not self.found[i] and self.hunt[i] in self.objs_at_loc:
                self.found[i] = True

    def go(self, loc):
        """Visits LOC.
        """
        self.path.append(loc)
        self.visited_count[loc] += 1
        self.travel_distance += self.world.cost(self.loc, loc)
        self.loc = loc

    def run(self):
        """Iterative logic. Must be overridden. The parent implementation should
        always be called first in the overriding imlpementation.
        """
        # Collect objects at the current location
        self.collect()
        if self.done():
            return

        # Make observations at the current location
        if self.visited_count[self.loc] == 1:
            expected_objs = self.arrangement_space.pot_objs_at(self.loc)
            actual_objs = self.objs_at_loc

            for expected_obj in expected_objs:
                seen = expected_obj in actual_objs
                self.arrangement_space.observe(expected_obj, self.loc, seen)
