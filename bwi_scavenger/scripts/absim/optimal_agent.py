import agent as agent


class OptimalAgent(agent.Agent):
    """The offline optimal agent essentially cheats by looking up the locations
    containing target objects and traveling the shortest path between them. This
    provides a bound on the performance of other algorithms.
    """
    def epoch(self):
        self.all_paths = None

    def setup(self):
        super().setup()

        # Generate all possible paths
        if self.all_paths is None:
            self.all_paths = self.world.graph.permute_paths(self.loc)

        # Identify which locations contain objects
        needed_locs = []
        for loc in self.world.graph.nodes:
            if len(self.world.occurrences[loc]) > 0:
                needed_locs.append(loc)

        # Identify cheapest path that visits all necessary nodes
        self.optimal_path = None
        best_cost = -1
        for path in self.all_paths:
            cost = 0
            unvisited = needed_locs.copy()
            i = 0
            while len(unvisited) > 0:
                loc = path[i]
                if loc in unvisited:
                    unvisited.remove(loc)
                if i < len(path) - 1 and len(unvisited) > 0:
                    cost += \
                        self.world.graph.shortest_path(loc, path[i + 1]).cost
                    i += 1
            if self.optimal_path is None or cost < best_cost:
                self.optimal_path = path
                best_cost = cost
        assert self.optimal_path is not None

        self.optimal_path = self.world.graph.stitch_path(self.optimal_path)
        self.path_index = 1

    def run(self):
        # Collect objects at current location and update the occurrence space
        super().run()
        if self.done():
            return

        # Advance along the optimal path
        self.go(self.optimal_path[self.path_index])
        self.path_index += 1
