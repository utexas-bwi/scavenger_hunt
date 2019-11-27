import agent
import world


class SalesmanAgent(agent.Agent):
    """The salesman agent disregards the scavenger hunt entirely and takes the
    shortest path between all locations.
    """
    def epoch(self):
        super().epoch()
        print("Generating paths...");
        # Generate all paths through the world
        self.salesman_paths = self.world.graph.permute_paths(self.start_loc)
        print("Done.")

    def setup(self):
        super().setup()
        self.path_index = 0

        # Identify the shortest path, whose nodes may not be adjacent
        path = None
        best_path_cost = 0
        for p in self.salesman_paths:
            cost = self.world.graph.path_cost(p)
            if path is None or cost < best_path_cost:
                path = p
                best_path_cost = cost

        # Convert to a traversable path of adjacent nodes
        self.path = self.world.graph.stitch_path(path)

    def run(self):
        # Collect objects at current location and update the occurrence space
        super().run()

        if self.done():
            return

        # Path finished && hunt incomplete -> something wrong
        assert self.path_index < len(self.path) - 1

        # Move to next loc in path
        self.path_index += 1
        self.go(self.path[self.path_index])
