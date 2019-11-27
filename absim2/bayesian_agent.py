import agent
import world


class BayesianAgent(agent.Agent):
    """The exhaustive Bayesian agent evaluates all possible solutions at every
    step of the hunt using Bayesian search theory. It always visits the first
    node in the path with the lowest expected cost given the current arrangement
    space.
    """
    def expected_path_cost(self, path):
        cost = 0
        # For each valid arrangement
        for i in range(0, len(self.arrangement_space.arrangements)):
            if self.arrangement_space.valid_arrangements[i]:
                arrangement = self.arrangement_space.arrangements[i]
                travel_distance = 0
                # Determine which objects are yet to be found
                hunt = []
                for j in range(len(self.hunt)):
                    if not self.found[j]:
                        hunt.append(self.hunt[j])
                # Step along path until hunt completion
                for j in range(1, len(path)):
                    # If hunt complete, stop early
                    if len(hunt) == 0:
                        break
                    # Move along path
                    n_to, n_from = path[j], path[j-1]
                    travel_distance += \
                        self.world.graph.shortest_path(n_to, n_from).cost
                    # Collect objects
                    new_hunt = hunt.copy()
                    for obj in hunt:
                        if arrangement.contains(obj, n_to):
                            new_hunt.remove(obj)
                    hunt = new_hunt
                # Factor in expected contribution
                cost += travel_distance * arrangement.prob()

        return cost

    def run(self):
        # Collect objects at current location and update the occurrence space
        super().run()

        if self.done():
            return

        # Determine the lowest cost path, whose nodes may not be adjacent
        paths = self.world.graph.permute_paths(self.loc)
        best_path = None
        best_path_cost = 0
        for path in paths:
            path_cost = self.expected_path_cost(path)
            if best_path is None or path_cost < best_path_cost:
                best_path = path
                best_path_cost = path_cost
        assert best_path is not None

        # Convert to a traversable path of adjacent nodes
        path = self.world.graph.stitch_path(best_path)

        # Visit first loc in selected path
        self.go(path[1])
