from random_agent import RandomAgent

import agent
import math


class GreedyAgent(agent.Agent):
    """Visits the location with the greatest potential number of currently
    unfound objects. If two locations have the same number, the closer of the
    two is chosen. If no adjacent nodes may contain unfound objects, a random
    adjacent node is chosen.

    Note that only nodes adjacent to the current node are considered. If the
    world graph is not complete, performance will be poor.
    """
    def compare_locs(self, best_dist, best_occs, new_dist, new_occs):
        return new_occs > best_occs or \
               new_occs == best_occs and new_dist < best_dist

    def run(self):
        conns = self.map.connections[self.current_node]
        best_dist = math.inf
        best_occurrences = 0
        best_conn = None

        # For each potential destination
        for conn in conns:
            # Skip already visited locations
            if conn in self.visited:
                continue

            dist = self.map.cost(self.current_node, conn)
            occurrences = 0

            # Compute potential novel occurrences at destination
            for label in self.hunt:
                if self.map.prob_obj(label, conn) > 0:
                    occurrences += 1

            # Evaluate destination
            if best_conn is None or self.compare_locs(best_dist,
                                                      best_occurrences,
                                                      dist, occurrences):
              # New best destination
              best_conn = conn
              best_dist = dist
              best_occurrences = occurrences

        # If a good destination could not be found, pick a random one
        if best_conn is None:
            best_conn = RandomAgent.pick_uniform(conns)

        # Visit best
        self.traverse(best_conn)
