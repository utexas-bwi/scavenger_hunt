from random_agent import RandomAgent

import agent
import math


class GreedyAgent(agent.Agent):
    """Visits the location with the greatest potential number of currently
    unfound objects. If two locations have the same number, the closer of the
    two is chosen.

    Note that only nodes adjacent to the current node are considered. If the
    world graph is not complete, performance will be poor.
    """
    def compare_locs(self, min_dist, max_hits, new_dist, new_hits):
        return new_hits > max_hits or                                          \
               (new_hits == max_hits and new_dist < min_dist)

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
            for label in self.map.labels_at_node(conn):
                if label in self.hunt:
                    occurrences += 1

            # Evaluate destination
            if best_conn is None or (                                          \
               occurrences > best_occurrences or                               \
               occurrences == best_occurrences and dist < best_dist):
              # New best destination
              best_conn = conn
              best_dist = dist
              best_occurrences = occurrences

        # If a good destination could not be found, pick a random one
        if best_conn is None:
            best_conn = RandomAgent.pick_uniform(conns)

        # Visit best
        self.traverse(best_conn)
