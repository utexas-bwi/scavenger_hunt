from random_agent import RandomAgent

import agent
import math
import pathutil


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

    def setup(self):
        self.active_path = None
        self.path_index = 0

    def run(self):
        unvisited = [l for l in self.map.nodes if l not in self.visited]

        # If no unvisited nodes, hunt should be done
        if len(unvisited) == 0:
            self.traverse(None)
            if len(self.hunt) > 0:
                raise RuntimeError("impossible hunt")
            return

        # Move along path
        self.path_index += 1

        # Reset active path if end reached
        if self.active_path is not None and \
           self.path_index == len(self.active_path):
            self.active_path = None

        # If no active path, compute one
        if self.active_path is None:
            # Identify prefered destination
            best_dist = math.inf
            best_occurrences = 0
            best_conn = None

            for conn in unvisited:
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
                best_conn = RandomAgent.pick_uniform(unvisited)

            # Generate path to destination
            self.active_path = pathutil.dijkstra(
                self.map, self.current_node, best_conn
            )
            self.path_index = 1

        # Visit next node in active path
        self.traverse(self.active_path[self.path_index])
