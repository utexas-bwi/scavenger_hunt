import greedy_agent


class ProximityAgent(greedy_agent.GreedyAgent):
    """Visits the closest location that may potentially contain an unfound
    object. If two locations are equal in distance, the one with more unfound
    objects is chosen. If no adjacent nodes may contain unfound objects, a
    random adjacent node is chosen.

    Note that only nodes adjacent to the current node are considered. If the
    world graph is not complete, performance will be poor.
    """
    def compare_locs(self, best_dist, best_occs, new_dist, new_occs):
        return new_dist < best_dist and new_occs > 0 or \
               new_dist == best_dist and new_occs > best_occs
