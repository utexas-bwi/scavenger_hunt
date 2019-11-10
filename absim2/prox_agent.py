import prob_agent


class ProxAgent(prob_agent.ProbAgent):
    """The proximity-greedy agent visits the closest location with the potential
    to contain a new object. If two locations are equally favorable, the one
    with the higher aggregate probability of new object occurrences is chosen.

    This agent is ProbAgent with the primary and tiebreaking heuristics
    switched.
    """
    def choose_next_loc(self):
        best_node = None
        best_node_score = 0
        best_node_dist = 0
        for node in self.world.graph.nodes:
            if not self.visited(node):
                score = 0
                pot_objs = self.arrangement_space.pot_objs_at(node)
                path = self.world.graph.shortest_path(self.loc, node)
                # Node is scored based on aggregate probability of novel finds
                for pot_obj in pot_objs:
                    score += self.arrangement_space.prob_obj(pot_obj, node)
                if best_node is None or \
                   (score > 0 and path.cost < best_node_dist) or \
                   (path.cost == best_node_dist and score > best_node_score):
                    best_node = node
                    best_node_score = score
                    best_node_dist = path.cost
        assert best_node is not None
        return best_node
