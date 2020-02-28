import absim.prob_agent as prob_agent


class ProxAgent(prob_agent.ProbAgent):
    """The proximity-greedy agent visits the closest location with the potential
    to contain a new object. If two locations are equally favorable, the one
    with the higher aggregate probability of new object occurrences is chosen.

    This agent is ProbAgent with the primary and tiebreaking heuristics
    switched.
    """
    def choose_next_loc(self):
        best_node = None
        best_node_path = None
        best_node_score = 0
        for node in self.world.graph.nodes:
            if not self.visited(node):
                path = self.world.graph.shortest_path(self.loc, node)
                score = self.arrangement_space.prob_any_obj(node)
                # Node is scored based on probability of finding any object
                if best_node is None or \
                    (score and path.cost < best_node_path.cost) or \
                    (path.cost == best_node_path.cost and score > best_node_score):
                    best_node = node
                    best_node_score = score
                    best_node_path = path
        assert best_node is not None
        return best_node_path.nodes[1]
