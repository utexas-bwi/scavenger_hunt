import agent as agent
import world as world


class ProbAgent(agent.Agent):
    """The probability-greedy agent visits the location with the highest
    probability of finding any object. If two locations are equally
    favorable, the closer of the two is chosen.
    """
    def setup(self):
        super().setup()

    def choose_next_loc(self):
        best_node = None
        best_node_score = 0
        best_node_path = None
        for node in self.world.graph.nodes:
            if not self.visited(node):
                path = self.world.graph.shortest_path(self.loc, node)
                score = self.arrangement_space.prob_any_obj(node)
                if best_node is None or \
                   score > best_node_score or \
                   (score == best_node_score and path.cost < best_node_path.cost):
                    best_node = node
                    best_node_score = score
                    best_node_path = path
        assert best_node is not None
        return best_node_path.nodes[1]

    def run(self):
        # Collect objects at current location and update the occurrence space
        super().run()

        if self.done():
            return

        # Visit next location in the active path
        self.go(self.choose_next_loc())

