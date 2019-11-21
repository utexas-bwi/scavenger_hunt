import agent
import world


class ProbAgent(agent.Agent):
    """The probability-greedy agent visits the location with the highest
    probability of finding any object. If two locations are equally
    favorable, the closer of the two is chosen.
    """
    def setup(self):
        super().setup()

    def choose_next_path(self):
        best_node = None
        best_node_score = 0
        best_node_path = None
        for node in self.world.graph.nodes:
            if not self.visited(node):
                prob_nothing_here = 1;
                pot_objs = self.arrangement_space.pot_objs_at(node)
                path = self.world.graph.shortest_path(self.loc, node)
                # Node is scored based on the probability of finding any object
                # This is one minus the product of all the probabilities of the
                # objects not appearing at this location
                for pot_obj in pot_objs:
                    prob_nothing_here *= 1 - self.arrangement_space.prob_obj(pot_obj, node)
                # score is the probability that something is here
                score = 1 - prob_nothing_here
                if best_node is None or \
                   score > best_node_score or \
                   (score == best_node_score and path.cost < best_node_path.cost):
                    best_node = node
                    best_node_score = score
                    best_node_path = path
        assert best_node is not None
        return best_node_path

    def run(self):
        # Collect objects at current location and update the occurrence space
        super().run()

        if self.done():
            return

        # if self.active_path is None or \
        #    self.active_path_index == len(self.active_path):
        new_path = self.choose_next_path()

        # Visit next location in the active path
        self.go(new_path.nodes[1])
