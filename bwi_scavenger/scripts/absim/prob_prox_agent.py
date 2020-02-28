import absim.agent as agent
import absim.world as world


class ProbProxAgent(agent.Agent):
    """This agent visits the location with the least expected wasted cost.
    This cost is the product of the probability of not finding an object at
    the location times the distance to the location.
    """
    def setup(self):
        super().setup()

    def choose_next_path(self):
        best_node = None
        best_node_score = 0
        best_node_path = None
        for node in self.world.graph.nodes:
            if not self.visited(node):
                prob_nothing_here = 1
                pot_objs = self.arrangement_space.pot_objs_at(node)
                path = self.world.graph.shortest_path(self.loc, node)
                # score is the expected wasted distance of going to that node
                # this means high is bad and low is good
                for pot_obj in pot_objs:
                    prob_nothing_here *= 1 - self.arrangement_space.prob_obj(pot_obj, node)
                score = path.cost * prob_nothing_here
                if best_node is None or \
                   score < best_node_score:
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
