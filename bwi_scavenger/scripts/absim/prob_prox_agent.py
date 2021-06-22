import agent as agent
import world as world


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
                path = self.world.graph.shortest_path(self.loc, node)
                
                prob_any = self.arrangement_space.prob_any_obj(node)
                score = prob_any / path.cost

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
