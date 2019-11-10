import agent
import world


class ProbAgent(agent.Agent):
    """The probability-greedy agent visits the location with the highest
    aggregate probabiliy of finding new objects. If two locations are equally
    favorable, the closer of the two is chosen.
    """
    def setup(self):
        super().setup()
        self.active_path = None
        self.active_path_index = 0

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
                   score > best_node_score or \
                   (score == best_node_score and path.cost < best_node_dist):
                    best_node = node
                    best_node_score = score
                    best_node_dist = path.cost
        assert best_node is not None
        return best_node

    def run(self):
        # Collect objects at current location and update the occurrence space
        super().run()

        if self.done():
            return

        # If no currently active path, pick a destination and generate one
        if self.active_path is None or \
           self.active_path_index == len(self.active_path):
            dest = self.choose_next_loc()
            self.active_path = \
                self.world.graph.shortest_path(self.loc, dest).nodes
            self.active_path_index = 1

        # Visit next location in the active path
        self.go(self.active_path[self.active_path_index])
        self.active_path_index += 1
