import agent
import pathutil


class OptimalAgent(agent.Agent):
    def run(self):
        self.traverse(None)

        if self.is_done():
            return

        # Generate all paths through those nodes
        all_paths = []
        all_nodes = [node for node in self.map.nodes if node != self.current_node]
        pathutil.complete_traverse(all_nodes, all_paths, [])

        # Identify cheapest path
        best_path = None
        best_cost = -1
        for path in all_paths:
            path.insert(0, self.current_node)
            cost = 0
            i = 0
            virtual_hunt = self.hunt.copy()

            while i < len(path):
                virtual_hunt_new = virtual_hunt.copy()
                for lab in self.map.labels_at_node(path[i]):
                    if lab in virtual_hunt:
                        virtual_hunt_new.remove(lab)
                virtual_hunt = virtual_hunt_new

                if len(virtual_hunt) == 0:
                    break

                if i < len(path) - 1:
                    cost += self.map.cost(path[i], path[i + 1])
                    i += 1

            if best_path is None or cost < best_cost:
                best_path = path
                best_cost = cost

        self.traverse(best_path[1])
