import agent
import copy
import math


class TreeNode:
    """Simple B+ tree for generating collectively exhaustive occurrence trees.
    """
    def __init__(self, children=[], data=None):
        self.children = children
        self.data = data


def tree_depth_traverse(root, all_paths, current_path):
    """Recursively finds all paths from the root of a tree to its leaves.
    """
    current_path.append(root)
    if len(root.children) == 0:
        all_paths.append(current_path.copy())
        return
    for child in root.children:
        tree_depth_traverse(child, all_paths, current_path.copy())


def all_distributions(map):
    """ Generates a list of the form
    [
        [(inst00, locs00), (inst01, locs01), ..., (inst0N, locs0N)],
        [(inst10, locs10), (inst11, locs11), ..., (inst1N, locs1N)],
        ...,
        [(instM0, locsM0), (instM1, locsM1), ..., (instMN, locsMN)]
    ]
    which represents all possible instance arrangements on a map. Each member of
    the topmost list is a different possible arrangement of instances, that is,
    a list of all instances paired with the locations from which they can be
    seen in that particular arrangement. These arrangements are collectively
    exhaustive and mutually exclusive.

    Parameters
    ----------
    map : world.Map
        finalized map

    Return
    ------
    list
        all possible object arrangements
    """
    root = TreeNode()
    leaves = [root]
    new_leaves = []

    # Generate event tree
    for distr in map.distributions:
        for event in distr.probs:
            for leaf in leaves:
                node = TreeNode([], (distr.label, event))
                leaf.children.append(node)
                new_leaves.append(node)
        leaves = new_leaves.copy()
        new_leaves.clear()

    # Generate all paths from root to leaf of event tree
    all_paths = []
    tree_depth_traverse(root, all_paths, [])

    # Put those paths into a more useful form
    all_distrs = []
    for path in all_paths:
        distr = []
        for i in range(1, len(path)):
            distr.append(path[i].data)
        all_distrs.append(distr)

    return all_distrs


def complete_graph_traverse(unvisited_nodes, all_paths, current_path):
    """Recursively finds all permutations of a node list, i.e. all paths through
    a complete graph that visit each node once.
    """
    if len(unvisited_nodes) == 0:
        all_paths.append(current_path.copy())
        return
    for node in unvisited_nodes:
        new_path = current_path.copy()
        new_path.append(node)
        new_unvisited_nodes = unvisited_nodes.copy()
        new_unvisited_nodes.remove(node)
        complete_graph_traverse(new_unvisited_nodes, all_paths, new_path)


def distr_label_at(map, distr, label, node):
    for occur in distr:
        if map.object_labels[occur[0]] == label:
            if node in occur[1][0]:
                return True
    return False


def estimate_path_cost(map, hunt, path, distributions):
    total_cost = 0

    for distr in distributions:
        # Compute probability of distribution occurring
        prob = 1
        for event in distr:
            prob *= event[1][1]

        # Compute estimated cost of path
        current_hunt = hunt.copy()
        traveled = 0
        current_node = path[0]

        # Move along path until all objects found
        for i in range(1, len(path)):
            # Collect items at current location
            for item in current_hunt:
                if distr_label_at(map, distr, item, current_node):
                    current_hunt.remove(item)
            # If everything has been found, we're done here
            if len(current_hunt) == 0:
                break
            # Otherwise, move to the next node in the path
            traveled += map.cost(current_node, path[i])
            current_node = path[i]

        # Contribution to total cost = probability of this distribution *
        #                              length of path necessary to complete hunt
        total_cost += prob * traveled

    return total_cost


class DynamicAgent(agent.Agent):
    def setup(self):
        # Generate the occurrence space
        self.occurrence_space = all_distributions(self.map)

        # Generate all possible paths through remaining nodes
        all_paths = []
        unvisited = [loc for loc in self.map.nodes if loc not in self.visited]
        complete_graph_traverse(unvisited, all_paths, [])

        # Identify path with lowest expected cost
        best_path = None
        best_cost = math.inf
        est = 0
        for path in all_paths:
            est += 1
            print(est, "/", len(all_paths))
            path.insert(0, self.current_node)
            cost = estimate_path_cost(self.map, self.hunt,
                                      path, self.occurrence_space)
            if best_path is None or cost < best_cost:
                best_path = path
                best_cost = cost

        self.path = best_path
        self.path_index = 1


    def run(self):
        # # Generate paths through all unvisited nodes
        # unvisited = [loc for loc in self.map.nodes if loc not in self.visited]
        #
        # # If we've been everywhere, collect and conclude
        # if len(unvisited) == 0:
        #     self.traverse(None)
        #     if len(self.hunt) != 0:
        #         raise RuntimeError("impossible scavenger hunt")
        #     return

        if self.path_index < len(self.path):
            self.traverse(self.path[self.path_index])
            self.path_index += 1
        else:
            self.traverse(None)
