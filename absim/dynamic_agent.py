import agent
import copy
import math


class TreeNode:
    """Simple B+ tree for generating collectively exhaustive occurrence trees.
    """
    def __init__(self, children=[], data=None):
        self.children = children
        self.data = data


def tree_depth_traverse(root, all_paths, current_path=[]):
    """Finds all paths from tree root to leaves.

    Parameters
    ----------
    root : TreeNode
        root node to examine
    all_paths : list
        list to populate with paths
    current_path : list
        incomplete path in the current recursion branch
    """
    current_path.append(root)
    if len(root.children) == 0:
        all_paths.append(current_path.copy())
        return
    for child in root.children:
        tree_depth_traverse(child, all_paths, current_path.copy())


def generate_occurrence_space(map):
    """Generates all possible arrangements of instances on a scavenger hunt map
    according to its occurrence model.

    Parameters
    ----------
    map : world.Map
        finalized map

    Return
    ------
    list
        list of instance arrangements of the form; each arrangement is of the
        form [
            (inst0_label, ([node00, ..., node0M], P0)),
            ...
            (instN_label, ([nodeN0, ..., nodeNM], PN))
        ]. Note that probabilities are included only for convenience; this
        arrangement is a concrete observation, with an instance K being present
        at the nodes in its tuple, and a probability PK of that having occurred.
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
    tree_depth_traverse(root, all_paths)

    # Put those paths into a more useful form
    all_distrs = []
    for path in all_paths:
        distr = []
        for i in range(1, len(path)):
            distr.append(path[i].data)
        all_distrs.append(distr)

    return all_distrs


def complete_graph_traverse(unvisited_nodes, all_paths, current_path=[]):
    """Finds all permutations of a node list, i.e. all paths through a complete
    graph that visit each node once.

    Parameters
    ----------
    unvisited_nodes : list
        nodes to traverse
    all_paths : list
        list to be populated with paths
    current_path : list
        incomplete path in the current recursion branch; leave default for
        topmost call
    """
    # No more nodes to visit; add path and end
    if len(unvisited_nodes) == 0:
        all_paths.append(current_path.copy())
        return
    # Branch down each subsequent unvisited node
    for node in unvisited_nodes:
        new_path = current_path.copy()
        new_path.append(node)
        new_unvisited_nodes = unvisited_nodes.copy()
        new_unvisited_nodes.remove(node)
        complete_graph_traverse(new_unvisited_nodes, all_paths, new_path)


def distr_label_at(map, distr, label, node):
    """Gets is an instance with some object label is present at a node in a
    distribution.

    Parameters
    ----------
    distr : list
        distribution from an occurrence space generated via
        generate_occurrence_space()
    label : str
        object label
    node : str
        node name

    Return
    ------
    bool
        if label is present
    """
    for occur in distr:
        if map.object_labels[occur[0]] == label:
            if node in occur[1][0]:
                return True
    return False


def estimate_path_cost(map, hunt, path, occurrence_space):
    """Computes the expected cost of a path given a map, list of unfound
    objects, and an occurrence space.
    """
    total_cost = 0

    for distr in occurrence_space:
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
    def reset(self):
        super(DynamicAgent, self).reset()
        pass

    def epoch(self):
        print("[DynamicAgent] Building occurrence space...")
        self.occurrence_space = generate_occurrence_space(self.map)
        print("[DynamicAgent] Built occurrence space with %s events." % \
              len(self.occurrence_space))

    def setup(self):
        self.path = None
        self.path_index = 1
        self.last_find_count = 0

    def run(self):
        # Generate all possible paths through remaining nodes
        if self.path is None or self.last_find_count > 0:
            all_paths = []
            unvisited = [loc for loc in self.map.nodes \
                         if loc not in self.visited]
            complete_graph_traverse(unvisited, all_paths)

            # Identify path with lowest expected cost
            self.path = None
            self.path_index = 1
            best_cost = math.inf
            for path in all_paths:
                path.insert(0, self.current_node)
                cost = estimate_path_cost(self.map, self.hunt,
                                          path, self.occurrence_space)
                if self.path is None or cost < best_cost:
                    self.path = path
                    best_cost = cost

        # Step along current path if end not yet reached
        if self.path_index < len(self.path):
            self.last_find_count = self.traverse(self.path[self.path_index])
            self.path_index += 1
        # If end reached, hunt should be at its end; collect and conclude
        else:
            self.traverse(None)
            if len(self.hunt) > 0:
                raise RuntimeError("impossible scavenger hunt")
