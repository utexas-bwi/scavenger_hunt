import agent
import copy
import math
import pathutil
import util


class TreeNode:
    """Simple B+ tree for generating collectively exhaustive occurrence trees.
    """
    def __init__(self, children, data):
        self.children = children
        self.data = data


def tree_depth_traverse(root, all_paths, current_path):
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
        all_paths.append(current_path)
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
    root = TreeNode([], None)
    leaves = [root]
    new_leaves = []

    # Generate event tree
    for distr in map.distributions:
        for event in distr.probs:
            for leaf in leaves:
                node = TreeNode([], [distr.label, event.copy()])
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
        distr_p = []
        for i in range(1, len(path)):
            distr_p.append(copy.deepcopy(path[i].data))
        all_distrs.append(distr_p)

    return all_distrs


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


def estimate_path_cost(map, hunt, path, occurrence_space, valid_vec):
    """Computes the expected cost of a path.

    Parameters
    ----------
    map : world.Map
        finalized world map
    hunt : list
        list of currently unfound object labels
    occurrence_space : list
        current occurrence space
    valid_vec : list
        boolean vector indicating which distributions in the occurrence space
        are still valid
    """
    total_cost = 0

    # Every possible distribution in the occurrence space is considered
    for j in range(len(occurrence_space)):
        # If this distribution has been invalidated, skip it
        if not valid_vec[j]:
            continue
        distr = occurrence_space[j]

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
            traveled += map.cost(current_node, path[i])
            current_node = path[i]
            # Collect items at current location
            new_hunt = current_hunt.copy()
            for item in current_hunt:
                if distr_label_at(map, distr, item, current_node):
                    new_hunt.remove(item)
            current_hunt = new_hunt
            # If everything has been found, we're done here
            if len(current_hunt) == 0:
                break

        # Contribution to total cost = probability of this distribution *
        #                              length of path necessary to finish
        total_cost += prob * traveled

    return total_cost


class DynamicAgent(agent.Agent):
    def epoch(self):
        """One-time occurrence space generation for the assigned map.
        """
        self.occurrence_space_orig = generate_occurrence_space(self.map)
        self.occurrence_space = []

    def setup(self):
        """DynamicAgent begins with no path and a full occurrence space.
        """
        super().setup()
        self.path = None
        self.path_index = 1
        # All possible object distributions for the map
        self.occurrence_space = copy.deepcopy(self.occurrence_space_orig)
        # Vector of bools indicating which distributions in the occurrence
        # space are still possible. As observations are made, distributions
        # may be eliminated.
        self.valid_occurrences = [True] * len(self.occurrence_space)
        self.p_exceptions = {}

    def observe(self, inst, node, seen):
        """Updates the active occurrence space by eliminating impossible
        distributions and renormalizing probabilities based on a world
        observation.

        Parameters
        ----------
        inst : str
            label of the observed instance
        node : str
            location of the observation
        seen : bool
            True if the instance was seen, False if it was not
        """
        # If all distributions involving this observation have already been
        # ruled out, don't bother
        possible = False
        for i in range(len(self.valid_occurrences)):
            if self.valid_occurrences[i]:
                for event in self.occurrence_space[i]:
                    if event[0] == inst and (node in event[1][0]) == seen:
                        possible = True
        if not possible:
            return

        # Eliminate distributions made impossible by this observation
        for i in range(len(self.valid_occurrences)):
            if self.valid_occurrences[i]:
                distr = self.occurrence_space[i]
                for event in distr:
                    if event[0] == inst and (node in event[1][0]) != seen:
                        self.valid_occurrences[i] = False
                        break

        if seen:
            # Mark probability of inst occurring at node as 1.0 in all
            # valid distributions
            for i in range(len(self.valid_occurrences)):
                if self.valid_occurrences[i]:
                    for event in self.occurrence_space[i]:
                        if event[0] == inst and node in event[1][0]:
                            event[1][1] = 1.0
        else:
            # Update occurrence space to reflect the increased chance
            # that inst occurs elsewhere, since it does not occur at
            # node
            for i in range(len(self.valid_occurrences)):
                if self.valid_occurrences[i]:
                    # Find probability of inst occurring at node
                    key = inst + node
                    p = self.map.prob_inst(inst, node) if \
                        key not in self.p_exceptions else self.p_exceptions[key]
                    for event in self.occurrence_space[i]:
                        if event[0] == inst and \
                           not util.approx(event[1][1], 1.0):
                            p_old = event[1][1]
                            event[1][1] = min(p_old / (1 - p), 1.0)
                            # Hacky solution that avoids changing the probabilty
                            # distributions in the world.Map itself
                            for loc in event[1][0]:
                                self.p_exceptions[inst + loc] = event[1][1]

    def run(self):
        # Determine which instances may appear at the current node
        expected_instances = [i for i in self.map.object_labels \
                              if self.map.prob_inst(i, self.current_node) > 0]
        # For every expected instance, update the active occurrence space
        # based on whether or not that instance actually appeared
        for inst in expected_instances:
            if self.map.object_labels[inst] in self.hunt:
                present = inst in self.map.nodes[self.current_node].instances
                self.observe(inst, self.current_node, present)

        # Ensure the occurrence space is still normalized
        total_prob = 0
        for i in range(len(self.occurrence_space)):
            if self.valid_occurrences[i]:
                prob = 1
                for event in self.occurrence_space[i]:
                    prob *= event[1][1]
                total_prob += prob
        assert util.approx(total_prob, 1.0)

        # Collect objects at the current node
        finds = self.traverse(None)

        # If the hunt is over, conclude
        if self.is_done():
            return

        # If no path is active or an object was just found, we need a new path
        # to follow
        if self.path is None or finds > 0:
            # Generate all possible paths through all nodes besides the current
            all_paths = []
            unvisited = [loc for loc in self.map.nodes \
                         if loc != self.current_node]
            pathutil.complete_traverse(unvisited, all_paths)

            # Identify path with lowest expected cost
            self.path = None
            self.path_index = 1
            best_cost = math.inf
            for path in all_paths:
                path.insert(0, self.current_node)
                cost = estimate_path_cost(self.map, self.hunt,
                                          path, self.occurrence_space,
                                          self.valid_occurrences)
                if self.path is None or cost < best_cost:
                    self.path = path
                    best_cost = cost
        assert self.path is not None

        # Step along current path if end not yet reached
        if self.path_index < len(self.path):
            self.traverse(self.path[self.path_index])
            self.path_index += 1
        # If end reached, hunt should be at its end; collect and conclude
        else:
            self.traverse(None)
            assert len(self.hunt) == 0
