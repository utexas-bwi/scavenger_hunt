"""General scavenger hunt agent logic.
"""
import world


class Agent:
    """Abstraction of a scavenger hunt-solving algorithm.
    """
    def __init__(self, map, hunt, start):
        """
        Parameters
        ----------
        map : world.Map
            finalized game world
        hunt : list of str
            object hunt list
        start : str
            name of starting node
        """
        self.map = map
        self.original_hunt = hunt
        self.hunt = hunt.copy()
        self.travel_distance = 0
        self.start_node = start
        self.current_node = start
        self.visited = [start]

    def traverse(self, dest):
        """Collects objects at the current location and then moves to another.

        Parameters
        ----------
        dest : str
            name of destination node or None to collect objects at current node

        Return
        ------
        int
            number of objects collected at current node (not destination node)
        """
        # Collect objects at current location
        finds = 0
        for label in self.map.labels_at_node(self.current_node):
            if label in self.hunt:
                self.hunt.remove(label)
                finds += 1

        if dest is None or self.is_done():
            return finds

        # Mark destination as visited
        self.visited.append(dest)

        # Travel edge
        self.travel_distance += self.map.cost(self.current_node, dest)
        self.current_node = dest

        return finds

    def reset(self):
        """Resets data specific to a particular scavenger hunt. This should be
        extended by child classes where appropriate.
        """
        self.hunt = self.original_hunt.copy()
        self.current_node = self.start_node
        self.visited = [self.current_node]
        self.travel_distance = 0

    def epoch(self):
        """One-time epoch logic, called just prior to simulation begin.
        """
        pass

    def setup(self):
        """One-time setup logic, called just prior to each scavenger hunt.
        """
        pass

    def run(self):
        """Traversal logic. This method is called repeatedly until is_done().
        Each run() call should result in exactly one call to traverse().
        """
        raise RuntimeError("should be overridden")

    def is_done(self):
        """Has the scavenger hunt been completed?

        Return
        ------
        bool
            if all objects have been found
        """
        return len(self.hunt) == 0
