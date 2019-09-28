"""Representation of scavenger hunt worlds.

A world is a weighted, directed, complete graph and associated occurrence model.
The occurrence model describes the probability distribution of objects across
graph nodes.

It is important to distinguish between "objects" and "instances". An object is
a type of item that may appear in the world and on hunt lists. An object has a
unique identifying string called a label. An instance is intuitive--an instance
of an object. Instances are also identified with unique labels, ideally
including its object label and a numeric ID (e.g. the set of N chair instances
has labels {chair0, chair1, ..., chairN}. An instance may be viewed from one or
more world location. Multiple instances of the same object may appear across the
world. Hunt lists contain objects, never instances.
"""
import random
import util


class Distribution:
    """A probabalistic object distribution across locations.
    """
    def __init__(self, label, probs):
        """Should not be created directly; use Map.add_distr instead.

        Parameters
        ----------
        label : str
            unique object label
        probs : list
            list of the form [
                ([node00, ..., node0M], prob0),
                ([node10, ..., node1M], prob1),
                ...,
                ([nodeN0, ..., nodeNM], probN)
            ]
            with positive probabilities summing to 1.0
        """
        # Validate probability distribution
        sum = 0
        for item in probs:
            p = item[1]
            if p <= 0:
                raise RuntimeError("bad probability: %s" % p)
            sum += p
        if not util.approx(sum, 1.0):
            raise RuntimeError("bad probabilities")

        self.label = label
        self.probs = probs
        self.probs.sort(key=lambda x : x[1])

    def place(self):
        """Generate a location for the object to be at according to the
        distribution.

        Return
        ------
        list
            list of node names
        """
        f = random.random()
        locs = None
        for i in range(0, len(self.probs) - 1):
            p = self.probs[i]
            if f < p[1]:
                locs = p[0]
                break
        if locs is None:
            locs = self.probs[len(self.probs) - 1][0]
        return locs


class Node:
    """A location on a map, containing some set of objects.
    """
    def __init__(self, name):
        """
        Parameters
        ----------
        name : str
            unique name
        """
        self.name = name
        self.objects = []

    def __eq__(self, other):
        """
        Parameters
        ----------
        other : object
            rhs

        Return
        ------
        bool
            if other is a Node and shares my name
        """
        if not isinstance(other, self.__class__):
            return False
        return self.name == other.name

    def __str__(self):
        return self.name

    def __hash__(self):
        return hash(self.name)


class Edge:
    """A directed, weighted link between two nodes in a map.
    """
    def __init__(self, n0, n1, cost):
        """
        Parameters
        ----------
        n0 : str
            tail node name
        n1 : str
            head node name
        cost : float
            traversal cost
        """
        self.n0 = n0
        self.n1 = n1
        self.cost = cost


class Map:
    """A directed, weighted graph and mapping of nodes to object distributions.
    This is a comprehensive representation of the simulation world, minus the
    agent.
    """
    def __init__(self):
        """Maps are initially empty.
        """
        self.nodes = {}
        self.object_labels = {}
        self.edges = []
        self.connections = {}
        self.distributions = []

    def add_node(self, n):
        """Adds a new node to the map. This should not be called directly; use
        connect or one of its variants instead.

        Parameters
        ----------
        n : Node
            node to add
        """
        if not n.name in self.nodes:
            self.nodes[n.name] = n

    def connect(self, name0, name1, cost):
        """Adds a new edge to the map. This edge is directed name0 -> name1.

        Parameters
        ----------
        name0 : str
            name for tail node
        name1 : str
            name for head node
        cost: float
            traversal cost
        """
        n0, n1 = Node(name0), Node(name1)
        edge = Edge(name0, name1, cost)
        self.add_node(n0)
        self.add_node(n1)
        self.edges.append(edge)

    def connect_bidi(self, name0, name1, cost):
        """Adds a new edge to the map. This edge is bidirectional
        name0 <-> name1.

        Parameters
        ----------
        name0 : str
            name for tail node
        name1 : str
            name for head node
        cost: float
            traversal cost
        """
        self.connect(name0, name1, cost)
        self.connect(name1, name0, cost)

    def object_at(self, label, name):
        """Gets whether or not an object with some label is present at a node.

        Parameters
        ----------
        label : str
            object label
        name : str
            node name

        Return
        ------
        bool
            if the object is present
        """
        return label in self.nodes[name].objects

    def finalize(self):
        """Finalizes the map by building the adjacency map. Probably don't edit
        the map further after calling this.
        """
        for node in self.nodes:
            # Find all adjacent nodes
            self.connections[node] = []
            for edge in self.edges:
                if edge.n0 == node:
                    self.connections[node].append(edge.n1)
            # No adjacent nodes is bad
            if len(self.connections[node]) == 0:
                raise RuntimeError("isolated node: %s" % node)
        # Populate map
        self.populate()

    def add_distr(self, label, *args):
        """Adds an object to the map with some occurrence model.

        Parameters
        ----------
        d : Distribution
            object distribution
        """
        self.distributions.append(Distribution(label, list(args)))

    def add_distr_list(self, label, probs):
        self.distributions.append(Distribution(label, probs))

    def populate(self):
        """Distributes objects across the map according to the provided
        distributions.
        """
        # Clear current objects
        for node in self.nodes:
            self.nodes[node].objects.clear()
        # Redistribute
        for d in self.distributions:
            label, locs = d.label, d.place()
            for loc in locs:
                self.nodes[loc].objects.append(label)

    def __str__(self):
        """Gets a string representation of the map showing node connections.

        Return
        ------
        str
            to-string
        """
        res = ""
        # Find longest name
        longest_name, longest_len = None, None
        for node in self.nodes:
            if longest_len is None or len(node) > longest_len:
                longest_name = node
                longest_len = len(node)

        # Build formatted connections list
        for c in self.nodes:
            # Build list
            conns = self.connections[c]
            conns_f = "["
            for i in range(0, len(conns)):
                last = i == len(conns) - 1
                conns_f += conns[i] + (", " if not last else "")
            conns_f += "]"
            # Pad to correct length
            c_name = str(c)
            while len(c_name) < longest_len:
                c_name += " "
            res += c_name + " -> " + conns_f + "\n"
            # Show objects
            res += "  %s\n" % str(self.nodes[c].objects)
        return res
