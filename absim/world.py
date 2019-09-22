"""Objects for representing scavenger hunt worlds.
"""
import random
import util


class Distribution:
    """A probabalistic object distribution across locations.
    """
    def __init__(self, label, probs):
        """Should not be called directly; use world.distr instead.

        Parameters
        ----------
        label : str
            unique object label
        probs : list
            list of the form [
                (node_name0, prob0),
                (node_name1, prob1),
                ...
                (node_nameN, probN)
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
        str
            name of node
        """
        f = random.random()
        loc = None
        for i in range(0, len(self.probs) - 1):
            p = self.probs[i]
            if f < p[1]:
                loc = p[0]
                break
        if loc is None:
            loc = self.probs[len(self.probs) - 1][0]
        return loc


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
        n0 : Node
            tail node
        n1 : Node
            head node
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
        self.nodes = []
        self.node_name_map = {}
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
        if not n in self.nodes:
            self.node_name_map[n.name] = n
            self.nodes.append(n)

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
        edge = Edge(n0, n1, cost)
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
        node = None
        for n in self.nodes:
            if n.name == name:
                node = n
                break
        if node is None:
            raise RuntimeError("unknown node: %s" % node)
        return label in node.objects

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

    def populate(self):
        """Distributes objects across the map according to the provided
        distributions.
        """
        # Clear current objects
        for node in self.nodes:
            node.objects.clear()
        # Redistribute
        for d in self.distributions:
            label, loc = d.label, d.place()
            self.node_name_map[loc].objects.append(label)

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
            if longest_len is None or len(node.name) > longest_len:
                longest_name = node.name
                longest_len = len(node.name)

        # Build formatted connections list
        for c in self.nodes:
            # Build list
            conns = self.connections[c]
            conns_f = "["
            for i in range(0, len(conns)):
                last = i == len(conns) - 1
                conns_f += conns[i].name + (", " if not last else "")
            conns_f += "]"
            # Pad to correct length
            c_name = str(c)
            while len(c_name) < longest_len:
                c_name += " "
            res += c_name + " -> " + conns_f + "\n"
            # Show objects
            res += "  %s\n" % str(c.objects)
        return res
