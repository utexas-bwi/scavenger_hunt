"""Discrete structures that comprise the scavenger hunt problem.

In general:
    * Locations (loc, locs) are nodes on a world.Graph with N nodes. They are
      represented by integers in the range [0, N).
    * Objects (obj, objs) are objects that appear in scavenger hunts. They are
      reresented by strings. The string representing an object is also its
      label; there is no concept of instances as there was in the original
      absim.
"""
import copy
import math
import random
import util as util


class Path:
    """A sequence of nodes with an associated cost.
    """
    def __init__(self, nodes, cost):
        self.nodes = nodes
        self.cost = cost


class Graph:
    """A weighted graph representing a world map.
    """
    def __init__(self, node_count):
        """Initializes a completely disconnected graph with NODE_COUNT nodes.
        """
        self.node_count = node_count
        self.nodes = [i for i in range(self.node_count)]
        self.name_ids = {}
        self.adj_mat = {}
        self.conns = {}
        self.finalized = False
        for i in range(node_count):
            self.adj_mat[i] = [None for j in range(node_count)]

    def connect(self, a, b, cost):
        """Adds an edge connecting A to B with cost COST.
        """
        assert not self.finalized
        self.adj_mat[a][b] = cost
        self.adj_mat[b][a] = cost

    def finalize(self):
        """Finalizes the graph by building the connection maps and computing all
        shortest paths. Graph becomes essentially immutable.
        """
        assert not self.finalized
        self.finalized = True
        # Generate connection mappings
        for i in range(self.node_count):
            self.conns[i] = []
            for j in range(self.node_count):
                if i != j and self.cost(i, j) is not None:
                    self.conns[i].append(j)
        # Generate all shortest paths
        self.shortest_paths = {}
        for n_from in range(self.node_count):
            for n_to in range(n_from + 1, self.node_count):
                path = self.find_shortest_path(n_from, n_to)
                key0 = str(n_from) + str(n_to)
                key1 = str(n_to) + str(n_from)
                self.shortest_paths[key0] = path
                path_rev = Path(path.nodes.copy(), path.cost)
                path_rev.nodes.reverse()
                self.shortest_paths[key1] = path_rev

    def cost(self, a, b):
        """Returns the cost of the edge connecting A and B. If no such edge
        exists, None is returned.
        """
        assert self.finalized
        if isinstance(a, str):
            a = self.name_ids[a]
        if isinstance(b, str):
            b = self.name_ids[b]
        return self.adj_mat[a][b]

    def find_shortest_path(self, start, end):
        """Computes the shortest world.Path between START and END. This should
        not be called manually.
        """
        assert self.finalized

        # Set initial prev and dist flags
        q = []
        dist = {}
        prev = {}
        for v in self.nodes:
            dist[v] = math.inf
            prev[v] = None
            q.append(v)

        dist[start] = 0

        # While there are still unvisited nodes
        while len(q) > 0:
            # Identify lowest cost unvisited node
            min_cost = math.inf
            u = None
            for v in q:
                cost = dist[v]
                if u is None or cost < min_cost:
                    u = v
                    min_cost = cost

            q.remove(u)

            # If the currently considered node is the goal, we're done
            if u == end:
                break

            # Update costs and planned path
            for v in self.conns[u]:
                alt = dist[u] + self.cost(u, v)
                if alt < dist[v]:
                    dist[v] = alt
                    prev[v] = u

        # Reconstruct path from prev flags
        path = [end]
        node = end
        while node is not None:
            p = prev[node]
            if p is not None:
                path.insert(0, p)
            node = p

        return Path(path, dist[end])

    def shortest_path(self, a, b):
        """Gets the shortest world.Path between A and B.
        """
        assert self.finalized
        key = str(a) + str(b)
        return self.shortest_paths[key]

    def salesman_paths_rec(self, loc_c, locs, current_path, all_paths, forks):
        """Recursive helper for salesman_paths(). Do not call.
        """
        current_path.append(loc_c)
        conns = self.conns[loc_c]
        new_conns = [c for c in conns if c not in current_path]
        # Dead end reached
        if len(new_conns) == 0:
            # If full map explored, we're done
            unvisited = [c for c in locs if c not in current_path]
            if len(unvisited) == 0 and current_path not in all_paths:
                all_paths.append(current_path)
                return
            # Otherwise, we backtrack and try different decisions at prev forks
            else:
                # For each fork
                for fork in forks:
                    # Retrace our steps to that fork, adding to our path
                    steps = current_path.copy()[:len(current_path) - 1]
                    current_path_new = current_path.copy()
                    while steps[len(steps) - 1] != fork:
                        current_path_new.append(steps.pop())
                    # This branch of recursion takes unexplored branches w/ it
                    forks_new = forks.copy()
                    forks_new.remove(fork)
                    # Rerun search with path including backtracing
                    self.salesman_paths_rec(
                        fork, locs, current_path_new, all_paths, forks_new
                    )
        # If we made a decision here, make a note so we come back and try others
        elif len(new_conns) > 1:
            forks.append(loc_c)
        # Explore new branches
        for conn in new_conns:
            self.salesman_paths_rec(
                conn, locs, current_path.copy(), all_paths, forks.copy()
            )
        # Cycles in the graph may give us shortcuts to previous forks
        for fork in forks:
            if self.cost(loc_c, fork) is not None:
                forks_new = forks.copy()
                forks_new.remove(fork)
                self.salesman_paths_rec(
                    fork, locs, current_path.copy(), all_paths, forks_new
                )

    def salesman_paths(self, loc):
        """Gets all paths beginning at LOC and passing through all other locs.

        NOTE: This implementation is incomplete. There is currently a bug that
        causes the same path to be discovered multiple times, requiring that we
        do an O(N^2) check to see if we've already found it. Additionally, the
        definition of a "salesman path" is somewhat nebulous for an incomplete
        graph, since the agent can visit every node with an arbitrary amount of
        backtracking. Search algorithms that consider all paths should use
        permute_paths() instead of this method.
        """
        assert self.finalized
        all_paths = []
        self.salesman_paths_rec(loc, self.nodes, [], all_paths, [])
        return all_paths

    def permute_paths_rec(self, locs, current_path, all_paths):
        """Recursive helper for permute_paths(). Do not call.
        """
        if len(locs) == 0:
            all_paths.append(current_path)
            return
        for loc in locs:
            new_locs = locs.copy()
            new_locs.remove(loc)
            new_current_path = current_path.copy()
            new_current_path.append(loc)
            self.permute_paths_rec(new_locs, new_current_path, all_paths)

    def permute_paths(self, loc):
        """Gets all permutations of locs starting at LOC.
        """
        assert self.finalized
        all_paths = []
        traverse_locs = self.nodes.copy()
        traverse_locs.remove(loc)
        self.permute_paths_rec(traverse_locs, [loc], all_paths)
        return all_paths

    def path_cost(self, path):
        """Gets the cost of a path in list form.
        """
        assert self.finalized
        cost = 0
        for i in range(1, len(path)):
            cost += self.shortest_path(path[i-1], path[i]).cost
        return cost

    def stitch_path(self, path):
        """Creates a valid path of nodes not necessarily adjacent by stitching
        together Dijkstra paths.
        """
        assert self.finalized
        stitched = []
        for i in range(1, len(path)):
            n_to, n_from = path[i], path[i-1]
            inter_path = self.shortest_path(n_from, n_to).nodes
            start_idx = 1 if i > 1 else 0
            stitched.extend(inter_path[start_idx:])
        return stitched


class Event:
    """An event as defined by the scavenger hunt problem.
    """
    def __init__(self, obj, locs, prob):
        """OBJ appears at locations in LOCS simultaneously PROB percent of the
        time.
        """
        self.obj = obj
        self.locs = locs.copy()
        self.prob = prob

    def __str__(self):
        return "%s@%s:%s" % (self.obj, str(self.locs), self.prob)


class Distribution:
    """A distribution as defined by the scavnger hunt problem.
    """
    def __init__(self, events):
        """The distribution is comprised of EVENTS. The events therein are
        collectively exhaustive, mutually exclusive, and describe the same
        object.
        """
        for i in range(1, len(events)):
            assert(events[i].obj == events[0].obj)

        self.obj = events[0].obj
        self.events = events
        self.events.sort(key=lambda e : e.prob)

        prob = 0
        for event in self.events:
            prob += event.prob

        assert util.approx(prob, 1.0)

    def place(self):
        """Generates a random location for the described object to appear in
        according to the distribution's events.
        """
        f = random.random()
        event = None
        for i in range(len(self.events) - 1):
            p = self.events[i].prob
            if f < p:
                event = self.events[i]
                break
            else:
                f -= p
        if event is None:
            event = self.events[len(self.events) - 1]
        return event


class Arrangement:
    """An arrangement as defined by the scavenger hunt problem.
    """
    def __init__(self, events):
        """The arrangement is comprised of EVENTS. Each event should describe
        a different object.
        """
        self.events = events
        self.objs = [event.obj for event in events]

    def prob(self):
        """Returns the probability of this arrangement occurring.
        """
        prob = 1
        for event in self.events:
            prob *= event.prob
        return prob

    def contains(self, obj, loc):
        """Returns whether or not OBJ is at LOC in this arrangement.
        """
        for event in self.events:
            if loc in event.locs and event.obj == obj:
                return True
        return False

    def __str__(self):
        return "<" + ", ".join([str(e) for e in self.events]) + ">"


class ArrangementSpace:
    """An arrangement space as defined by the scavenger hunt problem. All
    sampling methods (i.e. prob_obj()) reflect observations of the space made so
    far.
    """
    def __init__(self, world):
        """Arrangement spaces are automatically generated from a finalized
        WORLD.
        """
        self.world = world
        self.arrangements = []
        self.valid_arrangements = []
        self.p_event_exceptions = {}

        root = ([], None)
        leaves = [root]

        # Build event tree
        for distr in world.distrs:
            new_leaves = []
            for event in distr.events:
                for leaf in leaves:
                    node = ([], copy.deepcopy(event))
                    leaf[0].append(node)
                    new_leaves.append(node)
            leaves = new_leaves.copy()

        # Generate all paths from root to leaf
        def tree_depth_traverse(root, all_paths, current_path):
            current_path.append(root)
            if len(root[0]) == 0:
                all_paths.append(current_path)
                return
            for child in root[0]:
                tree_depth_traverse(child, all_paths,
                                    copy.deepcopy(current_path))

        all_paths = []
        tree_depth_traverse(root, all_paths, [])

        # Extract arrangements from tree paths
        for path in all_paths:
            events = []
            for i in range(1, len(path)):
                events.append(copy.deepcopy(path[i][1]))
            self.arrangements.append(Arrangement(events))
            self.valid_arrangements.append(True)

    def prob_obj(self, obj, loc):
        """Returns the probability of OBJ occurring at LOC.
        """
        key = obj + str(loc)
        return self.world.prob_obj(obj, loc) if \
               key not in self.p_event_exceptions else \
               self.p_event_exceptions[key]

    def pot_objs_at(self, loc):
        """Returns a list of objects that may potentially appear at LOC.
        """
        objs = []
        for i in range(len(self.arrangements)):
            if self.valid_arrangements[i]:
                for event in self.arrangements[i].events:
                    if loc in event.locs and event.obj not in objs:
                        objs.append(event.obj)
        return objs

    def prob_any_obj(self, loc):
        """Returns the probability of any object occurring at LOC.
        """
        prob_no_obj = 1
        # The probability of finding any object is one minus the product of all
        # the probabilities of the objects not appearing at this location
        for pot_obj in self.pot_objs_at(loc):
            prob_no_obj *= 1 - self.prob_obj(pot_obj, loc)

        return 1 - prob_no_obj

    def update_prob(self, obj, loc, p):
        """Changes the probability of OBJ appearing at LOC to p. Should not be
        called manually.
        """
        self.p_event_exceptions[obj + str(loc)] = p

    def observe(self, obj, loc, seen):
        """Makes an observation of the world that changes the arrangement space.
        The arrangement space is updated to reflect OBJ occurring at LOC if SEEN
        and OBJ not occurring at LOC otherwise.
        """
        # Invalidate arrangements that are precluded by this observation
        for i in range(len(self.arrangements)):
            if self.valid_arrangements[i]:
                for event in self.arrangements[i].events:
                    # If this event contradicts the observation
                    if event.obj == obj and (loc in event.locs) != seen:
                        self.valid_arrangements[i] = False

        if seen:
            # If obj was observed at loc, mark the probability of it occurring
            # elsewhere as 0
            for other_loc in self.world.graph.nodes:
                if loc != other_loc:
                    self.update_prob(obj, other_loc, 0.0)
            # Update all events involving obj occurring at loc to have
            # probability 1
            for i in range(len(self.arrangements)):
                if self.valid_arrangements[i]:
                    for event in self.arrangements[i].events:
                        if event.obj == obj and loc in event.locs:
                            event.prob = 1
                            for other_loc in event.locs:
                                self.update_prob(obj, other_loc, 1.0)
        else:
            # If obj was not observed at loc, mark the probability of it
            # occurring at loc as 0
            p_obj_loc = self.prob_obj(obj, loc)
            self.update_prob(obj, loc, 0.0)
            # Update all events involving obj not occurring at loc to have
            # increased probability
            for i in range(len(self.arrangements)):
                if self.valid_arrangements[i]:
                    # For each remaining valid event concerning this object
                    for event in self.arrangements[i].events:
                        if event.obj == obj:
                            # Increase its probability
                            event.prob = event.prob / (1 - p_obj_loc)
                            for other_loc in event.locs:
                                self.update_prob(obj, other_loc, event.prob)

        # Assert that the space remains collectively exhaustive and mutually
        # exclusive
        total_prob = 0
        for i in range(len(self.arrangements)):
            if self.valid_arrangements[i]:
                total_prob += self.arrangements[i].prob()
        assert util.approx(total_prob, 1.0)


class World:
    """A graph map and set of distributions completely describing a scavenger
    hunt problem. A world represents what is known at the beginning of the
    scavenger hunt, before any observations are made. This information is static
    throughout scavenger hunts. To change information, i.e. use Bayesian search
    logic in scavenger hunt algorithms, see ArrangementSpace.
    """
    def __init__(self, graph, distrs):
        """Defines a world with map GRAPH and object distributions DISTRS. The
        world is not initially finalized, though there is little reason not to
        finalize() it immediately after creation.
        """
        self.graph = graph  # World map
        self.distrs = distrs  # Distributions for all object
        self.objs = [distr.obj for distr in distrs]  # List of all objects
        self.probs = {}  # (obj, loc) -> P of finding obj at loc
        self.pot_objs = {}  # Loc -> potential objects to find at loc
        self.finalized = False
        self.arrangement_override = False

    def populate(self):
        """Creates a random internal arrangement according to the world's object
        distributions. This arrangement should not be accessed by scavenger
        hunt algorithms (with the exception of offline optimal).
        """
        # Don't populate if the user opted to override the arrangement
        if self.arrangement_override:
            return

        assert self.finalized
        self.arrangement = []  # Clear arrangement
        self.occurrences = {}  # Clear occurrences
        # Populate occurrences map with empty lists
        for loc in self.graph.nodes:
            self.occurrences[loc] = []
        # For each distribution
        for distr in self.distrs:
            # Sample the distribution and place the object
            event = distr.place()
            self.arrangement.append(event)
            # Update the occurrences map
            for loc in event.locs:
                self.occurrences[self.node_id(loc)].append(event.obj)

    def objs_at(self, loc):
        """Returns a list of objects present at LOC in the current arrangement.
        """
        assert self.finalized
        return self.occurrences[self.node_id(loc)].copy()

    def pot_objs_at(self, loc):
        """Returns a list of the objects that can appear at LOC.
        """
        assert self.finalized
        return self.pot_objs[loc]

    def cost(self, a, b):
        """Returns the cost of the edge from A to B or None if no such edge
        exists.
        """
        assert self.finalized
        return self.graph.cost(a, b)

    def node_id(self, node):
        """Returns the integer ID of the node named NODE.
        """
        if isinstance(node, str):
            return self.graph.name_ids[node]
        return node

    def finalize(self):
        """Finalizes the world by building constant-time lookup maps for
        occurrence probabilities and objects that may potentially be found at
        locations.
        """
        assert not self.finalized
        # Build the probability map
        for distr in self.distrs:
            for event in distr.events:
                for loc in event.locs:
                    self.probs[event.obj + str(self.node_id(loc))] = event.prob
        # Build the lists of all potential finds at all locations
        for loc in self.graph.nodes:
            self.pot_objs[loc] = []
        for distr in self.distrs:
            for event in distr.events:
                for loc in event.locs:
                    if distr.obj not in self.pot_objs[self.node_id(loc)]:
                        self.pot_objs[self.node_id(loc)].append(distr.obj)
        self.finalized = True

    def conns(self, loc):
        """Gets a list of the locations adjacent to LOC.
        """
        assert self.finalized
        return self.graph.conns[loc]

    def prob_obj(self, obj, loc):
        """Returns probability of OBJ occurring at LOC.
        """
        assert self.finalized
        key = obj + str(self.node_id(loc))
        return self.probs[key] if key in self.probs else 0
