import math


def complete_traverse(unvisited_nodes, all_paths, current_path=[]):
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
        complete_traverse(new_unvisited_nodes, all_paths, new_path)


def dijkstra(map, start, end):
    """Dijkstra's shortest path algo for world.Map.

    Parameters
    ----------
    map : world.Map
        finalized world map
    start : str
        name of start node
    end : str
        name of end node

    Return
    ------
    list
        path as a list of node names beginning with start and ending with end
    """
    # List of unvisited nodes
    q = []

    # Set initial costs and previous nodes
    for v in map.nodes:
        map.nodes[v].dist = math.inf
        map.nodes[v].prev = None
        q.append(v)

    map.nodes[start].dist = 0

    # Loop until all unvisited nodes have been considered or the end has been
    # reached
    while len(q) > 0:
        # Identify lowest cost unvisited node
        min_cost = math.inf
        u = None
        for v in q:
            cost = map.nodes[v].dist
            if u is None or cost < min_cost:
                u = v
                min_cost = cost

        q.remove(u)

        # If the currently considered node is the goal, we're done
        if u == end:
            break

        # Update costs and planned path
        for v in map.connections[u]:
            alt = map.nodes[u].dist + map.cost(u, v)
            if alt < map.nodes[v].dist:
                map.nodes[v].dist = alt
                map.nodes[v].prev = u

    # Reconstruct path from prev flags
    path = [end]
    node = end
    while node is not None:
        prev = map.nodes[node].prev
        if prev is not None:
            path.insert(0, prev)
        node = prev

    return path
