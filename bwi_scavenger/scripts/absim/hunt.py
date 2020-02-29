import argparse
import time
import absim.world as world

from absim.bayesian_agent import BayesianAgent
from absim.prob_agent import ProbAgent
from absim.prox_agent import ProxAgent
from absim.prob_prox_agent import ProbProxAgent
from absim.salesman_agent import SalesmanAgent


agent_lookup = {
    "prob" : ProbAgent,
    "prox" : ProxAgent,
    "prob_prox" : ProbProxAgent,
    "salesman" : SalesmanAgent,
    "bayes" : BayesianAgent
}


def parse_world(fname):
    """Parses a scavenger hunt world from a datfile.

    Parameters
    ----------
    fname : str
        source file name

    Returns
    -------
    world.World
        finalized scavenger hunt world
    list of str
        scavenger hunt
    str
        start location name
    """
    src = open(fname, "r")
    sec = None
    start_loc = None
    conns = []  # 3-tuples (from, to, cost)
    nodes = {}  # User-specified loc names -> integer IDs
    node_count = 0
    distrs = []
    hunt = []

    for line in src.readlines():
        # Blank lines and comments
        line = line.strip()
        if len(line) == 0 or line[0] == '#':
            continue

        # Section header line
        if line[0] == '[':
            sec = line[1:line.find(']')]
            continue

        # Map section
        if sec == "map":
            args = line.split()
            assert len(args) == 3
            n_from, n_to = args[0], args[1]

            # Parse for starting location
            if '*' in n_from:
                n_from = n_from.replace('*', '')
                if start_loc is None:
                    start_loc = n_from
            elif '*' in n_to:
                n_to = n_to.replace('*', '')
                if start_loc is None:
                    start_loc = n_to

            cost = float(args[2])
            if n_from not in nodes:
                nodes[n_from] = node_count
                node_count += 1
            if n_to not in nodes:
                nodes[n_to] = node_count
                node_count += 1
            conns.append((n_from, n_to, cost))
        # Distribution section
        elif sec == "distr":
            args = line.split()
            assert len(args) > 2
            obj = args[0]
            events = []
            ind = 1

            if obj not in hunt:
                hunt.append(obj)

            while ind < len(args):
                locs = []
                prob_ind = ind
                while args[prob_ind] in nodes:
                    locs.append(nodes[args[prob_ind]])
                    prob_ind += 1
                prob_arg = args[prob_ind]
                if '/' in prob_arg:
                    frac = prob_arg.split('/')
                    prob = float(frac[0]) / float(frac[1])
                else:
                    prob = float(prob_arg)
                events.append(world.Event(obj, locs, prob))
                ind = prob_ind + 1

            distrs.append(world.Distribution(events))
        else:
            assert False

    src.close()

    # Build graph
    g = world.Graph(node_count)
    for conn in conns:
        id_from, id_to, cost = nodes[conn[0]], nodes[conn[1]], conn[2]
        g.connect(id_from, id_to, cost)
        g.name_ids[conn[0]] = id_from
        g.name_ids[conn[1]] = id_to
    g.finalize()

    # Build world
    w = world.World(g, distrs)
    w.finalize()

    return w, hunt, start_loc


def simulate(world, hunt, start_loc, args):
    """Runs one or more scavenger hunts.

    Parameters
    ----------
    world : world.World
        scavenger hunt world
    hunt : list of str
        objects to find
    start_loc : str
        starting node name
    args : Namespace
        cmdline args parsed by argparse

    Returns
    -------
    float
        average distance traveled across all trials
    """
    total_distance = 0
    total_runtime = 0
    trials = args.trials
    agent = agent_lookup[args.agent](world, hunt, world.node_id(start_loc))
    agent.epoch()

    if not args.suppress:
        print(">>> Running %s trials of %s" % \
            (trials, agent.__class__.__name__))

    for i in range(trials):
        t_start, t_end = None, None
        world.populate()
        agent.setup()
        t_start = time.time()
        while not agent.done():
            agent.objs_at_loc = world.objs_at(agent.loc)
            agent.run()
        t_end = time.time()
        total_distance += agent.travel_distance
        total_runtime += t_end - t_start
        if not args.suppress:
            print("Progress: {:2.1%}".format(i / trials), end="\r")

    avg_distance = total_distance / trials
    avg_runtime = total_runtime / trials
    if not args.suppress:
        print("Average distance: %s" % avg_distance)
        if args.runtime:
            print("Average runtime: %ss" % avg_runtime)

    return avg_distance


if __name__ == "__main__":
    # Parse cmdline args
    ap = argparse.ArgumentParser()
    ap.add_argument("datfile", help="scavenger hunt world file", type=str)
    ap.add_argument("agent", help="algorithm to run", type=str)
    ap.add_argument("-t", "--trials", help="number of trials to run", type=int,
        default=1)
    ap.add_argument("-s", "--suppress", help="silence output",
        action='store_true')
    ap.add_argument("-r", "--runtime", help="show average runtime",
        action='store_true')
    args = ap.parse_args()

    # Generate scavenger hunt problem and simulate
    world, hunt, start_loc = parse_world(args.datfile)
    simulate(world, hunt, start_loc, args)
