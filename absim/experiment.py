import generate
import hunt
import numpy

# PRELIMINARY DATA WITH CONFIG
#
# NUM_EXPERIMENTS = 100
# NUM_TRIALS = 100
# AGENTS = ["random", "proximity", "greedy", "dynamic"]
# NODES_RANGE = [5, 7]
# COST_RANGE = [100, 300]
# OBJECTS_RANGE = [2, 5]
# OCCURRENCES_RANGE = [1, 3]
#
# dynamic won 100 of 100 experiments
# dynamic traveled 0.260590442080464 the dist of random on avg.
# dynamic traveled 0.6809178362207813 the dist of proximity on avg.
# dynamic traveled 0.7118388728770967 the dist of greedy on avg.


NUM_EXPERIMENTS = 10
NUM_TRIALS = 1000
AGENTS = ["optimal", "dynamic"] # ["dynamic", "random", "proximity", "greedy"]
NODES_RANGE = [5, 7]
COST_RANGE = [100, 300]
OBJECTS_RANGE = [2, 5]
OCCURRENCES_RANGE = [1, 3]

scores = {}
for agent in AGENTS:
    scores[agent] = []
wins = {}
for agent in AGENTS:
    wins[agent] = 0

for i in range(NUM_EXPERIMENTS):
    fname = "experiments/exp%s.dat" % i
    generate.generate(fname,
        NODES_RANGE, COST_RANGE, OBJECTS_RANGE, OCCURRENCES_RANGE
    )

    best_score = 0
    best_agent = None

    print("BEGIN EXPERIMENT %s OF %s" % (i + 1, NUM_EXPERIMENTS))

    agent_avgs = {}
    for agent in AGENTS:
        agent_avgs[agent] = 0

    for i in range(NUM_TRIALS):
        argv = [
            "hunt.py",
            fname,
            "-a",
            "AGENT",
            "-s",
            "-m"
        ]
        map, h, start_loc, params = hunt.parse(argv)
        map.populate()
        for agent in AGENTS:
            params["a"] = agent
            agent_avgs[agent] += hunt.simulate(map, h, start_loc, params)

    best_agent = None
    best_avg = 0
    for agent in agent_avgs:
        agent_avgs[agent] /= NUM_TRIALS
        print("%s: %s" % (agent, agent_avgs[agent]))
        if best_agent is None or agent_avgs[agent] < best_avg:
            best_agent = agent
            best_avg = agent_avgs[agent]

    wins[best_agent] += 1

    print("WINNER: %s" % best_agent)

print("dynamic won %s of %s experiments" % (wins["dynamic"], NUM_EXPERIMENTS))

# for agent in AGENTS:
#     if agent != "dynamic":
#         total_comp = 0
#         for i in range(len(scores[agent])):
#             comp = scores["dynamic"][i] / scores[agent][i]
#             total_comp += comp
#         fig = total_comp / NUM_EXPERIMENTS
#         print("dynamic traveled %s the dist of %s on avg." % (fig, agent))

for agent in AGENTS:
    dists = scores[agent]
    mean = numpy.mean(dists)
    stdev = numpy.std(dists)
    print("%s m=%s stdev=%s" % (agent, mean, stdev))
