import generate
import hunt

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
# dynamic traveled 0.23621295811499898 the dist of random on avg.
# dynamic traveled 0.6252361125098285 the dist of proximity on avg.
# dynamic traveled 0.6483475428500702 the dist of greedy on avg.


NUM_EXPERIMENTS = 100
NUM_TRIALS = 100
AGENTS = ["random", "proximity", "greedy", "dynamic"]

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

    for agent in AGENTS:
        argv = [
            "hunt.py",
            fname,
            "-t",
            str(NUM_TRIALS),
            "-a",
            agent,
            "-s"
        ]
        map, h, start_loc, params = hunt.parse(argv)
        avg_dist = hunt.simulate(map, h, start_loc, params)
        scores[agent].append(avg_dist)

        if best_agent is None or avg_dist < best_score:
            best_score = avg_dist
            best_agent = agent

    wins[agent] += 1

    print("finished experiment %s of %s" % (i, NUM_EXPERIMENTS))

print("dynamic won %s of %s experiments" % (wins["dynamic"], NUM_EXPERIMENTS))

for agent in AGENTS:
    if agent != "dynamic":
        total_comp = 0
        for i in range(len(scores[agent])):
            comp = scores["dynamic"][i] / scores[agent][i]
            total_comp += comp
        fig = total_comp / NUM_EXPERIMENTS
        print("dynamic traveled %s the dist of %s on avg." % (fig, agent))
