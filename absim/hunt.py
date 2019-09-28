from random_agent import RandomAgent

import agent
import sys
import world


# Enforce usage
if len(sys.argv) < 2:
    print("Usage: python3 test.py <hunt file> <flags>")
    sys.exit(0)


# Parse parameters
params = {}

for i in range(2, len(sys.argv)):
    arg = sys.argv[i]
    if arg[0] == '-':
        value_follows = i + 1 < len(sys.argv) and sys.argv[i + 1][0] != '-'
        value = sys.argv[i + 1] if value_follows else None
        params[arg[1:]] = value


map = world.Map()             # Game world
hunt = []                     # Scavenger hunt
src = open(sys.argv[1], "r")  # Config file
sec = None                    # Section currently being parsed

# Parse config from hunt file
for line in src.readlines():
    line = line.strip()
    if len(line) == 0 or line[0] == '#':
        continue

    # Entering new section
    if line[0] == '[':
        sec = line[1:line.find(']')]
    # Building map
    elif sec == "map":
        args = line.split()
        bidi = len(args) > 3 and args[3] == '!'
        cost = float(args[2])
        if bidi:
            map.connect_bidi(args[0], args[1], cost)
        else:
            map.connect(args[0], args[1], cost)
    # Object identities
    elif sec == "obj":
        args = line.split()
        map.object_labels[args[0]] = args[1]
    # Object distributions
    elif sec == "distr":
        args = line.split()
        obj = args[0]
        events = []
        ind = 1

        while ind < len(args):
            locs = []  # List of locations for this event
            prob_ind = ind  # Index of event probability
            while args[prob_ind] in map.nodes:
                locs.append(args[prob_ind])  # This was a location; add it
                prob_ind += 1
            prob_arg = args[prob_ind]
            # Parse probability argument, which may be in fraction form
            if '/' in prob_arg:
                frac = prob_arg.split('/')
                prob = float(frac[0]) / float(frac[1])
            else:
                prob = float(prob_arg)
            events.append((locs, prob))
            ind = prob_ind + 1

        map.add_distr_list(obj, events)
    # Hunt object list
    elif sec == "hunt":
        hunt.append(line)

src.close()
map.finalize()

print(map)

################################################################################
# AGENT
################################################################################
a = RandomAgent(map, ["box", "chair"], "a")

while not a.is_done():
    a.run()

print(a.travel_distance)
