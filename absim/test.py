from random_agent import RandomAgent

import agent
import world


################################################################################
# WORLD SETUP
################################################################################
m = world.Map()
m.connect_bidi("a", "b", 10)
m.connect_bidi("b", "c", 5)
m.connect_bidi("c", "a", 4)
m.add_distr("box",
    ("a", 0.5),
    ("b", 0.4),
    ("c", 0.1)
)
m.add_distr("chair",
    ("a", 0.05),
    ("b", 0.75),
    ("c", 0.2)
)
m.finalize()

################################################################################
# AGENT
################################################################################
a = RandomAgent(m, ["box", "chair"], m.node_name_map["a"])

while not a.is_done():
    a.run()

print(a.travel_distance)
