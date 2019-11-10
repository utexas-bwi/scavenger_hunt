# Abstract Simulator 2

## Creating a World File

World files, usually ending in `.dat`, define a scavenger hunt problem. The
simulator takes a world file as input.

### Map Section [map]

Defines the world graph as a list of edges. An edge `n1 n2 x` defines a
bidirectional edge between nodes `n1` and `n2` with weight `x`. The list
of nodes need not be explicitly enumerated anywhere.

A triangular map might look like this:

```
[map]
a* b 10
a c 8
b c 5
```

Note the asterisk by node `a`. This indicates that scavenger hunt agents start
their hunts at this node. There must be exactly one asterisk in the map section.

### Object Distribution Section [distr]

Defines which objects appear where and with what probability. The scavenger hunt
need not be explicitly enumerated anywhere; if an object appears in this
section, it is included in the hunt. There is no concept of instances as there
was in the original absim.

A line of the form `obj0 loc0 p0 loc1 p1` states that an object named `obj0`
appears at graph node `loc0` (`p0`\*100)% of the time and at `loc1` (`p1`\*100)%
of the time. These probabilities must follow the definition of probability.
There may be multiple locations associated with an appearance, e.g.
`obj1 loc0 loc1 0.45` states that `obj1` can be seen from `loc0` and `loc1`
simultaneously 45% of the time. Be aware that most agent logic has not been
tested in such scenarios, as we are largely uninterested in them for our
experiment.

Probabilities may also appear in fraction form `a/b`.

```
[distr]
box a 0.5 b 0.5
plant b 1/4 c 3/4
robot b 1.0
```

## Creating an Algorithm

Scavenger hunt algorithms extend `agent.Agent`. The `run` method is overridden
with iterative logic and is called by the simulator until the hunt is complete.
Optional overrides `setup` and `epoch` contain once per hunt and once per
simulation logic, respectively. When overriding agent methods, always call the
parent's implementation on the first line of the override.

Each call to `run` shall result in exactly one call to `go(dest)` to visit some
node `dest`. Agents automatically collect objects as they travel.

Some useful methods for agents:
* `self.loc` - the current location
* `self.visited(loc)` - gets if the agent has visited `loc`
* `self.go(loc)` - moves the agent to location `loc`; must be adjacent
* `self.world.cost(a, b)` - gets the edge weight from `a` to `b`
* `self.world.graph.shortest_path(a, b)` - gets the shortest `world.Path` from
   `a` to `b`
* `self.arrangement_space.pot_objs_at(loc)` - gets a list of the unfound objects
   that may potentially be found at `loc`
* `self.arrangement_space.prob_obj(obj, loc)` - gets the probability of finding
  `obj` at `loc`

Note that the arrangement space is updated automatically as agents make
observations about where objects are or aren't. All information derived from
arrangement space methods reflect these observations.

After implementing a new algorithm, integrate it into hunt.py by importing it
at the top of the file and mapping a name to its constructor in `agent_lookup`,
in the body of `simulate`.

## Running Simulations

Run simulations with
`python3 hunt.py [-h] [-t TRIALS] [-s] [-r] datfile agent`. `datfile` is the
world file and `agent` is the name of the algorithm to run.

See `python3 hunt.py -h` for details about the optional flags.
