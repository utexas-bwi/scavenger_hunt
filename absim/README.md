# Abstract Simulator

## Creating a World File

### Map Section [map]
Defines the world graph. Note that `DynamicAgent` requires a complete graph at
the moment.

The graph is defined as a list of edges. An edge `n1 n2 x !` defines an edge
from a node named `n1` to a node named `n2` with cost `x`. The `!` indicates
the edge is bidirectional.

A triangular map might look like this:

```
[map]
a b 10 !
a c 8 !
b c 5 !
```

### Start Section [start]
This section contains only the name of the starting node that the simulated
agent begins at, e.g.

```
[start]
a
```

### Objects Section [obj]
A list of instance IDs and their object labels. Instances may not share IDs,
but they may share object labels.

```
[obj]
chair0 chair
chair1 chair
plant0 plant
```

### Distribution Section [distr]
Defines which instances appear where and with what probability. Each instance
enumerated in the objects section should have its own line in this section.

A line of the form `inst loc0 p0 loc1 p1` states that the instance `inst`
appears at graph node `loc0` (`p0`\*100)% of the time and at `loc1` (`p1`\*100)%
of the time. These probabilities must follow the definition of probability.
There may be multiple locations associated with an appearance, e.g.
`inst0 loc0 loc1 0.45 loc1 0.55`. Probabilities may also appear in fraction
form `a/b`.

```
[distr]
chair0 a 0.5 b 0.5
chair1 a c 0.25 b 0.75
plant0 b 1.0
```

### Hunt Section [hunt]
The list of objects the agent must find. These are object labels and not
instance names.

```
[hunt]
chair
plant
```

## Creating an Algorithm

Scavenger hunt algorithms extend `agent.Agent`. The `run` method is overridden
with iterative logic and is called until the hunt is complete. Methods `setup`
and `epoch` may be optionally overridden and will run once per scavenger hunt
and once per program, respectively.

Each `run` call should result in a single call to `agent.traverse(dest)` to
travel to the node `dest`. Agents collect objects at the current node before
moving to the destination node. Calling `agent.traverse(None)` will collect
objects at the current node without moving anywhere. This method returns the
number of objects collected.

After implementing a new algorithm, integrate it into hunt.py by importing it
at the top of the file and mapping it to a name in `agent_lookup` on hunt.py:13.

## Running Simulations

Run simulations with
`python3 hunt.py [WORLD FILE] -t [NUMBER OF TRIALS] -a [AGENT NAME]`. The number
of trials defaults to 1. The agent name may not be omitted.
