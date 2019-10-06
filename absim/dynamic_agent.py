import agent
import math


def all_distributions(map):
    """ Generates a list of the form
    [
        [(inst00, locs00), (inst01, locs01), ..., (inst0N, locs0N)],
        [(inst10, locs10), (inst11, locs11), ..., (inst1N, locs1N)],
        ...,
        [(instM0, locsM0), (instM1, locsM1), ..., (instMN, locsMN)]
    ]
    which represents all possible instance arrangements on a map. Each member of
    the topmost list is a different possible arrangement of instances, that is,
    a list of all instances paired with the locations from which they can be
    seen in that particular arrangement. These arrangements are collectively
    exhaustive and mutually exclusive.

    Parameters
    ----------
    map : world.Map
        finalized map

    Return
    ------
    list
        all possible object arrangements
    """
    distr0 = map.distributions[0]
    distrs = [list() for i in range(len(distr0.probs))]


    distrs = [list() for i in range(len(map.distributions))]
    for i in range(0, len(map.distributions)):
        inst_i = map.distributions[i]
        for event_i in inst_i.probs:
            occur_i = (inst_i.label, event_i[0])
            distrs[i].push_back(occur_i)
            for j in range(0, len(map.distributions)):
                if j != i:
                    inst_j = map.distributions[j]
                    for event_j in inst_j.probs:
                        occur_j = (inst_j.label, event_j[0])
                    occur_j = (inst_j.label, )



class DynamicAgent(agent.Agent):
    def run(self):
        unvisited = [loc for loc in self.map.nodes if loc not in self.visited]
