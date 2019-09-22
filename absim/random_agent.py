import agent
import random


class RandomAgent(agent.Agent):
    def run(self):
        conns = self.map.connections[self.current_node]
        i = int(random.random() * len(conns))
        self.traverse(conns[i])
