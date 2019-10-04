"""Baseline algorithm--random search.
"""
import agent
import random


class RandomAgent(agent.Agent):
    @staticmethod
    def pick_uniform(conns):
        i = int(random.random() * len(conns))
        return conns[i]

    def run(self):
        """Travel to a random location. Every adjacent location has an equal
        probability of being chosen.
        """
        conns = self.map.connections[self.current_node]
        self.traverse(RandomAgent.pick_uniform(conns))
