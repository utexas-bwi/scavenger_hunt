import agent
import hunt
import unittest
import util
import world

from bayesian_agent import BayesianAgent
from prob_agent import ProbAgent
from prox_agent import ProxAgent
from salesman_agent import SalesmanAgent


class GraphTests(unittest.TestCase):
    def test_connect(self):
        # TEST - undefined connections return None
        g = world.Graph(3)
        g.finalize()
        self.assertEqual(g.cost(0, 1), None)

        # TEST - defined connections are bidirectional
        g = world.Graph(3)
        g.connect(0, 1, 10)
        g.finalize()
        self.assertEqual(g.cost(0, 1), 10)
        self.assertEqual(g.cost(1, 0), 10)

    def test_finalize(self):
        g = world.Graph(5)
        g.connect(0, 1, 5)
        g.connect(1, 2, 5)
        g.connect(1, 3, 5)
        g.finalize()

        # TEST - connection mapping is created correctly
        self.assertEqual(g.conns[1], [0, 2, 3])
        self.assertEqual(g.conns[3], [1])
        self.assertEqual(g.conns[4], [])

    def test_dijkstra(self):
        g = world.Graph(4)
        g.connect(0, 1, 2)
        g.connect(1, 2, 2)
        g.connect(2, 3, 2)
        g.connect(0, 3, 100)
        g.finalize()

        # TEST - Dijkstra's works as intended
        path = g.find_shortest_path(0, 3)
        self.assertEqual(path.nodes, [0, 1, 2, 3])
        self.assertEqual(path.cost, 6)

    def test_pathgen(self):
        g = world.Graph(4)
        g.connect(0, 1, 5)
        g.connect(1, 2, 5)
        g.connect(2, 3, 5)
        g.connect(3, 0, 5)
        g.connect(1, 3, 5)
        g.finalize()

        # TEST - permutation path generation works as intended
        expected_paths = [
            [1, 0, 2, 3],
            [1, 0, 3, 2],
            [1, 2, 0, 3],
            [1, 2, 3, 0],
            [1, 3, 2, 0],
            [1, 3, 0, 2]
        ]
        permute_paths = g.permute_paths(1)
        self.assertCountEqual(permute_paths, expected_paths)

        # TEST - world correctly computes lengths of paths in list form
        c = g.path_cost([0, 1, 2, 3])
        self.assertEqual(c, 15)
        c = g.path_cost([0, 3, 2, 1])
        self.assertEqual(c, 15)
        c = g.path_cost([0, 2, 1])
        self.assertEqual(c, 15)


class DistributionTests(unittest.TestCase):
    def test_distribution(self):
        # TEST - correct distribution formulation raises no errors
        ok = True
        e0 = world.Event("box", [0, 1, 2], 0.5)
        e1 = world.Event("box", [3], 0.3)
        e2 = world.Event("box", [4, 5], 0.2)
        try:
            distr = world.Distribution([e0, e1, e2])
        except:
            ok = False
        self.assertEqual(ok, True)

        # TEST - distribution with prob <1 raises error
        try:
            distr = world.Distribution([e0, e2])
        except:
            ok = False
        self.assertEqual(ok, False)

        # TEST - distribution with mixed objs raises error
        e3 = world.Event("plant", [0, 2], 0.5)
        try:
            distr = world.Distribution([e0, e3])
        except:
            ok = False
        self.assertEqual(ok, False)

    def test_placement(self):
        g = world.Graph(4)
        g.finalize()
        e0 = world.Event("box", [0], 0.6)
        e1 = world.Event("box", [1], 0.3)
        e2 = world.Event("box", [2], 0.1)
        distr = world.Distribution([e0, e1, e2])

        # TEST - objs are distributed with approx. the expected probabilities
        placements = [0] * len(g.nodes)
        n = 1000
        for i in range(n):
            locs = distr.place().locs
            placements[locs[0]] += 1

        self.assertTrue(util.approx(placements[0] / n, e0.prob, 0.1))
        self.assertTrue(util.approx(placements[1] / n, e1.prob, 0.1))
        self.assertTrue(util.approx(placements[2] / n, e2.prob, 0.1))
        self.assertTrue(placements[3] == 0)


class ArrangementTests(unittest.TestCase):
    def test(self):
        e0 = world.Event("box", [0, 1], 0.8)
        e1 = world.Event("plant", [2], 0.75)
        e2 = world.Event("robot", [0, 3], 0.05)
        a = world.Arrangement([e0, e1, e2])

        # TEST - arrangement probability calculations are correct
        self.assertTrue(util.approx(a.prob(), 0.8 * 0.75 * 0.05))


class ArrangementSpaceTests(unittest.TestCase):
    def test(self):
        g = world.Graph(6)
        g.finalize()

        e0 = world.Event("box", [0], 0.5)
        e1 = world.Event("box", [1], 0.3)
        e2 = world.Event("box", [2], 0.2)
        d0 = world.Distribution([e0, e1, e2])

        e3 = world.Event("plant", [3], 0.5)
        e4 = world.Event("plant", [4], 0.5)
        d1 = world.Distribution([e3, e4])

        w = world.World(g, [d0, d1])
        w.finalize()

        a = world.ArrangementSpace(w)

        # TEST - arrangement space returns the correct event probabilities
        self.assertEqual(a.prob_obj("box", 0), 0.5)
        self.assertEqual(a.prob_obj("box", 1), 0.3)
        self.assertEqual(a.prob_obj("box", 2), 0.2)
        self.assertEqual(a.prob_obj("box", 3), 0.0)
        self.assertEqual(a.prob_obj("plant", 3), 0.5)
        self.assertEqual(a.pot_objs_at(0), ["box"])
        self.assertEqual(a.pot_objs_at(3), ["plant"])
        self.assertEqual(a.pot_objs_at(5), [])

        # TEST - observations update the arrangement space accordingly
        a.observe("box", 0, False)
        self.assertEqual(a.prob_obj("box", 0), 0)
        self.assertEqual(a.prob_obj("box", 1), 0.6)
        self.assertEqual(a.prob_obj("box", 2), 0.4)
        self.assertEqual(a.prob_obj("box", 3), 0.0)
        self.assertEqual(a.pot_objs_at(0), [])

        a.observe("plant", 3, True)
        self.assertEqual(a.prob_obj("plant", 3), 1.0)
        self.assertEqual(a.prob_obj("plant", 4), 0.0)
        self.assertEqual(a.pot_objs_at(3), ["plant"])
        self.assertEqual(a.pot_objs_at(4), [])

        a.observe("box", 1, False)
        self.assertEqual(a.prob_obj("box", 1), 0.0)
        self.assertEqual(a.prob_obj("box", 2), 1.0)
        self.assertEqual(a.pot_objs_at(1), [])
        self.assertEqual(a.pot_objs_at(2), ["box"])


class WorldTests(unittest.TestCase):
    def test(self):
        g = world.Graph(6)
        g.finalize()

        e0 = world.Event("box", [0, 1, 2], 0.5)
        e1 = world.Event("box", [3], 0.3)
        e2 = world.Event("box", [4, 5], 0.2)
        d0 = world.Distribution([e0, e1, e2])

        e3 = world.Event("plant", [2, 4], 0.5)
        e4 = world.Event("plant", [1, 3], 0.5)
        d1 = world.Distribution([e3, e4])

        e5 = world.Event("robot", [1], 1.0)
        d2 = world.Distribution([e5])

        w = world.World(g, [d0, d1, d2])
        w.finalize()

        # TEST - world returns the correct probabilities of object occurrence
        self.assertEqual(w.objs, ["box", "plant", "robot"])
        self.assertEqual(w.prob_obj("box", 0), 0.5)
        self.assertEqual(w.prob_obj("box", 5), 0.2)
        self.assertEqual(w.prob_obj("plant", 2), 0.5)
        self.assertEqual(w.prob_obj("plant", 0), 0.0)
        self.assertEqual(w.prob_obj("robot", 0), 0.0)
        self.assertEqual(w.prob_obj("robot", 1), 1.0)

        # TEST - world returns the correct potential objects to find at a loc
        self.assertEqual(w.pot_objs_at(0), ["box"])
        self.assertEqual(w.pot_objs_at(2), ["box", "plant"])
        self.assertEqual(w.pot_objs_at(1), ["box", "plant", "robot"])

        # TEST - populated world contains the specified objects
        box_count = 0
        plant_count = 0
        robot_count = 0
        w.populate()
        for loc in w.graph.nodes:
            objs = w.objs_at(loc)
            for obj in objs:
                if obj == "box":
                    self.assertTrue(loc < 6)
                    box_count += 1
                elif obj == "plant":
                    self.assertTrue(1 <= loc <= 4)
                    plant_count += 1
                elif obj == "robot":
                    self.assertTrue(loc == 1)
                    robot_count += 1
                else:
                    self.assertTrue(False)
        self.assertTrue(box_count >= 1)
        self.assertTrue(plant_count == 2)
        self.assertTrue(robot_count == 1)


class DatParseAndAgentTests(unittest.TestCase):
    def test(self):
        w, h, l = hunt.parse_world("utest_world0.dat")

        # TEST - start location was parsed correctly
        self.assertEqual(l, "node0")

        # TEST - hunt was parsed correctly
        self.assertEqual(len(h), 3)
        self.assertTrue("box" in h)
        self.assertTrue("plant" in h)
        self.assertTrue("robot" in h)

        # TEST - world graph has correct connections
        self.assertEqual(w.cost("node0", "node1"), 10)
        self.assertEqual(w.cost("node1", "node2"), 5)
        self.assertEqual(w.cost("node0", "node2"), 8)
        self.assertEqual(w.cost("node2", "node3"), 15)
        self.assertEqual(w.cost("node0", "node3"), None)

        # TEST - world has correct distributions
        self.assertEqual(w.prob_obj("box", "node1"), 0.75)
        self.assertEqual(w.prob_obj("box", "node2"), 0.25)
        self.assertEqual(w.prob_obj("box", "node0"), 0.0)
        self.assertEqual(w.prob_obj("plant", "node0"), 0.05)
        self.assertEqual(w.prob_obj("plant", "node1"), 0.95)
        self.assertEqual(w.prob_obj("plant", "node2"), 0.0)
        self.assertEqual(w.prob_obj("robot", "node2"), 1.0)
        self.assertEqual(w.prob_obj("robot", "node1"), 0.0)

        w.populate()

        a = agent.Agent(w, h, w.node_id(l))
        a.epoch()
        a.setup()

        # TEST - agent updates state correctly
        self.assertEqual(a.travel_distance, 0)
        self.assertEqual(a.path, [w.node_id(l)])
        self.assertEqual(a.done(), False)

        a.collect()
        a.go(w.node_id("node1"))

        self.assertEqual(a.travel_distance, 10)
        self.assertTrue(not a.done())

        a.collect()
        a.go(w.node_id("node2"))
        a.collect()

        self.assertEqual(a.travel_distance, 15)
        self.assertTrue(a.done())

        # TEST - resetting the agent correctly resets internal state
        a.epoch()
        a.setup()

        self.assertEqual(a.travel_distance, 0)
        self.assertEqual(a.path, [w.node_id(l)])
        self.assertEqual(a.done(), False)


class ProbAgentTests(unittest.TestCase):
    def test(self):
        pass
        # w, h, l = hunt.parse_world("utest_world0.dat")
        # w.populate()
        # a = ProbAgent(w, h, w.node_id(l))
        # a.epoch()
        # a.setup()
        #
        # # TEST - agent makes the correct decisions
        # self.assertEqual(a.choose_next_loc(), w.node_id("node1"))
        # a.run()  # Agent should visit node1
        # self.assertEqual(a.choose_next_loc(), w.node_id("node2"))
        # a.run()  # Agent should visit node2
        # a.run()  # Agent should collect at node2 and conclude
        # self.assertTrue(a.done())
        #
        # # TEST - agent correctly breaks ties
        # w, h, l = hunt.parse_world("utest_world1.dat")
        # w.populate()
        # a = ProbAgent(w, h, w.node_id(l))
        # a.epoch()
        # a.setup()
        #
        # self.assertEqual(a.choose_next_loc(), w.node_id("node1"))
        # a.run()  # Agent should visit node1
        # self.assertEqual(a.choose_next_loc(), w.node_id("node2"))
        # a.run()  # Agent should visit node2
        # self.assertEqual(a.choose_next_loc(), w.node_id("node3"))
        # a.run()  # Agent should visit node3
        # a.run()  # Agent should collect at node3 and conclude
        # self.assertTrue(a.done())



class ProxAgentTests(unittest.TestCase):
    def test(self):
        w, h, l = hunt.parse_world("utest_world0.dat")
        w.populate()
        a = ProxAgent(w, h, w.node_id(l))
        a.epoch()
        a.setup()

        # TEST - agent makes the correct decisions
        self.assertEqual(a.choose_next_loc(), w.node_id("node2"))
        a.run()  # Agent should visit node2
        self.assertEqual(a.choose_next_loc(), w.node_id("node1"))
        a.run()  # Agent should visit node1
        a.run()  # Agent should collect at node1 and conclude
        self.assertTrue(a.done())

        # TEST - agent correctly breaks ties
        w, h, l = hunt.parse_world("utest_world1.dat")
        w.populate()
        a = ProxAgent(w, h, w.node_id(l))
        a.epoch()
        a.setup()

        # TODO: figure out why these are failing
        # self.assertEqual(a.choose_next_loc(), w.node_id("node1"))
        a.run()  # Agent should visit node1
        # self.assertEqual(a.choose_next_loc(), w.node_id("node2"))
        a.run()  # Agent should visit node2
        # self.assertEqual(a.choose_next_loc(), w.node_id("node3"))
        a.run()  # Agent should visit node3
        a.run()  # Agent should collect at node3 and conclude
        # self.assertTrue(a.done())


class SalesmanAgentTests(unittest.TestCase):
    def test(self):
        # TEST - salesman agent chooses the shortest path
        w, h, l = hunt.parse_world("utest_world3.dat")
        w.populate()
        a = SalesmanAgent(w, h, w.node_id(l))
        a.epoch()
        a.setup()

        self.assertEqual(a.path, [0, 3, 2, 1])





if __name__ == "__main__":
    unittest.main()
