import absim.agent as agent
import absim.world as world

import torch
from torch import nn
import numpy as np

state_dict_path = './absim/dqn_16x16.pth'

class Mlp16x16(nn.Module):
    def __init__(self, state_shape, action_shape):
        super().__init__()
        self.model = nn.Sequential(*[
            nn.Linear(np.prod(state_shape), 16), nn.ReLU(inplace=True),
            nn.Linear(16, 16), nn.ReLU(inplace=True),
            nn.Linear(16, np.prod(action_shape))
        ])
    def forward(self, obs, state=None, info={}):
        if not isinstance(obs, torch.Tensor):
            obs = torch.tensor(obs, dtype=torch.float)
        batch = obs.shape[0]
        logits = self.model(obs.view(batch, -1))
        return logits, state

class DQN16x16Agent(agent.Agent):
    """The agent load a 16x16 mlp model that compute the Q value function. The
    Action is selected by choose the action that returns the best Q value.
    """
    def setup(self):
        super().setup()
        state_shape = len(self.world.graph.nodes)
        action_shape = len(self.world.graph.nodes)

        state_dict = {}
        state_dict_raw = torch.load(state_dict_path)
        for key in state_dict_raw.keys():
            if key.split('.')[0] == 'model':
                state_dict[key[6:]] = state_dict_raw[key]
        self.net = Mlp16x16(state_shape, action_shape)
        self.net.load_state_dict(state_dict)

    def get_obs(self):
        '''Observation is a vector of probabilities of finding at least one
        object at a node. Agent's current node postion is -1.
        '''
        obs = []
        for node in self.world.graph.nodes:
            obs.append(self.arrangement_space.prob_any_obj(node))
        obs = [0 if self.visited_count[i] > 0 else o for i, o in enumerate(obs)]
        obs[self.loc] = -1
        return torch.tensor([obs])

    def choose_next_loc(self):
        actions = np.array(self.net(self.get_obs())[0].detach().cpu())
        action = np.argmax(actions.reshape(-1))
        return action

    def run(self):
        # Collect objects at current location and update the occurrence space
        super().run()

        if self.done():
            return

        # Visit next location in the active path
        self.go(self.choose_next_loc())
