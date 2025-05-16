import numpy as np
from stable_baselines3 import PPO
from env import Environment
class Agents:

    def __init__(self, model_path):
        self.model_path = model_path
        self.agents = []
        self.n_robots = 0
        self.state = None


    def init_agents(self, state):
        self.model = PPO.load(self.model_path)
        self.state = state
        self.n_robots = len(state['robots'])
        self.env = Environment(map_file=state['map'], max_time_steps=state['max_time_steps'])

    def get_actions(self, state):
        obs = self.env.get_state(state)
        actions = self.model.predict(obs, deterministic=True)[0]
        # list_actions = ['S', 'L', 'R', 'U', 'D']
        
        # for i in range(self.n_robots):
        #     move = np.random.randint(0, len(list_actions))
        #     pkg_act = np.random.randint(0, 3)
        #     actions.append((list_actions[move], str(pkg_act)))
        return actions