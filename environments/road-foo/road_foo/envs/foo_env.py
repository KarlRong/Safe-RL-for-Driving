import gym
from gym import error, spaces, utils
from gym.utils import seeding


class FooEnv(gym.Env):
    metadata = {'render.modes': ['human']}
    i = 0

    def __init__(self):
        i = 0

    def step(self, action):
        i = i + action

    def reset(self):
        i = 0

    def render(self, mode='human'):
        i = i + i

    def close(self):
        i = 0



