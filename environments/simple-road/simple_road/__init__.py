from gym.envs.registration import register

register(
    id='road-v0',
    entry_point='simple_road.envs:SimpleRoad',
    # kwargs={'m': 9, 'n': 9, 'magicSquares': {18: 54, 63: 14}}
)