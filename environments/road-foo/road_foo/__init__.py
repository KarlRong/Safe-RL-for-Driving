from gym.envs.registration import register

register(
    id='foo-v0',
    entry_point='road_foo.envs:FooEnv',
)

register(
    id='grid-v0',
    entry_point='road_foo.envs:GridWorld',
    kwargs={'m': 9, 'n': 9, 'magicSquares': {18: 54, 63: 14}}
)