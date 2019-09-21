from gym.envs.registration import register

register(
    id='foo-v0',
    entry_point='road_foo.envs:FooEnv',
)
