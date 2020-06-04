"""Runner script for single and multi-agent reinforcement learning experiments.

This script performs an RL experiment using the PPO algorithm. Choice of
hyperparameters can be seen and adjusted from the code below.

Usage
    python train.py EXP_CONFIG
"""

import argparse
import json
import os
import sys
from time import strftime

import ray
from ray import tune
from ray.tune import run_experiments
from ray.tune.registry import register_env
from flow.utils.registry import make_create_env
try:
    from ray.rllib.agents.agent import get_agent_class
except ImportError:
    from ray.rllib.agents.registry import get_agent_class
from copy import deepcopy

from flow.core.util import ensure_dir
from flow.utils.registry import env_constructor
from flow.utils.rllib import FlowParamsEncoder, get_flow_params
from typing import Dict
from ray.rllib.env import BaseEnv
from ray.rllib.policy import Policy
from ray.rllib.policy.sample_batch import SampleBatch
from ray.rllib.evaluation import MultiAgentEpisode, RolloutWorker
from ray.rllib.agents.callbacks import DefaultCallbacks
import numpy as np


class MyCallbacks(DefaultCallbacks):
    def on_episode_start(self, worker: RolloutWorker, base_env: BaseEnv,
                         policies: Dict[str, Policy],
                         episode: MultiAgentEpisode, **kwargs):
        print("episode {} started".format(episode.episode_id))
        # episode.user_data["pole_angles"] = []
        # episode.hist_data["pole_angles"] = []

    # def on_episode_step(self, worker: RolloutWorker, base_env: BaseEnv,
    #                     episode: MultiAgentEpisode, **kwargs):
    #     print("on_episode_step")
    #     print(base_env.get_unwrapped()[0].num_collide)
    #     pole_angle = abs(episode.last_observation_for()[2])
    #     raw_angle = abs(episode.last_raw_obs_for()[2])
    #     assert pole_angle == raw_angle
        # episode.user_data["pole_angles"].append(pole_angle)

    def on_episode_end(self, worker: RolloutWorker, base_env: BaseEnv,
                       policies: Dict[str, Policy], episode: MultiAgentEpisode,
                       **kwargs):
        print("on_episode_end")
        sucess = []
        fail = []
        collide = []
        outrange = []
        redcross = []
        custom_reward = []
        for env in base_env.get_unwrapped():
            sucess.append(env.num_sucess)
            fail.append(env.num_fail)
            collide.append(env.num_collide)
            outrange.append(env.num_out_range)
            redcross.append(env.num_red)
            custom_reward.append(env.custom_reward)

        sucess = np.array(sucess)
        fail = np.array(fail)
        collide = np.array(collide)
        outrange = np.array(outrange)
        redcross = np.array(redcross)
        custom_reward = np.array(custom_reward)
        print("sucess", sucess, "fail", fail, "collide", collide, "outrange", outrange, "redcross", redcross, "custom_reward", custom_reward)
        episode.custom_metrics["average sucess"] = np.sum(sucess)
        episode.custom_metrics["average fail"] = np.sum(fail)
        episode.custom_metrics["average collide"] = np.sum(collide)
        episode.custom_metrics["average outrange"] = np.sum(outrange)
        episode.custom_metrics["average redcross"] = np.sum(redcross)
        episode.custom_metrics["custom_reward"] = np.sum(custom_reward)
        episode.custom_metrics["sucess rate"] = np.sum(sucess) / (np.sum(sucess) + np.sum(fail))
        # pole_angle = np.mean(episode.user_data["pole_angles"])
        # print("episode {} ended with length {} and pole angles {}".format(
        #     episode.episode_id, episode.length, pole_angle))
        # episode.custom_metrics["pole_angle"] = pole_angle
        # episode.hist_data["pole_angles"] = episode.user_data["pole_angles"]

    def on_sample_end(self, worker: RolloutWorker, samples: SampleBatch,
                      **kwargs):
        print("returned sample batch of size {}".format(samples.count))

    def on_train_result(self, trainer, result: dict, **kwargs):
        print("trainer.train() result: {} -> {} episodes".format(
            trainer, result["episodes_this_iter"]))
        # you can mutate the result dict to add new fields to return
        result["callback_ok"] = True

    def on_postprocess_trajectory(
            self, worker: RolloutWorker, episode: MultiAgentEpisode,
            agent_id: str, policy_id: str, policies: Dict[str, Policy],
            postprocessed_batch: SampleBatch,
            original_batches: Dict[str, SampleBatch], **kwargs):
        print("postprocessed {} steps".format(postprocessed_batch.count))
        if "num_batches" not in episode.custom_metrics:
            episode.custom_metrics["num_batches"] = 0
        episode.custom_metrics["num_batches"] += 1

def mycallback():
    return DefaultCallbacks()

print(callable(DefaultCallbacks))

def parse_args(args):
    """Parse training options user can specify in command line.

    Returns
    -------
    argparse.Namespace
        the output parser object
    """
    parser = argparse.ArgumentParser(
        formatter_class=argparse.RawDescriptionHelpFormatter,
        description="Parse argument used when running a Flow simulation.",
        epilog="python train.py EXP_CONFIG")

    # required input parameters
    parser.add_argument(
        'exp_config', type=str,
        help='Name of the experiment configuration file, as located in '
             'exp_configs/rl/singleagent or exp_configs/rl/multiagent.')

    # optional input parameters
    parser.add_argument(
        '--rl_trainer', type=str, default="RLlib",
        help='the RL trainer to use. either RLlib or Stable-Baselines')

    parser.add_argument(
        '--num_cpus', type=int, default=1,
        help='How many CPUs to use')
    parser.add_argument(
        '--num_steps', type=int, default=5000,
        help='How many total steps to perform learning over')
    parser.add_argument(
        '--rollout_size', type=int, default=1000,
        help='How many steps are in a training batch.')

    return parser.parse_known_args(args)[0]

def setup_exps_rllib(flow_params,
                     n_cpus,
                     n_rollouts):
    """Return the relevant components of an RLlib experiment.

    Parameters
    ----------
    flow_params : dict
        flow-specific parameters (see flow/utils/registry.py)
    n_cpus : int
        number of CPUs to run the experiment over
    n_rollouts : int
        number of rollouts per training iteration

    Returns
    -------
    str
        name of the training algorithm
    str
        name of the gym environment to be trained
    dict
        training configuration parameters
    """
    horizon = flow_params['env'].horizon

    alg_run = "PPO"

    agent_cls = get_agent_class(alg_run)
    config = deepcopy(agent_cls._default_config)

    config["num_workers"] = n_cpus
    config["num_cpus_per_worker"] = 1
    config["use_pytorch"] = False
    config["num_gpus"] = 0
    config["train_batch_size"] = horizon * n_rollouts
    config["gamma"] = 0.999  # discount rate
    # config["model"].update({"fcnet_hiddens": [32, 32, 32]})
    config["use_gae"] = True
    config["lambda"] = 0.97
    config["kl_target"] = 0.02
    config["num_sgd_iter"] = 10
    config['clip_actions'] = True  # FIXME(ev) temporary ray bug
    config["horizon"] = horizon
    config["callbacks"] = MyCallbacks
    # save the flow params for reply
    flow_json = json.dumps(
        flow_params, cls=FlowParamsEncoder, sort_keys=True, indent=4)
    config['env_config']['flow_params'] = flow_json
    config['env_config']['run'] = alg_run

    create_env, gym_name = make_create_env(params=flow_params)

    # Register as rllib env
    register_env(gym_name, create_env)
    return alg_run, gym_name, config


if __name__ == "__main__":
    flags = parse_args(sys.argv[1:])

    # import relevant information from the exp_config script
    module = __import__("rl.singleagent", fromlist=[flags.exp_config])
    if hasattr(module, flags.exp_config):
        submodule = getattr(module, flags.exp_config)
    else:
        assert False, "Unable to find experiment config!"
    if flags.rl_trainer == "RLlib":
        flow_params = submodule.flow_params
        n_cpus = submodule.N_CPUS
        n_rollouts = submodule.N_ROLLOUTS

        alg_run, gym_name, config = setup_exps_rllib(
            flow_params, n_cpus, n_rollouts)

        ray.init(num_cpus=n_cpus + 1)
        trials = run_experiments({
            flow_params["exp_tag"]: {
                "run": alg_run,
                "env": gym_name,
                "config": {
                    **config
                },
                # "restore": "/home/rong/ray_results/highway_lanechange/PPO_LaneChangeAccelPOEnv-v0_f9e64d14_0_2020-02-25_01-27-48iwsfxxrq/checkpoint_197/checkpoint-197",
                "checkpoint_freq": 1,
                "checkpoint_at_end": True,
                "max_failures": 999,
                "stop": {
                    "training_iteration": 400,
                },
            }
        })
    else:
        assert False, "rl_trainer should be either 'RLlib'"
