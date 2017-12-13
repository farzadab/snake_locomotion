from rllab.algos.trpo import TRPO
from rllab.algos.ppo import PPO
from rllab.algos.cma_es import CMAES
from rllab.baselines.linear_feature_baseline import LinearFeatureBaseline
from rllab.envs.box2d.cartpole_env import CartpoleEnv
from rllab.envs.normalized_env import normalize
from rllab.policies.gaussian_mlp_policy import GaussianMLPPolicy
from rllab_env import SimpleSnakeEnv, LLC

env = SimpleSnakeEnv(num_links=9, controller=None, graphical=False)

policy = GaussianMLPPolicy(
    env_spec=env.spec,
    # The neural network policy should have two hidden layers, each with 32 hidden units.
    hidden_sizes=(32, 32, 32)
)

baseline = LinearFeatureBaseline(env_spec=env.spec)

algo = PPO(
    env=env,
    policy=policy,
    baseline=baseline,
    batch_size=40000,
    whole_paths=True,
    max_path_length=2000,
    n_itr=200,
    discount=0.995,
    step_size=0.01,
)
algo.train()
