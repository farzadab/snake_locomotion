from rllab.algos.trpo import TRPO
from rllab.algos.ppo import PPO
from rllab.algos.cma_es import CMAES
from rllab.baselines.linear_feature_baseline import LinearFeatureBaseline
from rllab.envs.box2d.cartpole_env import CartpoleEnv
from rllab.envs.normalized_env import normalize
from rllab.policies.gaussian_mlp_policy import GaussianMLPPolicy
from rllab_env import SimpleSnakeEnv
from controllers import LLC

NUM_LINKS = 9
env = SimpleSnakeEnv(num_links=NUM_LINKS, controller=LLC(NUM_LINKS), graphical=True)

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
    batch_size=80000,
    whole_paths=True,
    max_path_length=4000,
    n_itr=400,
    discount=0.999,
    step_size=0.04,
)
algo.train()
