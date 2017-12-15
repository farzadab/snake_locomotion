import argparse
from rllab.algos.trpo import TRPO
from rllab.algos.ppo import PPO
from rllab.algos.cma_es import CMAES
from rllab.baselines.linear_feature_baseline import LinearFeatureBaseline
from rllab.envs.box2d.cartpole_env import CartpoleEnv
from rllab.envs.normalized_env import normalize
from rllab.misc.instrument import run_experiment_lite
from rllab.policies.gaussian_mlp_policy import GaussianMLPPolicy
from rllab_env import SimpleSnakeEnv
from controllers import get_controller_by_name

def run_task(*_):
    NUM_LINKS = 9
    env = SimpleSnakeEnv(
        num_links=NUM_LINKS, graphical=args.graphical,
        controller=get_controller_by_name(args.controller)(NUM_LINKS-1))

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
        max_path_length=args.max_path_length,
        n_itr=args.n_iters,
        discount=0.999,
        step_size=0.04,
    )
    algo.train()


parser = argparse.ArgumentParser()
# parser.add_argument('--exp_name', type=str, help='name of the experiment (and the output folder)')
parser.add_argument('--controller', type=str, help='name of the controller')
parser.add_argument('--max_path_length', type=int, default=4000, help='Max length of rollout')
parser.add_argument('--n_iters', type=int, default=40, help='Number of iterations')
parser.add_argument('--graphical', type=bool, default=False, help='Graphical interface')
args = parser.parse_args()


if __name__ == '__main__':
    run_experiment_lite(
        run_task,
        # Number of parallel workers for sampling
        n_parallel=1,
        # Only keep the snapshot parameters for the last iteration
        snapshot_mode="last",
        # Specifies the seed for the experiment. If this is not provided, a random seed
        # will be used
        seed=1,
        # plot=True,
        exp_name= args.controller + '_' + str(args.n_iters) + '-iters',
    )
