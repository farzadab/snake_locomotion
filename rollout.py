from rllab.sampler.utils import rollout
import pybullet as p
import joblib
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('--exp_name', type=str, help='name of the experiment (and the output folder)')
args = parser.parse_args()

data = joblib.load('/Users/inaz/rllab/data/local/experiment/%s/params.pkl' % args.exp_name)
p.connect(p.GUI)
data['env'].graphical = True
data['env'].reset()
rollout(data['env'], data['policy'])