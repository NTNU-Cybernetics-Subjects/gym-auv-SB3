import os
import sys
import subprocess
import numpy as np
from time import time, sleep
import argparse
import json
import copy
#from tqdm import tqdm
import progressbar
import torch
import gym
import gym_auv
import gym_auv.reporting
import multiprocessing

from stable_baselines3.common.utils import set_random_seed
from stable_baselines3.common.vec_env import VecVideoRecorder, DummyVecEnv, SubprocVecEnv, VecFrameStack
from stable_baselines3.common.noise import NormalActionNoise, OrnsteinUhlenbeckActionNoise
from stable_baselines3 import PPO, DDPG, TD3, A2C, SAC
from sklearn.model_selection import ParameterGrid

from stable_baselines3.common.callbacks import EveryNTimesteps, EventCallback, BaseCallback, EvalCallback
import queue
from collections import deque
from datetime import datetime



test = [1]

for i in range(20):
    t = int(np.sum(test)/4) % 4
    print(t, len(test))
    test.append(1)
