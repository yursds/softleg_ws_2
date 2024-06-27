import torch
import numpy            as np
from stable_baselines3  import PPO

# TODO: device should be a parameter

def build_sb3_rl_model(model_path):

    model = PPO.load(model_path)
    
    return model

def normalize_obs():
    pass

def rescale_actions():
    pass

def run_inference(model:PPO, observation:np.ndarray) -> np.ndarray:
    """
    Runs inference on a model given an observation.

    Args:
        model: A PyTorch model.
        observation: A numpy array containing the observation.

    Returns:
        A numpy array containing the action.
    """
    
    actions, _ = model.predict(observation, deterministic=True)
    
    return actions
