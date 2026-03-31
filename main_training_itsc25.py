import json
import os

from sigmarl.helper_training import SaveData
from sigmarl.mappo_cavs import mappo_cavs
from sigmarl.ppo_goal_reaching import ppo_goal_reaching
from sigmarl.constants import SCENARIOS


path = "checkpoints/itsc25"
path_to_json_file = os.path.join(path, "reward-1.28_data.json")

with open(path_to_json_file, "r") as file:
    data = json.load(file)
    saved_data = SaveData.from_dict(data)
    parameters = saved_data.parameters

# Keep the ITSC25 training observation/action configuration so the saved model
# can be loaded later with the same actor input structure.
parameters.is_testing_mode = False
parameters.is_load_model = False
parameters.is_load_final_model = False
parameters.is_load_out_td = False
parameters.is_save_eval_results = False
parameters.is_save_simulation_video = False
parameters.is_real_time_rendering = False
parameters.is_continue_train = False

# Match the single-agent ITSC25 setup that evaluation and rendering use.
parameters.scenario_type = "CPM_mixed"
parameters.n_agents = 1
parameters.n_nearing_agents_observed = 0
parameters.is_using_cbf = False
parameters.is_using_prioritized_marl = False
parameters.is_using_centralized_cbf = False
parameters.prioritization_method = "random"

# Save to a separate folder so the new run does not overwrite the reference
# checkpoint files.
parameters.where_to_save = "outputs/itsc25_retrain/"
parameters.model_name = ""

# Refresh a few runtime values explicitly for training.
parameters.n_agents = min(
    parameters.n_agents, SCENARIOS[parameters.scenario_type]["n_agents"]
)
parameters.num_vmas_envs = 32
parameters.random_seed = 0
parameters.n_iters = 250

if parameters.scenario_type.lower() == "goal_reaching_1":
    env, policy, priority_module, parameters = ppo_goal_reaching(parameters=parameters)
else:
    env, policy, priority_module, _, parameters = mappo_cavs(parameters=parameters)
