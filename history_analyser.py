import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import os

# logs/history/DockingStraightVariationScenario-v0/1734445950__17-12-2024__15:32:30__ppo
DIR_PATH = os.path.dirname(os.path.realpath(__file__))
# senario_id = "1734448676__17-12-2024__16:17:56__ppo"
# senario = "DockingStraightScenario-v0"
# senario = "DockingStraightScenario-v0/"
# id = "1734560241__18-12-2024__23:17:21__ppo"
# id = "1734547859__18-12-2024__19:50:59__ppo"
# id = "1734630970__19-12-2024__18:56:10__ppo"
# id = "1734703709__20-12-2024__15:08:29__ppo"
# senario = "DockingStraightVariationScenario-v0" 
# id = "1734778058__21-12-2024__11:47:38__ppo"
# id = "1734791955__21-12-2024__15:39:15__ppo"

# id = "1734867552__22-12-2024__12:39:12__ppo"
# senario = "DockingStraightVariationScenario-v1"
# id = "1734886310__22-12-2024__17:51:50__ppo"

# senario = "DockingStraightScenario-v0"
# id = "1734955745__23-12-2024__13:09:05__ppo"

# senario = "DockingStraightScenario-v0"
# id = "1734964629__23-12-2024__15:37:09__ppo"
# senario = "DockingStraightVariationScenario-v1"
senario = "DockingStraightScenario-v0"
id = "1735850721__02-01-2025__21:45:21__ppo"

senario = "DockingStraightScenario-v0"
id = "1735839946__02-01-2025__18:45:46__ppo"
# id = "1734973794__23-12-2024__18:09:54__ppo"

logfile = os.path.join(DIR_PATH, "logs", "history", senario, id, "history.csv")
# logfile = "/home/hurodor/Dev/gym-auv-SB3/trained_agents/penelizerStraightVariation/1734869505__22-12-2024__13:11:45__ppo/history/history.csv"
# logfile = "/home/hurodor/Dev/gym-auv-SB3/logs/history/DockingStraightScenario-v0/1735850721__02-01-2025__21:45:21__ppo/history.csv"

df = pd.read_csv(logfile)

                # "cross_track_error": np.array(
                #     self._tmp_storage["cross_track_error"]
                # ).mean(),
                # "reached_goal": int(self.reached_goal),
                # "collision": int(self.collision),
                # "reward": self.cumulative_reward,
                # "timesteps": self.t_step,
                # "duration": self.t_step * self.config["t_step_size"],
                # "progress": self.progress,
                # "pathlength": self.path.length if self.path is not None else 0,


reward = df["reward"].to_numpy()
cummulative_reward = [reward[i] + reward[i+1] for i in range(len(reward)-1)]
cummulative_reward.append(cummulative_reward[-1])
reached_goal = df["reached_goal"].to_numpy()


print(f"reached goal {np.sum(reached_goal)} times in {len(reached_goal)+1} episodes")
episodes = np.arange(1, len(reward)+1, 1)
plt.plot(episodes, reward, label="reward")
plt.plot(episodes, cummulative_reward, label="cummulative reward")
plt.legend()
plt.show()
