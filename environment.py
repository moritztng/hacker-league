import hacker_league
import numpy as np

class Environment:
    def __init__(self, two_agents=False, frequency=60):
        self.ball = hacker_league.State()
        self.agents = [hacker_league.Agent()]
        if two_agents:
            self.agents.append(hacker_league.Agent())
        self.scores = None
        self.step_duration = 1 / frequency
        self.reset()

    def reset(self):
        self.ball.position = np.array([0.0, 1.0, 0.0], dtype=np.float32)
        self.ball.velocity = np.array([0.0, 0.0, 0.0], dtype=np.float32)
        self.ball.orientation = np.array([0.0, 0.0, 0.0], dtype=np.float32)
        self.agents[0].state.position = np.array([0.0, 0.375, -40.0], dtype=np.float32)
        self.agents[0].state.velocity = np.array([0.0, 0.0, 0.0], dtype=np.float32)
        self.agents[0].state.orientation = np.array([0.0, 0.0, 0.0], dtype=np.float32)
        if len(self.agents) == 2:
            self.agents[1].state.position = np.array([0.0, 0.375, 40.0], dtype=np.float32)
            self.agents[1].state.velocity = np.array([0.0, 0.0, 0.0], dtype=np.float32)
            self.agents[1].state.orientation = np.array([0.0, np.pi, 0.0], dtype=np.float32)
        self.scores = np.array([0, 0], dtype=np.uint8)

    def step(self, actions):
        for agent, action in zip(self.agents, actions):
            agent.action = action
        hacker_league.step(self.ball, self.agents, self.step_duration, self.scores)

if __name__ == "__main__":
    environment = Environment()
    action = hacker_league.Action()
    action.throttle = 1.0
    action.steering = 0.0
    for _ in range(5):
        environment.step([action])
        agentState = environment.agents[0].state
        print(agentState.position, agentState.orientation, agentState.velocity)
