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
        for agent, action_array in zip(self.agents, actions):
            action = hacker_league.Action()
            action.throttle = action_array[0]
            action.steering = action_array[1]
            agent.action = action
        hacker_league.step(self.ball, self.agents, self.step_duration, self.scores)

class Model:
    def __init__(self, input_dim, hidden_dim, output_dim):
        self.weights0 = np.random.randn(input_dim, hidden_dim) * np.sqrt(1 / input_dim)
        self.bias0 = np.zeros((1, hidden_dim))
        self.weights1 = np.random.randn(hidden_dim, output_dim) * np.sqrt(1 / hidden_dim)
        self.bias1 = np.zeros((1, output_dim))

    def forward(self, x):
        x = np.tanh(np.dot(x, self.weights0) + self.bias0)
        x = np.tanh(np.dot(x, self.weights1) + self.bias1)
        return x

    def params_add(self, params):
        w0 = self.weights0.size
        b0 = self.bias0.size
        w1 = self.weights1.size
        self.weights0 += params[:w0].reshape(self.weights0.shape)
        self.bias0 += params[w0:w0 + b0].reshape(self.bias0.shape)
        self.weights1 += params[w0 + b0:w0 + b0 + w1].reshape(self.weights1.shape)
        self.bias1 += params[w0 + b0 + w1:].reshape(self.bias1.shape)

    def n_params(self):
        return self.weights0.size + self.bias0.size + self.weights1.size + self.bias1.size

if __name__ == "__main__":
    population_size = 50
    sigma = 0.01
    alpha = 0.01
    max_steps = 600
    max_generations = 10000

    environment = Environment(two_agents=False)
    model = Model(18, 8, 2)
    for generation in range(max_generations):
        perturbations = np.random.randn(population_size, model.n_params())
        rewards = np.zeros(population_size)
        for i in range(population_size):
            model.params_add(sigma * perturbations[i])
            total_reward = 0
            environment.reset()
            for _ in range(max_steps):
                agent = environment.agents[0].state
                ball = environment.ball
                environment.step([model.forward(np.concatenate([agent.position, agent.velocity, agent.orientation, ball.position, ball.velocity, ball.orientation])).squeeze()])
                total_reward -= 1
                if environment.agents[0].state.position[2] > 0:
                    break
            rewards[i] = total_reward
        model.params_add(alpha / (population_size * sigma) * np.dot(perturbations.T, (rewards - np.mean(rewards)) / (np.std(rewards) + 1e-8)))
        print(f"Generation {generation + 1}, Average Reward: {np.mean(rewards)}")
