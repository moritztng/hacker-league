import hacker_league_physics, socket, struct, subprocess
import numpy as np


class Environment:
    def __init__(self, two_agents=False, frequency=60):
        self.ball = hacker_league_physics.State()
        self.agents = [hacker_league_physics.Agent()]
        if two_agents:
            self.agents.append(hacker_league_physics.Agent())
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
            self.agents[1].state.position = np.array(
                [0.0, 0.375, 40.0], dtype=np.float32
            )
            self.agents[1].state.velocity = np.array([0.0, 0.0, 0.0], dtype=np.float32)
            self.agents[1].state.orientation = np.array(
                [0.0, np.pi, 0.0], dtype=np.float32
            )
        self.scores = np.array([0, 0], dtype=np.uint8)

    def step(self, actions):
        for agent, action_array in zip(self.agents, actions):
            action = hacker_league_physics.Action()
            action.throttle = action_array[0]
            action.steering = action_array[1]
            agent.action = action
        hacker_league_physics.step(
            self.ball, self.agents, self.step_duration, self.scores
        )


class Model:
    def __init__(self, input_dim, hidden_dim, output_dim):
        self.weights0 = np.random.randn(input_dim, hidden_dim) * np.sqrt(1 / input_dim)
        self.bias0 = np.zeros((1, hidden_dim))
        self.weights1 = np.random.randn(hidden_dim, output_dim) * np.sqrt(
            1 / hidden_dim
        )
        self.bias1 = np.zeros((1, output_dim))

    def forward(self, x):
        x = np.tanh(np.dot(x, self.weights0) + self.bias0)
        x = np.tanh(np.dot(x, self.weights1) + self.bias1)
        return x

    def get_params(self):
        return np.concatenate(
            [
                self.weights0.ravel(),
                self.bias0.ravel(),
                self.weights1.ravel(),
                self.bias1.ravel(),
            ]
        )

    def set_params(self, params):
        w0 = self.weights0.size
        b0 = self.bias0.size
        w1 = self.weights1.size
        self.weights0 = params[:w0].reshape(self.weights0.shape)
        self.bias0 = params[w0 : w0 + b0].reshape(self.bias0.shape)
        self.weights1 = params[w0 + b0 : w0 + b0 + w1].reshape(self.weights1.shape)
        self.bias1 = params[w0 + b0 + w1 :].reshape(self.bias1.shape)

    def n_params(self):
        return (
            self.weights0.size + self.bias0.size + self.weights1.size + self.bias1.size
        )


def play(policy):
    game_process = subprocess.Popen(
        ["./hacker-league", "environment"],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind(("localhost", 5000))
        while True:
            data, address = sock.recvfrom(72)
            if len(data) != 72:
                raise ValueError(f"error receive data length")

            action = policy(np.array(struct.unpack("18f", data))).squeeze()

            sock.sendto(struct.pack("ff", *action), address)


def evolution_strategies(
    params,
    fitness,
    max_generations=100,
    population_size=50,
    noise_stdev=0.01,
    learning_rate=0.01,
    adam=False,
    rank_transformation=True,
):
    class AdamOptimizer:
        def __init__(self, learning_rate=0.01, beta1=0.9, beta2=0.999, epsilon=1e-8):
            self.learning_rate = learning_rate
            self.beta1 = beta1
            self.beta2 = beta2
            self.epsilon = epsilon
            self.m = None
            self.v = None
            self.t = 0

        def update(self, grads):
            if self.m is None:
                self.m = np.zeros_like(grads)
            if self.v is None:
                self.v = np.zeros_like(grads)
            self.t += 1
            # Update biased first and second moment estimates
            self.m = self.beta1 * self.m + (1 - self.beta1) * grads
            self.v = self.beta2 * self.v + (1 - self.beta2) * (grads**2)
            # Compute bias-corrected moment estimates
            m_hat = self.m / (1 - self.beta1**self.t)
            v_hat = self.v / (1 - self.beta2**self.t)
            return self.learning_rate * m_hat / (np.sqrt(v_hat) + self.epsilon)

    adam = AdamOptimizer(learning_rate=learning_rate) if adam else None

    best_params = params
    max_fitness = float("-inf")
    for generation in range(max_generations):
        perturbations = np.random.randn(population_size, params.size)
        rewards = np.zeros(population_size)
        for i in range(population_size):
            perturbed_params = params + noise_stdev * perturbations[i]
            rewards[i] = fitness(perturbed_params)
            if rewards[i] > max_fitness:
                max_fitness = rewards[i]
                best_params = perturbed_params
                print(f"Generation {generation + 1}, Max Fitness {max_fitness}")

        if rank_transformation:
            ranks = np.empty(len(rewards), dtype=int)
            ranks[rewards.argsort()] = np.arange(len(rewards))
            ranks = ranks.astype(np.float32) / (len(ranks) - 1) - 0.5
            transformed_rewards = ranks
        else:
            transformed_rewards = (rewards - np.mean(rewards)) / (
                np.std(rewards) + 1e-8
            )

        gradient = np.dot(perturbations.T, transformed_rewards) / (
            population_size * noise_stdev
        )
        if adam:
            gradient = adam.update(gradient)
        else:
            gradient *= learning_rate
        params += gradient
    return best_params


if __name__ == "__main__":
    environment = Environment(two_agents=False)
    model = Model(18, 8, 2)

    def fitness(params):
        max_steps = 600
        model.set_params(params)
        loss = 0
        environment.reset()
        for _ in range(max_steps):
            agent = environment.agents[0].state
            ball = environment.ball
            environment.step(
                [
                    model.forward(
                        np.concatenate(
                            [
                                ball.position,
                                ball.velocity,
                                ball.orientation,
                                agent.position,
                                agent.velocity,
                                agent.orientation,
                            ]
                        )
                    ).squeeze()
                ]
            )
            loss -= 1
            if environment.scores[0] > 0:
                break
        return loss

    best_params = evolution_strategies(model.get_params(), fitness)

    model.set_params(best_params)

    def policy(observation):
        return model.forward(observation)

    play(policy)

# optimizer = ng.optimizers.registry["CMA"](
#     parametrization=model.n_params(), budget=10000
# )
# min_loss = float("inf")
# for _ in range(optimizer.budget):
#     x = optimizer.ask()
#     loss = -fitness(*x.args)
#     if loss < min_loss:
#         min_loss = loss
#         print(min_loss)
#     optimizer.tell(x, loss)

# mirrored sampling
# l2 norm
# print gradient
