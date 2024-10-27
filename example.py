import hacker_league
import numpy as np


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


environment = hacker_league.Environment(two_agents=False)
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


hacker_league.play(policy)
