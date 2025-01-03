import socket, struct, subprocess, os, requests, tempfile
import numpy as np
from . import hacker_league_physics


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


def play(policy):
    if os.path.exists("/etc/debian_version"):
        distro = "debian"
    elif os.path.exists("/etc/arch-release"):
        distro = "arch"
    else:
        raise RuntimeError("os not supported")

    binary_name = f"hacker-league_x86_64_{distro}"

    temp_dir_path = os.path.join(tempfile.gettempdir(), "hacker_league")
    os.makedirs(temp_dir_path, exist_ok=True)

    addresses = [
        "https://github.com/moritztng/hacker-league/releases/latest/download/" + binary_name,
        "https://raw.githubusercontent.com/moritztng/hacker-league/refs/heads/main/gamepad.txt",
        "https://raw.githubusercontent.com/moritztng/hacker-league/refs/heads/main/font.png",
    ]
    for address in addresses:
        file_name = address.split("/")[-1]
        file_path = os.path.join(temp_dir_path, file_name)
        if not os.path.exists(file_path):
            with open(file_path, "wb") as file:
                file.write(requests.get(address).content)
            if "hacker-league" in file_name:
                os.chmod(file_path, 0o755)

    game_process = subprocess.Popen(
        [os.path.join(temp_dir_path, binary_name), "environment"],
        env={**os.environ, 'HACKER_LEAGUE': temp_dir_path},
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
