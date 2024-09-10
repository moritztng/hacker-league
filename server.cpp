#include <iostream>
#include <thread>
#include <chrono>
#include <vector>
#include <mutex>
#include <cstring>
#include <netinet/in.h> // For socket programming on Linux/Unix systems (use Winsock for Windows)
#include <sys/socket.h>
#include <arpa/inet.h> // For htons, ntohs, inet_addr, etc.
#include <unistd.h>
#include <eigen3/Eigen/Dense>

struct ObjectState
{
    Eigen::Vector3f position;
    Eigen::Vector3f velocity;
    Eigen::Vector3f orientation;
};

struct Sphere
{
    ObjectState objectState;
    float radius;
};

struct Box
{
    ObjectState objectState;
    Eigen::Vector3f size;
};

struct Action
{
    float throttle;
    float steering;
    bool ballCamPressed;
    bool close;
};

struct Player
{
    ObjectState carState;
    Action action;
};

struct Input
{
    Action action;
    bool ballCamPressed;
    bool close;
};

struct State
{
    uint32_t id;
    uint32_t statesBehind;
    Box arena;
    Sphere ball;
    Eigen::Vector2f goal;
    Eigen::Vector3f carSize;
    std::array<Player, 2> players;
    uint8_t player;
    Input input;
    bool ballCam;
};

int main()
{
    State state{
        .id = 0,
        .statesBehind = 0,
        .arena = {.objectState = {.position = {0.0f, 10.0f, 0.0f},
                                  .velocity = {0.0f, 0.0f, 0.0f},
                                  .orientation = {0.0f, 0.0f, 0.0f}},
                  .size = {100.0f, 20.0f, 200.0f}},
        .ball = {.objectState = {.position = {0.0f, 1.0f, 0.0f},
                                 .velocity = {0.0f, 0.0f, 0.0f},
                                 .orientation = {0.0f, 0.0f, 0.0f}},
                 .radius = 1.0f},
        .goal = {20.0, 8.0},
        .carSize = {1.25f, 0.75f, 2.f},
        .players = {Player{.carState = {.position = {0.0f, 0.375f, 5.0f},
                                        .velocity = {0.0f, 0.0f, 0.0f},
                                        .orientation = {0.0f, 0.0f, 0.0f}},
                           .action = {.throttle = 0.0f, .steering = 0.0f}},
                    Player{.carState = {.position = {3.0f, 0.375f, 5.0f},
                                        .velocity = {0.0f, 0.0f, 0.0f},
                                        .orientation = {0.0f, 0.0f, 0.0f}},
                           .action = {.throttle = 1.0f, .steering = 0.0f}}},
        .player = 0,
        .input = {.action = {.throttle = 0.0f, .steering = 0.0f}, .ballCamPressed = false, .close = false},
        .ballCam = true};
    int udpSocket = socket(AF_INET, SOCK_DGRAM, 0);
    if (udpSocket < 0)
    {
        throw std::runtime_error("Failed to create UDP socket!");
    }

    struct sockaddr_in serverAddress;
    serverAddress.sin_family = AF_INET;
    serverAddress.sin_port = htons(12345);
    serverAddress.sin_addr.s_addr = INADDR_ANY;

    if (bind(udpSocket, (struct sockaddr *)&serverAddress, sizeof(serverAddress)) < 0)
    {
        close(udpSocket);
        throw std::runtime_error("Failed to bind UDP socket!");
    }

    while (true)
    {
        State clientState;
        struct sockaddr clientAddress;
        socklen_t clientAddressLength;
        int recvLength = recvfrom(udpSocket, &clientState, sizeof(clientState), 0, &clientAddress, &clientAddressLength);
        if (recvLength == 0)
        {
            close(udpSocket);
            throw std::runtime_error("error reading");
        }
        state.players[clientState.player] = clientState.players[clientState.player];
        for (int i = 0; i < clientState.players.size(); i++)
        {
            if (i != clientState.player)
            {
                std::cout << state.players[i].action.throttle << std::endl;
                clientState.players[i] = state.players[i];
            }
        }
        clientState.ball = state.ball;
        sendto(udpSocket, &clientState, sizeof(clientState), 0, &clientAddress, clientAddressLength);

        constexpr uint FREQUENCY = 60;
        constexpr float PERIOD = 1.f / FREQUENCY;
        constexpr float MAX_ACCELERATION = 30;
        constexpr float CAR_FRICTION = 10;
        constexpr float BALL_FRICTION = 5;
        constexpr float TURN_RADIUS_MIN = 0.5;
        constexpr float TURN_RADIUS_RATE = 0.5;
        constexpr float MAX_SPEED = 50;
        constexpr float GRAVITY = 3;
        constexpr float BALL_RESTITUTION = 0.6;
        constexpr float MAX_DELTA_SPEED = MAX_ACCELERATION * PERIOD;
        constexpr float DELTA_CAR_FRICTION = CAR_FRICTION * PERIOD;
        constexpr float DELTA_BALL_FRICTION = BALL_FRICTION * PERIOD;
        constexpr float DELTA_GRAVITY = GRAVITY * PERIOD;
        // car
        // acceleration
        Eigen::Vector3f halfArenaSize = state.arena.size / 2.f;
        Eigen::Vector3f halfCarSize = state.carSize / 2.f;
        std::vector<Eigen::Vector2f> localCorners = {
            {-halfCarSize.x(), -halfCarSize.z()},
            {halfCarSize.x(), -halfCarSize.z()},
            {-halfCarSize.x(), halfCarSize.z()},
            {halfCarSize.x(), halfCarSize.z()}};
        for (auto &[carState, action] : state.players)
        {
            Eigen::Vector3f orientationVector = {std::sin(carState.orientation.y()), 0.0f, std::cos(carState.orientation.y())};
            carState.velocity += orientationVector * MAX_DELTA_SPEED * action.throttle;
            float speed = carState.velocity.norm();
            if (speed > DELTA_CAR_FRICTION)
            {
                carState.velocity -= carState.velocity.normalized() * DELTA_CAR_FRICTION;
                speed -= DELTA_CAR_FRICTION;
                if (speed > MAX_SPEED)
                {
                    carState.velocity *= MAX_SPEED / speed;
                    speed = MAX_SPEED;
                }
            }
            else
            {
                carState.velocity.setZero();
                speed = 0;
            }
            // steering
            int backwards = carState.velocity.dot(orientationVector) < 0 ? -1 : 1;
            carState.orientation.y() -= backwards * action.steering * speed / (speed * TURN_RADIUS_RATE + TURN_RADIUS_MIN) * PERIOD;
            carState.velocity = backwards * Eigen::Vector3f(std::sin(carState.orientation.y()), 0.0f, std::cos(carState.orientation.y())) * speed;
            // wall collision
            // TODO: make more efficient
            for (const auto &localCorner : localCorners)
            {
                Eigen::Vector2f globalCorner = Eigen::Rotation2Df(carState.orientation.y()).toRotationMatrix() * localCorner + Eigen::Vector2f(carState.position.x(), carState.position.z());
                float xDistance = std::abs(globalCorner.x()) - halfArenaSize.x();
                float zDistance = std::abs(globalCorner.y()) - halfArenaSize.z();
                if (xDistance > 0)
                {
                    int leftRight = globalCorner.x() > 0 ? 1 : -1;
                    carState.position.x() -= leftRight * (xDistance + 0.001f);
                    if (leftRight * carState.velocity.x() > 0)
                    {
                        carState.velocity.x() = 0;
                    }
                }
                if (zDistance > 0)
                {
                    int backFront = globalCorner.y() > 0 ? 1 : -1;
                    carState.position.z() -= backFront * (zDistance + 0.001f);
                    if (backFront * carState.velocity.z() > 0)
                    {
                        carState.velocity.z() = 0;
                    }
                }
            }

            // car ball collision
            Eigen::AngleAxisf::Matrix3 rotation = Eigen::AngleAxisf(carState.orientation.y(), Eigen::Vector3f::UnitY()).toRotationMatrix();
            Eigen::Vector3f localBallPosition = rotation.transpose() * (state.ball.objectState.position - carState.position);
            Eigen::Vector3f difference = localBallPosition.cwiseMax(-halfCarSize).cwiseMin(halfCarSize) - localBallPosition;
            float distance = difference.norm();
            if (distance < state.ball.radius)
            {
                state.ball.objectState.position -= rotation * difference * (state.ball.radius / distance - 1);
                Eigen::Vector3f collisionNormal = (state.ball.objectState.position - carState.position).normalized();
                float velocityAlongNormal = (state.ball.objectState.velocity - carState.velocity).dot(collisionNormal);
                if (velocityAlongNormal < 0)
                {
                    state.ball.objectState.velocity -= 1 * velocityAlongNormal * collisionNormal;
                }
            }
        }
        // ball
        // vertical
        if (state.ball.objectState.position.y() < state.ball.radius)
        {
            state.ball.objectState.position.y() = state.ball.radius;
            state.ball.objectState.velocity.y() *= state.ball.objectState.velocity.y() < -0.1 ? -BALL_RESTITUTION : 0;
        }
        else if (state.ball.objectState.position.y() > state.ball.radius)
        {
            state.ball.objectState.velocity.y() -= DELTA_GRAVITY;
            if (state.ball.objectState.position.y() > state.arena.size.y() - state.ball.radius)
            {
                state.ball.objectState.position.y() = state.arena.size.y() - state.ball.radius;
                state.ball.objectState.velocity.y() *= -1;
            }
        }
        // friction
        if (state.ball.objectState.position.y() == state.ball.radius)
        {
            Eigen::Vector3f &velocity = state.ball.objectState.velocity;
            float scale = std::max(1 - DELTA_BALL_FRICTION / std::hypot(velocity.x(), velocity.z()), 0.f);
            velocity.x() *= scale;
            velocity.z() *= scale;
        }
        // side walls
        if (halfArenaSize.x() - abs(state.ball.objectState.position.x()) < state.ball.radius)
        {
            state.ball.objectState.position.x() = (state.ball.objectState.position.x() < 0 ? -1 : 1) * (halfArenaSize.x() - state.ball.radius);
            state.ball.objectState.velocity.x() *= -1;
        }
        // front + back wall
        if (halfArenaSize.z() - abs(state.ball.objectState.position.z()) < state.ball.radius && (state.ball.objectState.position.y() > state.goal.y() || abs(state.ball.objectState.position.x()) > state.goal.x() / 2))
        {
            state.ball.objectState.position.z() = (state.ball.objectState.position.z() < 0 ? -1 : 1) * (halfArenaSize.z() - state.ball.radius);
            state.ball.objectState.velocity.z() *= -1;
        }
        // goal
        if (abs(state.ball.objectState.position.z()) > state.arena.size.z() / 2 + state.ball.radius)
        {
            state.ball.objectState.position.setZero();
            state.ball.objectState.velocity.setZero();
        }
        // goal posts + crossbar
        for (int i = 0; i < 2; ++i)
        {
            if ((i == 0) ? state.ball.objectState.position.y() < state.goal.y() - state.ball.radius : abs(state.ball.objectState.position.x()) < state.goal.x() / 2 - state.ball.radius)
            {
                Eigen::Vector3f difference = (i == 0) ? Eigen::Vector3f(state.goal.x() / 2 - abs(state.ball.objectState.position.x()), 0.f, state.arena.size.z() / 2 - abs(state.ball.objectState.position.z())) : Eigen::Vector3f(0.f, state.goal.y() - state.ball.objectState.position.y(), state.arena.size.z() / 2 - abs(state.ball.objectState.position.z()));
                float distance = difference.norm();
                if (distance < state.ball.radius)
                {
                    Eigen::Vector3f adjustedDifference = difference.cwiseProduct(state.ball.objectState.position.cwiseSign());
                    state.ball.objectState.position -= adjustedDifference * (state.ball.radius / distance - 1);
                    Eigen::Vector3f normal = adjustedDifference / distance;
                    state.ball.objectState.velocity -= 2 * state.ball.objectState.velocity.dot(normal) * normal;
                }
            }
        }

        for (auto &[carState, _] : state.players)
            carState.position += carState.velocity * PERIOD;
        state.ball.objectState.position += state.ball.objectState.velocity * PERIOD;

        state.id++;
    }

    close(udpSocket);
    return EXIT_SUCCESS;
}
