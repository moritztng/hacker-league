#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>

#include <thread>
#include <chrono>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <queue>

#include "common.h"

struct Input
{
    uint32_t id;
    Player player;
};

struct Client
{
    sockaddr_in address;
    std::queue<Input> queue;
    bool regulateQueue;
    size_t playerId;
    std::chrono::steady_clock::time_point lastUpdate;
};

void receive(int &udpSocket, std::vector<Client> &clients)
{
    while (true)
    {
        char buffer[48];
        struct sockaddr_in clientAddress;
        socklen_t clientAddressLength = sizeof(clientAddress);
        int recvLength = recvfrom(udpSocket, buffer, sizeof(buffer), 0, (struct sockaddr *)&clientAddress, &clientAddressLength);
        if (recvLength == 0)
        {
            close(udpSocket);
            throw std::runtime_error("error receiving input");
        }

        Client *client = nullptr;
        for (Client &c : clients)
        {
            if (c.address.sin_addr.s_addr == clientAddress.sin_addr.s_addr && c.address.sin_port == clientAddress.sin_port)
            {
                client = &c;
                break;
            }
        }

        if (client == nullptr)
        {
            if (clients.size() < 2)
            {
                clients.push_back(Client{.address = clientAddress, .regulateQueue = true, .playerId = clients.size() == 0 ? 0 : clients[0].playerId ^ 1, .lastUpdate = std::chrono::steady_clock::now()});
                client = &clients.back();
            }
            else
            {
                continue;
            }
        }

        Input input;
        std::memcpy(&input.id, buffer, 4);
        std::memcpy(input.player.carState.position.data(), buffer + 4, 12);
        std::memcpy(input.player.carState.velocity.data(), buffer + 16, 12);
        std::memcpy(input.player.carState.orientation.data(), buffer + 28, 12);
        std::memcpy(&input.player.action.steering, buffer + 40, 4);
        std::memcpy(&input.player.action.throttle, buffer + 44, 4);
        client->queue.push(input);
        client->lastUpdate = std::chrono::steady_clock::now();
    }
}

int main(int argc, char *argv[])
{
    if (argc != 2)
    {
        std::cerr << "Usage: " << argv[0] << " <Port>" << std::endl;
        return EXIT_FAILURE;
    }

    int udpSocket = socket(AF_INET, SOCK_DGRAM, 0);
    if (udpSocket < 0)
    {
        std::cerr << "error creating udp socket" << std::endl;
        return EXIT_FAILURE;
    }

    struct sockaddr_in serverAddress;
    serverAddress.sin_family = AF_INET;
    serverAddress.sin_port = htons(std::stoi(argv[1]));
    serverAddress.sin_addr.s_addr = INADDR_ANY;

    if (bind(udpSocket, (struct sockaddr *)&serverAddress, sizeof(serverAddress)) < 0)
    {
        close(udpSocket);
        std::cerr << "error binding udp socket" << std::endl;
        return EXIT_FAILURE;
    }

    std::vector<Client> clients;
    std::thread receiveThread(&receive, std::ref(udpSocket), std::ref(clients));

    Eigen::Vector3f arenaSize = {100.0f, 20.0f, 200.0f};
    Sphere ball = {.objectState = {.position = {0.0f, 1.0f, 0.0f},
                                   .velocity = {0.0f, 0.0f, 0.0f},
                                   .orientation = {0.0f, 0.0f, 0.0f}},
                   .radius = 1.0f};
    Eigen::Vector2f goal = {20.0, 8.0};
    Eigen::Vector3f carSize = {1.25f, 0.75f, 2.f};
    std::vector<Player> players = {Player{.carState = {.position = {0.0f, 0.375f, 5.0f},
                                                       .velocity = {0.0f, 0.0f, 0.0f},
                                                       .orientation = {0.0f, 0.0f, 0.0f}},
                                          .action = {.throttle = 0.0f, .steering = 0.0f}},
                                   Player{.carState = {.position = {3.0f, 0.375f, 5.0f},
                                                       .velocity = {0.0f, 0.0f, 0.0f},
                                                       .orientation = {0.0f, 0.0f, 0.0f}},
                                          .action = {.throttle = 0.0f, .steering = 0.0f}}};

    constexpr uint FREQUENCY = 60;
    constexpr float PERIOD = 1.f / FREQUENCY;

    while (true)
    {
        auto start = std::chrono::high_resolution_clock::now();
        clients.erase(std::remove_if(clients.begin(), clients.end(),
                                     [](const Client &client)
                                     {
                                         return std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - client.lastUpdate).count() > 5;
                                     }),
                      clients.end());

        std::vector<std::tuple<sockaddr_in *, size_t, uint32_t>> clientPlayerInputIds;
        for (auto &[address, queue, regulateQueue, playerId, _] : clients)
        {
            Input &input = queue.front();
            players[playerId] = input.player;
            clientPlayerInputIds.push_back({&address, playerId, input.id});

            if (queue.empty() || queue.size() > 9)
            {
                regulateQueue = true;
            }
            else if (queue.size() == 5)
            {
                regulateQueue = false;
            }
            if (regulateQueue)
            {
                if (queue.size() > 5)
                {
                    queue.pop();
                    queue.pop();
                }
            }
            else
            {
                queue.pop();
            }
        }
        for (const auto &[address, playerId, inputId] : clientPlayerInputIds)
        {
            const size_t otherPlayer = playerId ^ 1;
            char buffer[84];
            std::memcpy(buffer, &inputId, 4);
            std::memcpy(buffer + 4, players[otherPlayer].carState.position.data(), 12);
            std::memcpy(buffer + 16, players[otherPlayer].carState.velocity.data(), 12);
            std::memcpy(buffer + 28, players[otherPlayer].carState.orientation.data(), 12);
            std::memcpy(buffer + 40, &players[otherPlayer].action.steering, 4);
            std::memcpy(buffer + 44, &players[otherPlayer].action.throttle, 4);
            std::memcpy(buffer + 48, ball.objectState.position.data(), 12);
            std::memcpy(buffer + 60, ball.objectState.velocity.data(), 12);
            std::memcpy(buffer + 72, ball.objectState.orientation.data(), 12);
            sendto(udpSocket, buffer, sizeof(buffer), 0, (sockaddr *)address, sizeof(sockaddr));
        }

        physicsStep(arenaSize, goal, ball, carSize, players);

        float elapsed = std::chrono::duration_cast<std::chrono::duration<float>>(std::chrono::high_resolution_clock::now() - start).count();
        if (elapsed < PERIOD)
        {
            std::this_thread::sleep_for(std::chrono::duration<float>(PERIOD - elapsed));
        }
    }
    close(udpSocket);
    return EXIT_SUCCESS;
}
