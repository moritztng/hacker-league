#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>

#include <thread>
#include <chrono>
#include <netinet/in.h>
#include <sys/socket.h>
#include <netdb.h>
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
    sockaddr_in6 address;
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
        struct sockaddr_in6 clientAddress;
        socklen_t clientAddressLength = sizeof(clientAddress);
        int recvLength = recvfrom(udpSocket, buffer, sizeof(buffer), 0, (struct sockaddr *)&clientAddress, &clientAddressLength);
        if (recvLength == 0)
        {
            close(udpSocket);
            // TODO: catch exceptions
            throw std::runtime_error("error receiving input");
        }

        Client *client = nullptr;
        for (Client &c : clients)
        {
            if (memcmp(&c.address, &clientAddress, sizeof(clientAddress)) == 0)
            {
                client = &c;
                break;
            }
        }

        if (client == nullptr)
        {
            if (clients.size() < 2)
            {
                const uint8_t playerId = clients.size() == 0 ? 0 : clients[0].playerId ^ 1;
                clients.push_back(Client{.address = clientAddress, .regulateQueue = true, .playerId = playerId, .lastUpdate = std::chrono::steady_clock::now()});
                // TODO: deal with packet loss
                sendto(udpSocket, &playerId, sizeof(playerId), 0, (sockaddr *)&clientAddress, sizeof(clientAddress));
            }
            else
            {
                continue;
            }
        }
        else
        {
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
}

void publishServerAddress(std::string address, uint16_t port)
{
    constexpr int SERVER_PORT = 8080;
    constexpr uint16_t PERIOD = 30;
    const std::string addressesServerAddress = "hacker-league.molyz.app";

    const struct hostent *hostEntry = gethostbyname(addressesServerAddress.c_str());
    if (hostEntry == nullptr)
    {
        throw std::runtime_error("error resolving hostname");
    }

    struct sockaddr_in serverAddress
    {
    };
    serverAddress.sin_family = AF_INET;
    std::memcpy(&serverAddress.sin_addr.s_addr, hostEntry->h_addr, hostEntry->h_length);
    serverAddress.sin_port = htons(SERVER_PORT);

    const std::string body = address + " " + std::to_string(port);
    std::string httpRequest = "POST /publish HTTP/1.1\r\n"
                              "Host: " +
                              addressesServerAddress + "\r\n"
                                                       "Content-Type: text/plain\r\n"
                                                       "Content-Length: " +
                              std::to_string(body.size()) + "\r\n\r\n" +
                              body;

    while (true)
    {
        int tcpSocket = socket(AF_INET, SOCK_STREAM, 0);
        if (tcpSocket < 0)
        {
            throw std::runtime_error("error creating tcp socket");
        }

        if (connect(tcpSocket, (struct sockaddr *)&serverAddress, sizeof(serverAddress)) < 0)
        {
            close(tcpSocket);
            throw std::runtime_error("error connecting to the server");
        }

        if (send(tcpSocket, httpRequest.c_str(), httpRequest.size(), 0) < 0)
        {
            close(tcpSocket);
            throw std::runtime_error("error sending message");
        }

        char buffer[4096];
        const int bytesReceived = recv(tcpSocket, buffer, sizeof(buffer), 0);
        if (bytesReceived < 0)
        {
            throw std::runtime_error("error receiving response");
        }
        close(tcpSocket);

        std::this_thread::sleep_for(std::chrono::seconds(PERIOD));
    }
}

int main(int argc, char *argv[])
{
    if (!(argc == 2 || argc == 4))
    {
        std::cerr << "Usage\nLocal: " << argv[0] << " <Port>\nPublic: " << argv[0] << " <Port> <PublicAddress> <PublicPort>" << std::endl;
        return EXIT_FAILURE;
    }

    int udpSocket = socket(AF_INET6, SOCK_DGRAM, 0);
    if (udpSocket < 0)
    {
        std::cerr << "error creating udp socket" << std::endl;
        return EXIT_FAILURE;
    }
    int opt = 0;
    setsockopt(udpSocket, IPPROTO_IPV6, IPV6_V6ONLY, &opt, sizeof(opt));

    struct sockaddr_in6 serverAddress = {};
    serverAddress.sin6_family = AF_INET6;
    serverAddress.sin6_addr = in6addr_any;
    serverAddress.sin6_port = htons(std::stoi(argv[1]));

    if (bind(udpSocket, (struct sockaddr *)&serverAddress, sizeof(serverAddress)) < 0)
    {
        close(udpSocket);
        std::cerr << "error binding udp socket" << std::endl;
        return EXIT_FAILURE;
    }

    std::vector<Client> clients;
    std::thread receiveThread(&receive, std::ref(udpSocket), std::ref(clients));
    std::thread tcpThread;
    if (argc == 4)
    {
        tcpThread = std::thread(&publishServerAddress, argv[2], std::stoi(argv[3]));
    }

    Sphere ball = initialBall;
    std::vector<Player> players = initialPlayers;

    constexpr uint FREQUENCY = 60;
    constexpr float PERIOD = 1.f / FREQUENCY;

    constexpr uint16_t GAME_DURATION = 300;
    constexpr uint16_t TRANSITION_DURATION = 5;
    constexpr uint8_t QUEUE_MIN = 1;
    constexpr uint8_t QUEUE_MAX = 10;
    constexpr uint8_t QUEUE_TARGET = (QUEUE_MIN + QUEUE_MAX) / 2;

    int64_t startTime = 0;
    int64_t transitionTime = 0;

    const auto period = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::duration<float>(PERIOD));
    auto targetTime = std::chrono::high_resolution_clock::now();
    // TODO: proper signal handling
    while (true)
    {
        const int64_t currentTime = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();

        if (clients.size() == 2)
        {
            if (currentTime - startTime > GAME_DURATION)
            {
                startTime = currentTime + 5;
                transitionTime = currentTime;
                ball = initialBall;
                players[0].score = 0;
                players[1].score = 0;
            }
        }
        else
        {
            startTime = 0;
        }

        clients.erase(std::remove_if(clients.begin(), clients.end(),
                                     [](const Client &client)
                                     {
                                         return std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - client.lastUpdate).count() > 5;
                                     }),
                      clients.end());

        std::vector<std::tuple<sockaddr_in6 *, size_t, uint32_t>> clientPlayerInputIds;
        for (auto &[address, queue, regulateQueue, playerId, _] : clients)
        {
            if (queue.size() < QUEUE_MIN || queue.size() > QUEUE_MAX)
            {
                regulateQueue = true;
            }
            else if (queue.size() == (QUEUE_MIN + QUEUE_MAX) / 2)
            {
                regulateQueue = false;
            }
            if (!regulateQueue || queue.size() > QUEUE_MAX)
            {
                const Input &input = queue.front();
                players[playerId].action = input.player.action;
                players[playerId].carState = input.player.carState;
                clientPlayerInputIds.push_back({&address, playerId, input.id});
                for (int i = 0; i < (regulateQueue ? queue.size() - QUEUE_TARGET : 1); i++)
                    queue.pop();
            }
        }

        for (const auto &[address, playerId, inputId] : clientPlayerInputIds)
        {
            const size_t otherPlayer = playerId ^ 1;
            int64_t countdown = GAME_DURATION - currentTime + startTime;
            int64_t transitionCountdown = TRANSITION_DURATION - currentTime + transitionTime;
            if (transitionCountdown < 0)
                transitionCountdown = 0;
            if (countdown < 0)
                countdown = 0;
            char buffer[102];
            std::memcpy(buffer, &inputId, 4);
            std::memcpy(buffer + 4, players[otherPlayer].carState.position.data(), 12);
            std::memcpy(buffer + 16, players[otherPlayer].carState.velocity.data(), 12);
            std::memcpy(buffer + 28, players[otherPlayer].carState.orientation.data(), 12);
            std::memcpy(buffer + 40, &players[otherPlayer].action.steering, 4);
            std::memcpy(buffer + 44, &players[otherPlayer].action.throttle, 4);
            std::memcpy(buffer + 48, ball.objectState.position.data(), 12);
            std::memcpy(buffer + 60, ball.objectState.velocity.data(), 12);
            std::memcpy(buffer + 72, ball.objectState.orientation.data(), 12);
            std::memcpy(buffer + 84, &countdown, 8);
            std::memcpy(buffer + 92, &transitionCountdown, 8);
            std::memcpy(buffer + 100, &players[0].score, 1);
            std::memcpy(buffer + 101, &players[1].score, 1);
            sendto(udpSocket, buffer, sizeof(buffer), 0, (sockaddr *)address, sizeof(*address));
        }

        const uint8_t scores[2] = {players[0].score, players[1].score};
        physicsStep(arenaSize, goal, ball, carSize, players, true);
        if (players[0].score != scores[0] || players[1].score != scores[1])
        {
            transitionTime = currentTime;
            startTime += 5;
        }

        targetTime += period;
        std::this_thread::sleep_until(targetTime);
    }

    close(udpSocket);
    return EXIT_SUCCESS;
}
