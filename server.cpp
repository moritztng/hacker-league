#include <iostream>
#include <vector>
#include <optional>

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

void receive(int &udpSocket, std::vector<Client> &clients, bool &running)
{
    try
    {
        while (running)
        {
            char buffer[48];
            struct sockaddr_in6 clientAddress;
            socklen_t clientAddressLength = sizeof(clientAddress);
            int recvLength = recvfrom(udpSocket, buffer, sizeof(buffer), 0, (struct sockaddr *)&clientAddress, &clientAddressLength);
            if (recvLength == 0)
            {
                throw std::runtime_error("receiving input");
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
    catch (const std::exception &e)
    {
        std::cerr << "error: " << e.what() << std::endl;
    }
    running = false;
}

void publishServerAddress(std::string address, uint16_t port, std::vector<Client> &clients, bool &running)
{
    int tcpSocket = -1;
    try
    {
        constexpr int SERVER_PORT = 8080;
        constexpr uint16_t PERIOD = 30;
        const std::string addressesServerAddress = "hacker-league.molyz.app";

        const struct hostent *hostEntry = gethostbyname(addressesServerAddress.c_str());
        if (hostEntry == nullptr)
        {
            throw std::runtime_error("resolving hostname");
        }

        struct sockaddr_in serverAddress
        {
        };
        serverAddress.sin_family = AF_INET;
        std::memcpy(&serverAddress.sin_addr.s_addr, hostEntry->h_addr, hostEntry->h_length);
        serverAddress.sin_port = htons(SERVER_PORT);

        uint8_t time = PERIOD;
        size_t lastNPlayers = clients.size();
        while (running)
        {
            if (time < PERIOD && lastNPlayers == clients.size())
            {
                std::this_thread::sleep_for(std::chrono::seconds(1));
                time++;
                continue;
            }
            time = 0;
            lastNPlayers = clients.size();

            std::ostringstream bodyStream;
            bodyStream << "{ \"address\": \"" << address << "\", "
                       << "\"port\": " << port << ", "
                       << "\"nPlayers\": " << clients.size() << " }";
            std::string body = bodyStream.str();

            std::ostringstream httpRequestStream;
            httpRequestStream << "POST /publish HTTP/1.1\r\n"
                              << "Host: " << addressesServerAddress << "\r\n"
                              << "Content-Type: application/json\r\n"
                              << "Content-Length: " << body.size() << "\r\n\r\n"
                              << body;
            std::string httpRequest = httpRequestStream.str();

            tcpSocket = socket(AF_INET, SOCK_STREAM, 0);
            if (tcpSocket < 0)
            {
                throw std::runtime_error("creating tcp socket");
            }

            if (connect(tcpSocket, (struct sockaddr *)&serverAddress, sizeof(serverAddress)) < 0)
            {
                throw std::runtime_error("connecting to the server");
            }

            if (send(tcpSocket, httpRequest.c_str(), httpRequest.size(), 0) < 0)
            {
                throw std::runtime_error("sending message");
            }

            char buffer[4096];
            const int bytesReceived = recv(tcpSocket, buffer, sizeof(buffer), 0);
            if (bytesReceived < 0)
            {
                throw std::runtime_error("receiving response");
            }
            close(tcpSocket);
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "error: " << e.what() << std::endl;
    }
    if (tcpSocket > -1)
    {
        close(tcpSocket);
    }
    running = false;
}

int main(int argc, char *argv[])
{
    bool running = true;
    int udpSocket = -1;
    std::optional<std::thread> receiveThread, tcpThread;
    try
    {
        if (!(argc == 2 || argc == 4))
        {
            throw std::runtime_error("wrong number of arguments");
        }

        udpSocket = socket(AF_INET6, SOCK_DGRAM, 0);
        if (udpSocket < 0)
        {
            throw std::runtime_error("creating udp socket");
        }
        int opt = 0;
        setsockopt(udpSocket, IPPROTO_IPV6, IPV6_V6ONLY, &opt, sizeof(opt));

        struct sockaddr_in6 serverAddress = {};
        serverAddress.sin6_family = AF_INET6;
        serverAddress.sin6_addr = in6addr_any;
        serverAddress.sin6_port = htons(std::stoi(argv[1]));

        if (bind(udpSocket, (struct sockaddr *)&serverAddress, sizeof(serverAddress)) < 0)
        {
            throw std::runtime_error("binding udp socket");
        }

        std::vector<Client> clients;
        receiveThread = std::thread(&receive, std::ref(udpSocket), std::ref(clients), std::ref(running));
        if (argc == 4)
        {
            tcpThread = std::thread(&publishServerAddress, argv[2], std::stoi(argv[3]), std::ref(clients), std::ref(running));
        }

        Sphere ball = initialBall;
        std::vector<Player> players = initialPlayers;
        uint8_t scores[2];

        constexpr uint FREQUENCY = 60;
        constexpr float PERIOD = 1.f / FREQUENCY;

        constexpr uint16_t GAME_DURATION = 300;
        constexpr uint16_t TRANSITION_DURATION = 5;
        constexpr uint8_t QUEUE_MIN = 1;
        constexpr uint8_t QUEUE_MAX = 10;
        constexpr uint8_t QUEUE_TARGET = (QUEUE_MIN + QUEUE_MAX) / 2;

        constexpr uint8_t MAX_TIME_CLIENT_IDLE = 5;

        int64_t startTime = 0;
        int64_t transitionTime = 0;

        const auto period = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::duration<float>(PERIOD));
        auto targetTime = std::chrono::high_resolution_clock::now();
        while (running)
        {
            const int64_t currentTime = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();

            if (clients.size() == 2)
            {
                if (currentTime - startTime > GAME_DURATION)
                {
                    startTime = currentTime + TRANSITION_DURATION;
                    transitionTime = currentTime;
                    ball = initialBall;
                    scores[0] = 0;
                    scores[1] = 0;
                }
            }
            else
            {
                startTime = 0;
            }

            clients.erase(std::remove_if(clients.begin(), clients.end(),
                                         [](const Client &client)
                                         {
                                             return std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - client.lastUpdate).count() > MAX_TIME_CLIENT_IDLE;
                                         }),
                          clients.end());

            std::vector<std::tuple<sockaddr_in6 *, size_t, uint32_t>> clientPlayerInputIds;
            for (auto &[address, queue, regulateQueue, playerId, _] : clients)
            {
                if (queue.size() < QUEUE_MIN || queue.size() > QUEUE_MAX)
                {
                    regulateQueue = true;
                }
                else if (queue.size() == QUEUE_TARGET)
                {
                    regulateQueue = false;
                }
                if (!regulateQueue || queue.size() > QUEUE_MAX)
                {
                    const Input &input = queue.front();
                    players[playerId] = input.player;
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
                std::memcpy(buffer + 100, &scores, 2);
                sendto(udpSocket, buffer, sizeof(buffer), 0, (sockaddr *)address, sizeof(*address));
            }

            const uint8_t oldScore = scores[0] + scores[1];
            physicsStep(arenaSize, goal, ball, carSize, players, true, scores);
            if (scores[0] + scores[1] != oldScore)
            {
                transitionTime = currentTime;
                startTime += TRANSITION_DURATION;
            }

            targetTime += period;
            std::this_thread::sleep_until(targetTime);
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "error: " << e.what() << std::endl;
    }
    running = false;
    if (tcpThread)
        tcpThread->join();
    if (receiveThread)
        receiveThread->join();
    if (udpSocket > -1)
        close(udpSocket);
    return EXIT_SUCCESS;
}
