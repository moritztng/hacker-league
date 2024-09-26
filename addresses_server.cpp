#include <iostream>
#include <unistd.h>
#include <sstream>
#include <cstring>
#include <sqlite3.h>
#include <netinet/in.h>
#include <poll.h>

constexpr int PORT = 8080;
constexpr int MAX_CLIENTS = 100;
constexpr int TIMEOUT_SECONDS = 60;

int main()
{
    sqlite3 *db;

    if (sqlite3_open("servers.db", &db) != SQLITE_OK)
    {
        std::cerr << "can't open database: " << sqlite3_errmsg(db) << std::endl;
        return EXIT_FAILURE;
    }

    const char *createTableQuery = R"(
        CREATE TABLE IF NOT EXISTS servers (
            address TEXT NOT NULL,
            port TEXT NOT NULL,
            time INTEGER NOT NULL,
            n_players INTEGER NOT NULL,
            PRIMARY KEY (address, port)
        );
    )";

    char *errorMessage = nullptr;
    if (sqlite3_exec(db, createTableQuery, nullptr, nullptr, &errorMessage) != SQLITE_OK)
    {
        std::cerr << "error creating table: " << errorMessage << std::endl;
        sqlite3_free(errorMessage);
    }

    int serverSocket = socket(AF_INET6, SOCK_STREAM, 0);
    if (serverSocket < 0)
    {
        std::cerr << "error creating socket" << std::endl;
        return EXIT_FAILURE;
    }

    int opt = 1;
    setsockopt(serverSocket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in6 serverAddress
    {
    };
    serverAddress.sin6_family = AF_INET6;
    serverAddress.sin6_addr = in6addr_any;
    serverAddress.sin6_port = htons(PORT);

    if (bind(serverSocket, (struct sockaddr *)&serverAddress, sizeof(serverAddress)) < 0)
    {
        std::cerr << "error binding socket" << std::endl;
        close(serverSocket);
        return EXIT_FAILURE;
    }

    if (listen(serverSocket, MAX_CLIENTS) < 0)
    {
        std::cerr << "error listening on socket" << std::endl;
        close(serverSocket);
        return EXIT_FAILURE;
    }

    struct pollfd fds[MAX_CLIENTS + 1];
    int nfds = 1;
    fds[0].fd = serverSocket;
    fds[0].events = POLLIN;

    while (true)
    {
        int activity = poll(fds, nfds, TIMEOUT_SECONDS * 1000);

        if (activity < 0)
        {
            std::cerr << "poll error" << std::endl;
            return EXIT_FAILURE;
        }

        if (fds[0].revents & POLLIN)
        {
            struct sockaddr_in6 clientAddress;
            socklen_t clientLength = sizeof(clientAddress);
            int clientSocket = accept(serverSocket, (struct sockaddr *)&clientAddress, &clientLength);
            if (clientSocket < 0)
            {
                std::cerr << "error accepting connection" << std::endl;
                continue;
            }

            if (nfds < MAX_CLIENTS + 1)
            {
                fds[nfds].fd = clientSocket;
                fds[nfds].events = POLLIN;
                nfds++;
                std::cout << "client connected" << std::endl;
            }
            else
            {
                std::cerr << "maximum clients reached, rejecting connection" << std::endl;
                close(clientSocket);
            }
        }

        for (int i = 1; i < nfds; i++)
        {
            if (fds[i].revents & POLLIN)
            {
                char buffer[4096] = {};
                std::string request;
                ssize_t bytesRead = 0;
                size_t headerSize = 0;
                while (true)
                {
                    bytesRead = recv(fds[i].fd, buffer, sizeof(buffer) - 1, 0);
                    if (bytesRead == 0)
                    {
                        break;
                    }

                    buffer[bytesRead] = '\0';
                    request.append(buffer);

                    if ((headerSize = request.find("\r\n\r\n")) != std::string::npos)
                    {
                        headerSize += 4;
                        break;
                    }
                }

                if (bytesRead == 0)
                {
                    close(fds[i].fd);
                    nfds--;
                    std::cout << "client disconnected" << std::endl;
                    continue;
                }

                size_t contentLength = 0;
                size_t pos = request.find("Content-Length: ");
                if (pos != std::string::npos)
                {
                    pos += strlen("Content-Length: ");
                    const size_t endPos = request.find("\r\n", pos);
                    if (endPos != std::string::npos)
                    {
                        contentLength = std::stoul(request.substr(pos, endPos - pos));
                    }
                }

                if (contentLength > 0)
                {
                    const size_t totalBytes = headerSize + contentLength;
                    while (request.size() < totalBytes)
                    {
                        bytesRead = recv(fds[i].fd, buffer, sizeof(buffer) - 1, 0);
                        if (bytesRead == 0)
                        {
                            break;
                        }

                        buffer[bytesRead] = '\0';
                        request.append(buffer);
                    }
                }

                if (bytesRead == 0)
                {
                    close(fds[i].fd);
                    nfds--;
                    std::cout << "client disconnected" << std::endl;
                    continue;
                }

                if (request.find("GET /servers") == 0)
                {
                    const char *query = "SELECT address, port, n_players FROM servers WHERE time >= strftime('%s', 'now') - 60;";
                    sqlite3_stmt *stmt;

                    if (sqlite3_prepare_v2(db, query, -1, &stmt, nullptr) != SQLITE_OK)
                    {
                        std::cerr << "error preparing statement: " << sqlite3_errmsg(db) << std::endl;
                        continue;
                    }

                    std::ostringstream responseStream;
                    responseStream << "[";

                    bool firstElement = true;
                    while (sqlite3_step(stmt) == SQLITE_ROW)
                    {
                        const char *address = reinterpret_cast<const char *>(sqlite3_column_text(stmt, 0));
                        const char *port = reinterpret_cast<const char *>(sqlite3_column_text(stmt, 1));
                        const char *nPlayers = reinterpret_cast<const char *>(sqlite3_column_text(stmt, 2));

                        if (!firstElement)
                        {
                            responseStream << ",";
                        }
                        responseStream << "{\"address\": \"" << address << "\", \"port\": \"" << port << "\", \"nPlayers\": \"" << nPlayers << "\"}";
                        firstElement = false;
                    }

                    sqlite3_finalize(stmt);
                    responseStream << "]";

                    const std::string responseBody = responseStream.str();
                    const std::string responseHeaders = "HTTP/1.1 200 OK\r\n"
                                                        "Content-Type: application/json\r\n"
                                                        "Content-Length: " +
                                                        std::to_string(responseBody.size()) + "\r\n"
                                                                                              "Connection: close\r\n"
                                                                                              "\r\n";
                    const std::string response = responseHeaders + responseBody;
                    const ssize_t bytesWritten = send(fds[i].fd, response.c_str(), response.size(), 0);
                    if (bytesWritten < 0)
                    {
                        std::cerr << "error sending response: " << strerror(errno) << std::endl;
                    }

                    close(fds[i].fd);
                    nfds--;
                }
                else if (request.find("POST /publish") == 0)
                {
                    std::istringstream iss(request.substr(headerSize));
                    std::string address, port, nPlayers;

                    if (!(iss >> address >> port >> nPlayers))
                    {
                        std::cerr << "Could not separate ip, port, and nPlayers" << std::endl;
                        continue;
                    }

                    std::cout << "received server: address: " << address << " port: " << port << " number of player: " << nPlayers << std::endl;

                    const std::string query = "INSERT OR REPLACE INTO servers (address, port, time, n_players) VALUES (?, ?, strftime('%s', 'now'), ?);";
                    sqlite3_stmt *stmt;
                    sqlite3_prepare_v2(db, query.c_str(), -1, &stmt, nullptr);

                    sqlite3_bind_text(stmt, 1, address.c_str(), -1, SQLITE_STATIC);
                    sqlite3_bind_text(stmt, 2, port.c_str(), -1, SQLITE_STATIC);
                    sqlite3_bind_text(stmt, 3, nPlayers.c_str(), -1, SQLITE_STATIC);

                    if (sqlite3_step(stmt) != SQLITE_DONE)
                    {
                        std::cerr << "error inserting or updating client: " << sqlite3_errmsg(db) << std::endl;
                        continue;
                    }
                    sqlite3_finalize(stmt);
                    std::string response = "HTTP/1.1 204 No Content\r\n"
                                           "Content-Length: 0\r\n"
                                           "\r\n";

                    send(fds[i].fd, response.c_str(), response.size(), 0);
                    close(fds[i].fd);
                    nfds--;
                }
            }
        }
    }
    close(serverSocket);
    sqlite3_close(db);
    return EXIT_SUCCESS;
}
