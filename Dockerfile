# Stage 1: Build stage
FROM debian:12.7 as build

RUN apt-get update && \
    apt-get install -y curl

# Download the server binary
RUN curl -L -o /server "https://github.com/moritztng/hacker-league/releases/latest/download/server_x86_64_debian"

# Stage 2: Final stage
FROM debian:12.7

ENV localport=""
ENV publicip=""
ENV publicport=""

RUN apt-get update && \
    apt-get install -y libglfw3 libcurl4-openssl-dev && \
    rm -rf /var/lib/apt/lists/*

# Copy the server binary from the build stage
COPY --from=build /server /usr/local/bin/server

ENTRYPOINT ["server", "${localport:-10000}", "${publicip}", "${publicport}"]
