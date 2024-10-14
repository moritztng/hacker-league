FROM debian:12.7

ENV localport=""
ENV publicip=""
ENV publicport=""

RUN apt-get update
RUN apt-get install -y libglfw3 libcurl4-openssl-dev

 RUN curl -L -o server "https://github.com/moritztng/hacker-league/releases/latest/download/server_x86_64_debian"

 ENTRYPOINT ["server", ${localport:-10000}, ${publicip}, ${publicport}]
