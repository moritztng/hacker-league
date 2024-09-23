CXXFLAGS = -std=c++17 -O3

.PHONY: debug release clean

all: debug

debug: hacker-league server addresses_server

release: CXXFLAGS += -DNDEBUG
release: hacker-league server addresses_server

hacker-league: main.cpp common.h shaders/world/vert.spv.h shaders/world/frag.spv.h shaders/hud/vert.spv.h shaders/hud/frag.spv.h font.h font.png
	g++ $(CXXFLAGS) -o hacker-league main.cpp -lvulkan -lglfw -lpthread

font.h font.png: font
	./font

font: font.cpp
	g++ $(CXXFLAGS) -o font font.cpp

shaders/world/%.spv.h: shaders/world/shader.%
	shaders/glslc $< -o ./shaders/world/$*.spv
	xxd -i ./shaders/world/$*.spv > $@

shaders/hud/%.spv.h: shaders/hud/shader.%
	shaders/glslc $< -o ./shaders/hud/$*.spv
	xxd -i ./shaders/hud/$*.spv > $@

server: server.cpp common.h
	g++ $(CXXFLAGS) -o server server.cpp -lpthread

addresses_server: addresses_server.cpp
	g++ $(CXXFLAGS) -o addresses_server addresses_server.cpp -lsqlite3

clean:
	rm -f hacker-league font font.h font.png server addresses_server shaders/*/*spv*
