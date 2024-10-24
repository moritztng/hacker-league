CXXFLAGS = -std=c++17 -O3
GAME_SRC = main.cpp

.PHONY: debug release hacker-league_profile clean

all: debug

debug: hacker-league server

release: CXXFLAGS += -DNDEBUG
release: hacker-league server

hacker-league_profile: CXXFLAGS += -DTRACY_ENABLE -Itracy/public
hacker-league_profile: GAME_SRC += tracy/public/TracyClient.cpp
hacker-league_profile: hacker-league tracy/public/TracyClient.cpp

hacker-league: main.cpp common.h shaders/world/vert.spv.h shaders/world/frag.spv.h shaders/hud/vert.spv.h shaders/hud/frag.spv.h font.h font.png 
	g++ $(CXXFLAGS) -o $@ $(GAME_SRC) -lvulkan -lglfw -lpthread -lcurl

font.h font.png: font
	./$<

font: font.cpp
	g++ $(CXXFLAGS) -o $@ $<

shaders/world/%.spv.h: shaders/world/shader.%
	shaders/glslc $< -o ./shaders/world/$*.spv
	xxd -i ./shaders/world/$*.spv > $@

shaders/hud/%.spv.h: shaders/hud/shader.%
	shaders/glslc $< -o ./shaders/hud/$*.spv
	xxd -i ./shaders/hud/$*.spv > $@

server: server.cpp common.h
	g++ $(CXXFLAGS) -o $@ $< -lpthread

clean:
	rm -f hacker-league font font.h font.png server shaders/*/*spv*
