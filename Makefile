CXXFLAGS = -std=c++17 -O3
GAME_SRC = main.cpp
PYTHON_MODULE = hacker_league$(shell python3-config --extension-suffix)

.PHONY: debug release hacker-league_profile python_module clean

all: debug

debug: hacker-league server

release: CXXFLAGS += -DNDEBUG
release: hacker-league server

hacker-league_profile: CXXFLAGS += -DTRACY_ENABLE -Itracy/public
hacker-league_profile: GAME_SRC += tracy/public/TracyClient.cpp
hacker-league_profile: hacker-league tracy/public/TracyClient.cpp

hacker-league: main.cpp common.h shaders/world/vert.spv.h shaders/world/frag.spv.h shaders/hud/vert.spv.h shaders/hud/frag.spv.h font.h font.png 
	g++ $(CXXFLAGS) -o hacker-league $(GAME_SRC) -lvulkan -lglfw -lpthread -lcurl

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

python_module: $(PYTHON_MODULE)
$(PYTHON_MODULE): python_bindings.cpp common.h
	g++ $(CXXFLAGS) -shared -fPIC -I$(shell python3 -c "import sysconfig; print(sysconfig.get_path('include'))") -I/usr/include/pybind11 -I/usr/include/eigen3 $< -o $@

clean:
	rm -f hacker-league font font.h font.png server shaders/*/*spv* $(PYTHON_MODULE)
