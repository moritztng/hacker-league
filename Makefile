CXXFLAGS = -std=c++17 -O3

.PHONY: debug release clean

all: debug

debug: hacker-league server

release: CXXFLAGS += -DNDEBUG
release: hacker-league server

hacker-league: main.cpp shaders/vert.spv.h shaders/frag.spv.h 
	g++ $(CXXFLAGS) -o hacker-league main.cpp -lvulkan -lglfw -lpthread

shaders/%.spv.h: shaders/shader.%
	shaders/glslc $< -o ./shaders/$*.spv
	xxd -i ./shaders/$*.spv > $@

server: server.cpp 
	g++ $(CXXFLAGS) -o server server.cpp -lpthread

clean:
	rm -f hacker-league server shaders/*spv*
