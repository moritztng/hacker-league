CXXFLAGS = -std=c++17 -O3
LDFLAGS = -lvulkan -lglfw -lpthread

.PHONY: debug release clean

all: debug

debug: hacker-league

release: CXXFLAGS += -DNDEBUG
release: hacker-league

hacker-league: main.cpp shaders/vert.spv.h shaders/frag.spv.h 
	g++ $(CXXFLAGS) -o hacker-league main.cpp $(LDFLAGS)

shaders/%.spv.h: shaders/shader.%
	shaders/glslc $< -o ./shaders/$*.spv
	xxd -i ./shaders/$*.spv > $@

clean:
	rm -f hacker-league shaders/*spv*
