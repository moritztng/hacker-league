CXXFLAGS = -std=c++17 -O3
LDFLAGS = -lvulkan -lglfw -lpthread

.PHONY: debug release clean

all: debug

debug: universe

release: CXXFLAGS += -DNDEBUG
release: universe

universe: main.cpp shaders/vert.spv.h shaders/frag.spv.h 
	g++ $(CXXFLAGS) -o universe main.cpp $(LDFLAGS)

shaders/%.spv.h: shaders/shader.%
	shaders/glslc $< -o ./shaders/$*.spv
	xxd -i ./shaders/$*.spv > $@

clean:
	rm -f universe shaders/*spv*
