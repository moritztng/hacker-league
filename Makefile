CXXFLAGS = -std=c++17 -O3
LDFLAGS = -lvulkan -lglfw -lpthread

.PHONY: debug release clean

all: debug

debug: universe

release: CXXFLAGS += -DNDEBUG
release: universe

universe: main.cpp
	g++ $(CXXFLAGS) -o universe main.cpp $(LDFLAGS)

clean:
	rm -f universe
