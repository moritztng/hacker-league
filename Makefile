CXXFLAGS = -std=c++17 -O3
LDFLAGS = -lvulkan -lglfw -lpthread

universe: main.cpp
	g++ $(CXXFLAGS) -o universe main.cpp $(LDFLAGS)

.PHONY: clean
clean:
	rm -f universe
