CFLAGS = -std=c++17 -O2
LDFLAGS = -lglfw -lvulkan -lpthread

universe: main.cpp
	g++ $(CFLAGS) -o universe main.cpp $(LDFLAGS)

.PHONY: test clean

test: universe
	./universe

clean:
	rm -f universe
