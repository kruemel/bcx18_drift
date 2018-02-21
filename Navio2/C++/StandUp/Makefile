CXX ?= g++
NAVIO = ../Navio
INCLUDES = -I ../Navio -I ../Examples

all:
	$(MAKE) -C ../Navio all
	$(MAKE) -C ../Examples all
	$(CXX) -std=gnu++11 $(INCLUDES) StandUp.cpp -L$(NAVIO) -lnavio -o StandUp

clean:
	rm -f StandUp
