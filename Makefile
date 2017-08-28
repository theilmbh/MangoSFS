CC=g++
CFLAGS=--std=c++11 -lm

simtest: simtest.cpp Vessel.cpp CelestialBody.cpp orb.hpp Universe.cpp
	$(CC) $(CFLAGS) -o simtest simtest.cpp Vessel.cpp CelestialBody.cpp Universe.cpp orb.cpp
