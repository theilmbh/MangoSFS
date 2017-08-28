#ifndef CELBODY_H
#define CELBODY_H

#include "orb.hpp"

class CelestialBody {

public:
  double mu;
  CelestialBody *parent;
  ObjHandle h;

  CelestialBody();
  ~CelestialBody();

  Vector3 get_ephemeris(double mjd);
};

#endif
