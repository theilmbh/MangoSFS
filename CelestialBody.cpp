#include "CelestialBody.hpp"
#include "orb.hpp"

CelestialBody::CelestialBody()
{

}

CelestialBody::~CelestialBody()
{

}

Vector3 CelestialBody::get_ephemeris(double mjd)
{
  Vector3 origin = {0.0, 0.0, 0.0};
  return origin;
}
