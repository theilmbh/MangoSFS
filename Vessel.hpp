#ifndef VESSEL_H
#define VESSEL_H

#include "orb.hpp"
#include <vector>

class CelestialBody;

class Vessel {

public:
// Physical State Vectors
  LinearStateVector lin_state;
  AngularStateVector rot_state;

// Forces
  std::vector<ForceData*> force_stack;

// Internal handle and current reference body
  ObjHandle h;
  CelestialBody *ref;

// Physical Parameters
  double mass, empty_mass;
  PMI moi;

// Construct and Destruct
  Vessel();
  ~Vessel();

// Called at the beginning of each propagation step
  int preStep(double mjd, double simt, double dt);
  void update(double mjd, double simt, double dt);

// Gravity Reference
  void setReference(ObjHandle hRef);

// Interpolate mass due to propellant usage
  double interpTotalMass(double step);

// Forces
  void addForce(const Vector3& F, const Vector3& pos);
  void getTotalForce(Vector3& F); // Gets total force on vessel in body coords
  void getTotalTorque(Vector3& T);

// Coordinate Transforms
  void Local2Global(const Vector3& loc, Vector3& glob);
};

#endif
