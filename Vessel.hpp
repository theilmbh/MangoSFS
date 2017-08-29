#ifndef VESSEL_H
#define VESSEL_H

#include "orb.hpp"
#include <vector>

#define g0 9.81

class CelestialBody;
class Thruster;

class Vessel {

public:
// Physical State Vectors
  LinearStateVector lin_state;
  AngularStateVector rot_state;

// Forces + Thrusters
  std::vector<ForceData*> force_stack;
  std::vector<Thruster*> thrusters;

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

// Thruster Management
  ThrusterHandle addThruster(Vector3& pos, Vector3& dir,
                             double max_thrust, double Isp);
  bool delThruster(ThrusterHandle th);
};

#endif
