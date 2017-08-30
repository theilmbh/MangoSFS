#ifndef VESSEL_H
#define VESSEL_H

#include "orb.hpp"
#include "Thruster.hpp"
#include <vector>

#define g0 9.81

class CelestialBody;


class Vessel {

public:
// Physical State Vectors
  LinearStateVector lin_state;
  AngularStateVector rot_state;

// Forces + Thrusters
  std::vector<ForceData*> force_stack;
  std::vector<Thruster*> thrusters;
  std::vector<ThrusterGroup*> thruster_groups;

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

  /* Body Coordinates to Global Frame */
  void Local2Global(const Vector3& loc, Vector3& glob);

  /* Get LVLH Frame */
  void getLVLH(Vector3& X, Vector3& Y, Vector3& Z);

// Thruster Management
  ThrusterHandle addThruster(Vector3& pos, Vector3& dir,
                             double max_thrust, double Isp);
  bool delThruster(ThrusterHandle th);
  void killThrust(); /* Kills all currently active thrusters */

// Thruster Groups
  ThrusterGroup* createThrusterGroup(ThrusterGroupType type);
  void destroyThrusterGroup(ThrusterGroup *th_grp);
  void addThrusterToGroup(ThrusterGroup *th_grp, ThrusterHandle th);
};

#endif
