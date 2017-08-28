#ifndef UNIVERSE_H
#define UNIVERSE_H

#include <vector>
#include "orb.hpp"

class CelestialBody;
class Vessel;

class Universe {
// This class stores the simulation objects, the current time,
// and is responsible for propagating the objects through time.
public:
// Universe Parameters
  double mjd; // Current Modified Julian Date of the simulation
  double simt; // Current simulation time
  std::vector<CelestialBody *> celbody_list; // List of celestial bodies in simulation
  std::vector<Vessel *> vessel_list; //list of spacecraft

// Constructor / Destructor
  Universe(double init_mjd);
  ~Universe();

// Propagation Routines
  void propagate(double step); // Propagate the simulation by step seconds
  void propagate_linear(Vessel *ves, double step);
  void integrate_rk4_linear(double mjd, Vessel *ves, double step, Vector3 F, Vector3 G);
  ObjHandle compute_gravity(double mjd, Vessel *ves, Vector3& G);
  LinearStateVector eom_linear(double mjd, LinearStateVector state, Vector3 F, Vector3 G, double mass);

// Angular Propagation routines
  void integrate_rk4_angular(Vessel *ves, double step);
  AngularStateVector eom_angular(double mjd, AngularStateVector state, Vector3 T, PMI moi);

// Object Handling routines
  bool add_vessel(Vessel *ves);
  bool add_celbody(CelestialBody *cb);

};

#endif
