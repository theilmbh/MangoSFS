#ifndef THRUSTER_H
#define THRUSTER_H

#include "orb.hpp"
#include <vector>

enum ThrusterGroupType {
  MAIN,
  RETRO
};

struct ThrusterGroup {

  ThrusterGroupType type;
  std::vector<ThrusterHandle> thrusters;
};

class Thruster {

  Vector3 pos;
  Vector3 dir;

  double max_thrust;
  double Isp;
  double level; //Thrust level between 0-1

public:
  Thruster(Vector3& pos, Vector3& dir, double max_thrust, double isp);
  ~Thruster();

// Thrust Handling
  Vector3 getCurrentThrust();
  double  getCurrentThrustMag();
  double  getLevel();
  void    setLevel(double lvl);
  double  getIsp();

// Force Vectors
  Vector3 getPosition();
  Vector3 getDirection();

};

#endif
