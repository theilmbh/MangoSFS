#ifndef THRUSTER_H
#define THRUSTER_H

#include "orb.hpp"

class Thruster {

  Vector3 pos;
  Vector3 dir;

  double max_thrust;
  double Isp;
  double level; //Thrust level between 0-1

public:
  Thruster(Vector3& pos, Vector3& dir, double max_thrust, double isp);
  ~Thruster();

  Vector3 getCurrentThrust();
  double getLevel();
  void setLevel(double lvl);

  Vector3 getPosition();
  Vector3 getDirection();

};

#endif
