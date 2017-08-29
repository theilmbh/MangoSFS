#include "Thruster.hpp"

Thruster::Thruster(Vector3& pos, Vector3& dir, double max_thrust, double isp)
{
  this->pos = pos;
  this->dir = dir;
  this->max_thrust = max_thrust;
  this->Isp = isp;
}

Thruster::~Thruster()
{

}

Vector3 Thruster::getCurrentThrust()
{
  return (level*max_thrust/norm3(dir))*dir;
}

double Thruster::getCurrentThrustMag()
{
  return (level*max_thrust);
}

void Thruster::setLevel(double lvl)
{
  level = lvl;
}

double Thruster::getLevel()
{
  return level;
}

double Thruster::getIsp()
{
  return Isp;
}

Vector3 Thruster::getPosition()
{
  return this->pos;
}

Vector3 Thruster::getDirection()
{
  return this->dir;
}
