#include "Vessel.hpp"
#include "Thruster.hpp"
#include <cmath>

Vessel::Vessel()
{
  ref = 0;
}

Vessel::~Vessel()
{

}

int Vessel::preStep(double mjd, double simt, double dt)
{
  return 0;
}

void Vessel::update(double mjd, double simt, double dt)
{
  // Decrease mass according to current thruster usage
  double deltaM = 0;
  for(Thruster *th : thrusters)
  {
    deltaM += dt*(th->getCurrentThrustMag()/(g0*th->getIsp()));
  }
  if(mass-empty_mass < deltaM) {
    killThrust();
  }
  mass = fmax(mass-(empty_mass+deltaM), empty_mass);
}

void Vessel::addForce(const Vector3& F, const Vector3& pos)
{
  ForceData *fdat = new ForceData;
  fdat->F = F;
  fdat->pos = pos;
  force_stack.push_back(fdat);
}

void Vessel::getTotalForce(Vector3& F)
{
  // Get Total Force in Body coords
  F = Vector3(0.0, 0.0, 0.0);
  for(ForceData *fd : force_stack)
  {
    F = F + fd->F;
  }

  for(Thruster *th : thrusters)
  {
    F = F + th->getCurrentThrust();
  }
}

void Vessel::getTotalTorque(Vector3& T)
{
  T = Vector3(0.0, 0.0, 0.0);
  for(ForceData *fd : force_stack)
  {
    T = T + cross(fd->pos, fd->F);
  }

  for(Thruster *th : thrusters)
  {
    T = T + cross(th->getPosition(), th->getCurrentThrust());
  }
}

void Vessel::Local2Global(const Vector3& loc, Vector3& glob)
{
  Quaternion p = Quaternion(0.0, loc.x, loc.y, loc.z);
  Quaternion res = rot_state.q.inv()*p*rot_state.q;
  glob.x = res.x;
  glob.y = res.y;
  glob.z = res.z;
}

double Vessel::interpTotalMass(double step)
{
  return mass;
}

void Vessel::setReference(ObjHandle hRef)
{
  ref = (CelestialBody *)hRef;
}

/* Thruster Management */
ThrusterHandle Vessel::addThruster(Vector3& pos, Vector3& dir,
                                   double max_thrust, double Isp)
{
  Thruster *th = new Thruster(pos, dir, max_thrust, Isp);
  thrusters.push_back(th);
  return (ThrusterHandle)th;
}

bool Vessel::delThruster(ThrusterHandle th)
{
  delete (Thruster *)th;
  return 1;
}

void Vessel::killThrust()
{
  for(Thruster *th : thrusters)
  {
    th->setLevel(0.0);
  }
}

// Thruster Group Management
ThrusterGroup * Vessel::createThrusterGroup(ThrusterGroupType type)
{
  ThrusterGroup *th_grp = new ThrusterGroup;
  th_grp->type = type;
  return th_grp;
}

void Vessel::destroyThrusterGroup(ThrusterGroup *th_grp)
{
  delete th_grp;
}

void Vessel::addThrusterToGroup(ThrusterGroup *th_grp, ThrusterHandle th)
{
  th_grp->thrusters.push_back(th);
}
