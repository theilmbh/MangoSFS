#include "Universe.hpp"
#include "orb.hpp"
#include "Vessel.hpp"
#include "CelestialBody.hpp"

#include <vector>

Universe::Universe(double init_mjd)
{
  this->mjd  = init_mjd;
  this->simt = 0.0;
}

Universe::~Universe()
{

}

void Universe::propagate(double step)
{
  for(Vessel *v : vessel_list)
  {
    v->update(mjd, simt, step);
    propagate_linear(v, step);
    integrate_rk4_angular(v, step);
  }
  simt += step;
  mjd += step;
}

void Universe::propagate_linear(Vessel *ves, double step)
{
// Compute Gravity accelerations
  Vector3 F, G;
  ves->getTotalForce(F);
  ObjHandle hRef = compute_gravity(mjd, ves, G);

// Update vessel gravity reference
  ves->setReference(hRef);

// Rotate to global frame
  F = rotate_vector(ves->rot_state.q.inv(), F);

// integrate
  integrate_rk4_linear(mjd, ves, step, F, G);
}

void Universe::integrate_rk4_linear(double mjd, Vessel *ves, double step, Vector3 F, Vector3 G)
{
  LinearStateVector state = ves->lin_state;
  LinearStateVector k1, k2, k3, k4;
  double step2 = step/2;
  k1 = eom_linear(mjd, state, F, G, ves->interpTotalMass(0.0));
  k2 = eom_linear(mjd + step2, state + step2*k1, F, G, ves->interpTotalMass(step2));
  k3 = eom_linear(mjd + step2, state + step2*k2, F, G,  ves->interpTotalMass(step2));
  k4 = eom_linear(mjd+step, state + step*k3, F, G, ves->interpTotalMass(step));
  ves->lin_state = state + (step/6)*(k1 + 2*k2 + 2*k3 + k4);

}

LinearStateVector Universe::eom_linear(double mjd, LinearStateVector state, Vector3 F, Vector3 G, double mass)
{
  /* initialize dstate */
  LinearStateVector dstate;

  /* Position Derivatives (velocity) */
  dstate.r.x = state.v.x;
  dstate.r.y = state.v.y;
  dstate.r.z = state.v.z;

  /* Calculate gravitational accelerations */

  /*print_vector(grav);*/
  dstate.v.x = G.x + F.x / mass;
  dstate.v.y = G.y + F.y / mass;
  dstate.v.z = G.z + F.z / mass;

  return dstate;
}

ObjHandle Universe::compute_gravity(double mjd, Vessel *ves, Vector3& G)
{
  Vector3 r, r_hat, delta_r, accel = {0.0, 0.0, 0.0}; /* object position */
  double r_mag = 0;
  double g_mag = 0;
  double g_max = 0;
  CelestialBody *g_max_body = 0;

  r = ves->lin_state.r;

  for(CelestialBody *cb : celbody_list)
  {
    delta_r = r + (-1.0)*cb->get_ephemeris(mjd);
    r_mag   = norm3(delta_r);
    r_hat   = (1.0/r_mag)*delta_r;
    g_mag   = ((cb->mu) / (r_mag*r_mag));

    if(g_mag > g_max)
    {
      g_max_body = cb;
      g_max = g_mag;
    }
    accel = accel + -g_mag*r_hat;
  }
  G = accel;
  return g_max_body;
}

void Universe::integrate_rk4_angular(Vessel *ves, double step)
{
  AngularStateVector k1, k2, k3, k4;
  AngularStateVector state = ves->rot_state;
  PMI moi = ves->moi;
  double step2 = step/2.0;

  // Compute Torque
  Vector3 T;
  ves->getTotalTorque(T);
  T = rotate_vector(ves->rot_state.q.inv(), T);

  k1 = eom_angular(mjd, state, T, moi);
  k2 = eom_angular(mjd+step2, state + step2*k1, T, moi);
  k3 = eom_angular(mjd+step2, state + step2*k2, T, moi);
  k4 = eom_angular(mjd+step, state + step*k3, T, moi);
  ves->rot_state = (state + (step/6)*(k1 + 2*k2 + 2*k3 + k4));
  ves->rot_state.q = ves->rot_state.q.normalize();
}

AngularStateVector Universe::eom_angular(double mjd, AngularStateVector state, Vector3 T, PMI moi)
{
  AngularStateVector dstate;
  Quaternion q = state.q;
  Vector3 om = state.omega;

  // quaternion kinematics
  dstate.q.x = ( om.z * q.y - om.y * q.z + om.x * q.w)/2;
  dstate.q.y = (-om.z * q.x + om.x * q.z + om.y * q.w)/2;
  dstate.q.z = ( om.y * q.x - om.x * q.y + om.z * q.w)/2;
  dstate.q.w = (-om.x * q.x - om.y * q.y - om.z * q.z)/2;

  // Euler equations
  dstate.omega.x = ((moi.Iz - moi.Iy) * om.y * om.z + T.x) / moi.Ix;
  dstate.omega.y = ((moi.Ix - moi.Iz) * om.z * om.x + T.y) / moi.Iy;
  dstate.omega.z = ((moi.Iy - moi.Ix) * om.x * om.y + T.z) / moi.Iz;

  return dstate;
}

// Object handlers

bool Universe::add_vessel(Vessel *ves)
{
  vessel_list.push_back(ves);
  return 1;

}

bool Universe::add_celbody(CelestialBody *cb)
{
  celbody_list.push_back(cb);
  return 1;
}
