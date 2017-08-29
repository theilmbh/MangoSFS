#include <iostream>
#include <unistd.h>

#include "orb.hpp"
#include "Universe.hpp"
#include "CelestialBody.hpp"
#include "Vessel.hpp"

const double mu_earth = 3.986e14;



int main()
{
  const double mjd_start = 0.0;
  Universe uni(mjd_start);
  CelestialBody *Earth = new CelestialBody();
  Earth->mu = mu_earth;
  Vessel *Probe = new Vessel();

  Probe->mass = 1000.0;
  Probe->empty_mass = 10.0;
  LinearStateVector ProbeInitState;
  ProbeInitState.r.x = 6871.0e3 + 400e3;
  ProbeInitState.r.y = 0.0;
  ProbeInitState.r.z = 0.0;
  ProbeInitState.v.x = 0.0;
  ProbeInitState.v.y = sqrt(mu_earth/(6871.0e3 + 400e3));
  ProbeInitState.v.z = 0.0;
  Probe->lin_state = ProbeInitState;

  AngularStateVector ProbeRotState;
  ProbeRotState.q.w = 1.0;
  ProbeRotState.q.x = 0.0;
  ProbeRotState.q.y = 0.0;
  ProbeRotState.q.z = 0.0;

  ProbeRotState.omega.x = 0.0;
  ProbeRotState.omega.y = 0.0;
  ProbeRotState.omega.z = 0.0;
  Probe->addForce(Vector3(0.0, 0, 100.0), Vector3(0, 0.1, 0));
  Probe->addForce(Vector3(0.0, 0.0, -100.0), Vector3(0, -0.1, 0));

  Probe->rot_state = ProbeRotState;
  PMI moi = {2,2,2};
  Probe->moi = moi;

  uni.add_vessel(Probe);
  uni.add_celbody(Earth);

  print_state(Probe->lin_state);
  int frames = 0;
  while(1)
  {
    uni.propagate(0.1);
    if(frames % 10 == 0)
    {
      print_state(Probe->lin_state);
      double h = norm3(Probe->lin_state.r) - 6871.0e3;
      std::cout << "H: " << h << "\n";
      std:: cout << "\n";
      print_state(Probe->rot_state);
      std::cout << "\n";
    }
    frames +=1;
    usleep(100000);
  }


  print_state(Probe->lin_state);
  return 0;
}
