#include "orb.hpp"
#include <iostream>

void print_state(LinearStateVector state)
{
  std::cout << "X: " << state.r.x << " Y: " << state.r.y << " Z: " << state.r.z << std::endl;
  std::cout << "VX: " << state.v.x << " VY: " << state.v.y << " VZ: " << state.v.z << std::endl;
}

void print_state(AngularStateVector state)
{
  double roll, pitch, yaw;
  QuaternionToEuler(state.q, roll, pitch, yaw);
  std::cout << "Roll: " << roll << " Pitch: " << pitch << " Yaw: " << yaw << std::endl;
  std::cout << "OmegaX: " << state.omega.x << " OmegaY: " << state.omega.y << " OmegaZ: " << state.omega.z << std::endl;
}

void print_vector(Vector3 vec)
{
  std::cout << "X: " << vec.x << " Y: " << vec.y << " Z: " << vec.z << std::endl;
}

Vector3::Vector3()
{
  x = 0.0;
  y = 0.0;
  z = 0.0;
}

Vector3::Vector3(double a, double b, double c)
{
  x = a;
  y = b;
  z = c;
}

Vector3 operator+(Vector3 const& lhs, Vector3 const& rhs)
{
  Vector3 ret;
  ret.x = lhs.x + rhs.x;
  ret.y = lhs.y + rhs.y;
  ret.z = lhs.z + rhs.z;
  return ret;
}

Vector3 operator*(const double& c, Vector3 const& rhs)
{
  Vector3 ret;
  ret.x = c*rhs.x;
  ret.y = c*rhs.y;
  ret.z = c*rhs.z;
  return ret;
}

double norm3(Vector3 v)
{
  return sqrt(v.x*v.x + v.y*v.y + v.z*v.z);
}

Vector3 cross(const Vector3& a, const Vector3& b)
{
  Vector3 ret;
  ret.x = a.y*b.z - a.z*b.y;
  ret.y = a.z*b.x - a.x*b.z;
  ret.z = a.x*b.y - a.y*b.x;
  return ret;
}

Quaternion operator+(const Quaternion& lhs, const Quaternion& rhs)
{
  Quaternion ret;
  ret.w = lhs.w + rhs.w;
  ret.x = lhs.x + rhs.x;
  ret.y = lhs.y + rhs.y;
  ret.z = lhs.z + rhs.z;
  return ret;
}

Quaternion operator*(const double& c, const Quaternion& rhs)
{
  Quaternion ret;
  ret.w = c*rhs.w;
  ret.x = c*rhs.x;
  ret.y = c*rhs.y;
  ret.z = c*rhs.z;
  return ret;
}

Quaternion operator*(const Quaternion& lhs, const Quaternion& rhs)
{
  Quaternion ret;
  ret.w = lhs.w*rhs.w - lhs.x*rhs.x - lhs.y*rhs.y - lhs.z*rhs.z;
  ret.x = lhs.w*rhs.x + lhs.x*rhs.w + lhs.y*rhs.z - lhs.z*rhs.y;
  ret.y = lhs.w*rhs.y - lhs.x*rhs.z + lhs.y*rhs.w + lhs.z*rhs.x;
  ret.z = lhs.w*rhs.z + lhs.x*rhs.y - lhs.y*rhs.x + lhs.z*rhs.w;
  return ret;
}

Quaternion::Quaternion()
{
  w = 0;
  x = 0;
  y = 0;
  z = 0;
}

Quaternion::Quaternion(double a, double b, double c, double d)
{
  w = a;
  x = b;
  y = c;
  z = d;
}

Quaternion Quaternion::normalize() const
{
  // Returns a Normalized quaternion
  double norm = sqrt(pow(this->w, 2) + pow(this->x, 2) + pow(this->y, 2) + pow(this->z, 2));
  Quaternion ret = Quaternion(this->w / norm, this->x / norm, this->y / norm, this->z / norm);
  return ret;
}

Quaternion Quaternion::conj() const
{
  // Returns a conjugate quaternion
  Quaternion ret = Quaternion(w, -x, -y, -z);
  return ret;
}

Quaternion Quaternion::inv() const
{
  Quaternion ret = this->normalize();
  ret = ret.conj();
  return ret;
}


LinearStateVector operator+(LinearStateVector const& lhs, LinearStateVector const& rhs)
{
  LinearStateVector ret;

  ret.r = lhs.r + rhs.r;
  ret.v = lhs.v + rhs.v;
  return ret;
}

LinearStateVector operator*(const double& c, const LinearStateVector& rhs)
{
  LinearStateVector ret;
  ret.r = c*rhs.r;
  ret.v = c*rhs.v;
  return ret;

}

AngularStateVector operator*(const double& c, const AngularStateVector& rhs)
{
  AngularStateVector ret;
  ret.q = c*rhs.q;
  ret.omega = c*rhs.omega;
  return ret;
}

AngularStateVector operator+(const AngularStateVector& lhs, const AngularStateVector& rhs)
{
  AngularStateVector ret;
  ret.q = lhs.q + rhs.q;
  ret.omega = lhs.omega + rhs.omega;
  return ret;
}

void QuaternionToEuler(const Quaternion q, double& roll, double& pitch, double& yaw)
{
  // roll
  double sinr = 2 * (q.w*q.x + q.y*q.z);
  double cosr = 1 - 2 * (q.x*q.x + q.y*q.y);
  roll = atan2(sinr, cosr);

  double sinp = 2 * (q.w*q.y - q.z*q.x);
  if(fabs(sinp) >= 1)
  {
    pitch = copysign(pi /2, sinp);
  }
  else
  {
    pitch = asin(sinp);
  }

  double siny = 2 * (q.w*q.z + q.y*q.x);
  double cosy = 1 - 2 * (q.y*q.y + q.z*q.z);
  yaw = atan2(siny, cosy);
}

Vector3 rotate_vector(const Quaternion q, Vector3 v)
{
  Quaternion p = Quaternion(0.0, v.x, v.y, v.z);
  Quaternion res = q*p*q.inv();
  Vector3 ret = Vector3(res.x, res.y, res.z);
  return ret;
}
