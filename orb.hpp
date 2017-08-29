#ifndef ORB_H
#define ORB_H

#include <cmath>
typedef void * ObjHandle;
typedef void * ThrusterHandle;

const double pi = 3.14159;
typedef struct Vector3 {
  double x;
  double y;
  double z;
  Vector3();
  Vector3(double a, double b, double c);
} Vector3;

double norm3(Vector3 v);
Vector3 cross(const Vector3& a, const Vector3& b);
Vector3 operator*(const double& c, Vector3 const& rhs);
Vector3 operator+(Vector3 const& lhs, Vector3 const& rhs);

struct Quaternion {
  double w;
  double x;
  double y;
  double z;
  Quaternion();
  Quaternion(double a, double b, double c, double d);
  Quaternion normalize() const;
  Quaternion conj() const;
  Quaternion inv() const ;
};

Quaternion operator+(const Quaternion& lhs, const Quaternion& rhs);
Quaternion operator*(const double& c, const Quaternion& rhs);
Quaternion operator*(const Quaternion& lhs, const Quaternion& rhs);

struct LinearStateVector {
// Linear position
  Vector3 r;

// Linear Velocities
  Vector3 v;


};

// Operator Overloads
LinearStateVector operator*(const double& c, const LinearStateVector& rhs);
LinearStateVector operator+(LinearStateVector const& lhs, LinearStateVector const& rhs);

typedef struct AngularStateVector {
// Angular Position Quaternion
  Quaternion q;

// Angular Velocities
  Vector3 omega;
} AngularStateVector;

AngularStateVector operator*(const double& c, const AngularStateVector& rhs);
AngularStateVector operator+(const AngularStateVector& lhs, const AngularStateVector& rhs);

typedef struct PMI {
// Principle moments of Inertia
  double Ix;
  double Iy;
  double Iz;
} PMI;

struct ForceData {
  Vector3 pos; // Position in body frame
  Vector3 F; // direction in body frmae
};

void print_state(LinearStateVector state);
void print_state(AngularStateVector state);
void print_vector(Vector3 vec);

void QuaternionToEuler(const Quaternion q, double& roll, double& pitch, double& yaw);
Vector3 rotate_vector(const Quaternion q, Vector3 v);

#endif
