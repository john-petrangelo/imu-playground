#ifndef ARDUINO
#include <math.h>

#include "common.h"
#endif

Vector::Vector() {
  ;
}

Vector::Vector(float x, float y, float z) : x(x), y(y), z(z) {}

bool Vector::operator==(Vector const &other) const
{
  float const TOLERANCE = 0.01;
  
  return
    fabs(x - other.x) < TOLERANCE &&
    fabs(y - other.y) < TOLERANCE &&
    fabs(z - other.z) < TOLERANCE;
}

/*
 * Calculate the cross product of this vector and another vector (this x other).
 */
Vector Vector::crossproduct(Vector const &other) const
{
  Vector prod = (Vector){
    (y * other.z) - (z * other.y),
    (z * other.x) - (x * other.z),
    (x * other.y) - (y * other.x)
  };

  return prod;
}

/*
 * Calculate the dot product of this vector and another vector (this x other).
 */
float Vector::dotproduct(Vector const &other) const
{
  return (x * other.x) + (y * other.y) + (z * other.z);
}

/*
 * Calculate the magnitude of this vector.
 */
float Vector::magnitude() const
{
  return sqrt(dotproduct(*this));
}

/*
 * Return the unit vector pointing in the same direction as this vector.
 */
Vector Vector::normalize() const
{
  float m = magnitude();

  if (m == 0.0) {
    return *this;
  } else {
    return scale(1.0 / m);
  }
}

Vector Vector::opposite() const
{
  return (Vector){-x, -y, -z};
}

Vector Vector::scale(float scalar) const
{
  return (Vector){scalar * x, scalar * y, scalar * z};
}

void Vector::print(int digits) const
{
#ifdef ARDUINO
  Serial.print(x, digits);
  Serial.print(",");
  Serial.print(y, digits);
  Serial.print(",");
  Serial.print(z, digits);
#endif
}

void Vector::print() const
{
  print(2);
}

#ifndef ARDUINO
std::ostream& operator<<(std::ostream& os, Vector const &v)
{
  return os << "(" << v.x << ", "  << v.y << ", " << v.z << ")";  
}
#endif
