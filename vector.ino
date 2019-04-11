#ifndef ARDUINO
#include <math.h>
#include "common.h"
#endif

vector_t::vector_t() {
  vector_t();
}

vector_t::vector_t(float x, float y, float z) : x(x), y(y), z(z) {}

/*
 * Calculate the cross product of this vector and another vector (this x other).
 */
vector_t vector_t::crossproduct(vector_t const &other) const {
  vector_t prod = (vector_t){
    (y * other.z) - (z * other.y),
    (z * other.x) - (x * other.z),
    (x * other.y) - (y * other.x)
  };

  return prod;
}

/*
 * Calculate the dot product of this vector and another vector (this x other).
 */
float vector_t::dotproduct(vector_t const &other) const
{
  return (x * other.x) + (y * other.y) + (z * other.z);
}

/*
 * Calculate the magnitude of this vector.
 */
float vector_t::magnitude() const
{
  return sqrt(dotproduct(*this));
}

/*
 * Return the unit vector pointing in the same direction as this vector.
 */
vector_t vector_t::normalize() const
{
  float m = magnitude();

  if (m == 0.0) {
    return *this;
  } else {
    return scale(1.0 / m);
  }
}

vector_t vector_t::opposite() const
{
  return (vector_t){-x, -y, -z};
}

vector_t vector_t::scale(float scalar) const
{
  return (vector_t){scalar * x, scalar * y, scalar * z};
}

void vector_t::print(int digits) const
{
#ifdef ARDUINO
  Serial.print(x, digits);
  Serial.print(",");
  Serial.print(y, digits);
  Serial.print(",");
  Serial.print(z, digits);
#endif
}

void vector_t::print() const
{
  print(2);
}
