#ifndef ARDUINO
#include <math.h>
#include "common.h"
#endif

quaternion_t q_make(vector_t const &v)
{
  return (quaternion_t){0, v.x, v.y, v.z};
}

quaternion_t q_make(float angle, vector_t const &v)
{
  vector_t scaledV = v_scale(sin(angle), v);
  return (quaternion_t){cos(angle), scaledV.x, scaledV.y, scaledV.z};
}

quaternion_t q_conjugate(quaternion_t const &q)
{
  return (quaternion_t){q.w, -q.x, -q.y, -q.z};
}

quaternion_t q_multiply(quaternion_t const &q1, quaternion_t const &q2)
{
  return (quaternion_t) {
    q2.w*q1.w - q2.x*q1.x - q2.y*q1.y - q2.z*q1.z,
    q2.w*q1.x + q2.x*q1.w - q2.y*q1.z + q2.z*q1.y,
    q2.w*q1.y + q2.x*q1.z + q2.y*q1.w - q2.z*q1.x,
    q2.w*q1.z - q2.x*q1.y + q2.y*q1.x + q2.z*q1.w
  };
}

float q_magnitude(quaternion_t const &q)
{
  return sqrt((q.w * q.w) + (q.x * q.x) + (q.y * q.y) + (q.z * q.z));
}

quaternion_t q_normalize(quaternion_t const &q)
{
  float magnitude = q_magnitude(q);
  quaternion_t q_out = q;

  if (magnitude != 0.0) {
    q_out.w /= magnitude;
    q_out.x /= magnitude;
    q_out.y /= magnitude;
    q_out.z /= magnitude;
  }

  return q_out;
}

vector_t q_vector(quaternion_t const &q)
{
  return vector_t{q.x, q.y, q.z};
}

void q_print(quaternion_t const &q, int digits)
{
#ifdef ARDUINO
  Serial.print(q.w, digits);
  Serial.print(",");
  Serial.print(q.x, digits);
  Serial.print(",");
  Serial.print(q.y, digits);
  Serial.print(",");
  Serial.print(q.z, digits);
#endif
}

void q_print(quaternion_t const &q)
{
  q_print(q, 2);
}
