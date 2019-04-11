#ifndef __COMMON_H__
#define __COMMON_H__

// Custom types
struct Vector {
  float x;
  float y;
  float z;

  Vector();
  Vector(float x, float y, float z);
  Vector crossproduct(Vector const &other) const;
  float dotproduct(Vector const &other) const;
  float magnitude() const;
  Vector normalize() const;
  Vector opposite() const;
  Vector scale(float scalar) const;

  void print(int digits) const;
  void print() const;
};

struct quaternion_t {
  float w;
  float x;
  float y;
  float z;
};

struct attitude_t {
  Vector euler;
  Vector ihat;
  Vector jhat;
  Vector khat;
  float heading;
  float pitch;
  float roll;
};

// Quaternion functions
quaternion_t q_make(Vector const &v);
quaternion_t q_make(float angle, Vector const &v);

quaternion_t q_conjugate(quaternion_t const &q);
quaternion_t q_multiply(quaternion_t const &q1, quaternion_t const &q2);

float q_magnitude(quaternion_t const &q);
quaternion_t q_normalize(quaternion_t const &q);

Vector q_vector(quaternion_t const &q);

void q_print(quaternion_t const &q, int digits);
void q_print(quaternion_t const &q);

#endif /* __COMMON_H__ */
