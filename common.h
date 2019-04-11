#ifndef __COMMON_H__
#define __COMMON_H__

// Custom types
struct vector_t {
  float x;
  float y;
  float z;

  vector_t();
  vector_t(float x, float y, float z);
  vector_t crossproduct(vector_t const &other) const;
  float dotproduct(vector_t const &other) const;
  float magnitude() const;
  vector_t normalize() const;
  vector_t opposite() const;
  vector_t scale(float scalar) const;

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
  vector_t euler;
  vector_t ihat;
  vector_t jhat;
  vector_t khat;
  float heading;
  float pitch;
  float roll;
};

// Quaternion functions
quaternion_t q_make(vector_t const &v);
quaternion_t q_make(float angle, vector_t const &v);

quaternion_t q_conjugate(quaternion_t const &q);
quaternion_t q_multiply(quaternion_t const &q1, quaternion_t const &q2);

float q_magnitude(quaternion_t const &q);
quaternion_t q_normalize(quaternion_t const &q);

vector_t q_vector(quaternion_t const &q);

void q_print(quaternion_t const &q, int digits);
void q_print(quaternion_t const &q);

#endif /* __COMMON_H__ */
