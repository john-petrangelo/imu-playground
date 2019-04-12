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

  quaternion_t();
  quaternion_t(float w, float x, float y, float z);
  quaternion_t(Vector const &v);
  quaternion_t(float angle, Vector const &v);

  quaternion_t conjugate() const;
  quaternion_t multiply(quaternion_t const &other) const;

  float magnitude() const;
  quaternion_t normalize() const;

  Vector vector() const;

  void print(int digits) const;
  void print() const;
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

#endif /* __COMMON_H__ */
