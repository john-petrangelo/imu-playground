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

struct Quaternion {
  float w;
  float x;
  float y;
  float z;

  Quaternion();
  Quaternion(float w, float x, float y, float z);
  Quaternion(Vector const &v);
  Quaternion(float angle, Vector const &v);

  Quaternion conjugate() const;
  Quaternion multiply(Quaternion const &other) const;

  float magnitude() const;
  Quaternion normalize() const;

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
