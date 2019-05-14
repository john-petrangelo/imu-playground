#ifndef __COMMON_H__
#define __COMMON_H__

#ifndef ARDUINO
#include <ostream>
#endif

// Custom types
struct Vector {
  float x;
  float y;
  float z;

  Vector();
  Vector(float x, float y, float z);

  bool operator==(Vector const &other) const;
  
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
  Quaternion(Vector const &ihat, Vector const &jhat, Vector const &khat);
  Quaternion(float yaw, float pitch, float roll);
  Quaternion(Vector const &v);
  Quaternion(float angle, Vector const &v);

  bool operator==(Quaternion const &other) const;

  Quaternion conjugate() const;
  Quaternion multiply(Quaternion const &other) const;

  float magnitude() const;
  Quaternion normalize() const;

  Vector vector() const;

  void print(int digits) const;
  void print() const;
};

// These are useful for debugging unit tests.
#ifndef ARDUINO
  std::ostream& operator<<(std::ostream& os, Vector const &v);
  std::ostream& operator<<(std::ostream& os, Quaternion const &q);
#endif

struct Attitude {
  Vector euler;
  Vector ihat;
  Vector jhat;
  Vector khat;
  Quaternion q;
  float heading;
  float pitch;
  float roll;
};

// Prototypes for utils functions.
float fmap(float x, float in_min, float in_max, float out_min, float out_max);
float rad2deg(float rad);
float deg2rad(float deg);
float normalizeDeg(float in);
inline float sqr(float x) { return x * x; }

// Prototypes for attitude functions.
float getSimpleHeading(Vector const &mag);
float getSimplePitch(Vector const &accel);
float getSimpleRoll(Vector const &accel);
Attitude get_attitude_from_accel_mag(Vector const &accel, Vector const &mag);

#endif /* __COMMON_H__ */
