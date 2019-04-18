#ifndef ARDUINO
#include <math.h>
#include "common.h"

#define PI M_PI
#endif

// Scale the value from the input range to the output range.
float fmap(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Convert from radians to degrees.
float rad2deg(float rad) {
    return rad * 180.0 / PI;
}

// Convert from degrees to radians.
float deg2rad(float deg) {
    return deg * PI / 180.0;
}

// Normalize the degrees to -180..180.
float normalizeDeg(float in) {
  float out = fmod(in, 360.0);
  
  if (out < -180.0) {
    out += 360.0;
  } else if (out > 180.0) {
    out -= 360.0;
  }

  return out;
}

// Print the attitude as specified by heading, pitch, and roll.
void print_attitude(float heading, float pitch, float roll) {
#ifdef ARDUINO
  Serial.print(" HPR:");
  Serial.print(heading);
  Serial.print(" ");
  Serial.print(pitch);
  Serial.print(" ");
  Serial.print(roll);
#endif
}
