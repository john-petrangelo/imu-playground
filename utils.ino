// Scale the value over the given range to -1..1
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
  
  if (out < 180.0) {
    out += 360.0;
  }

  if (out > 180.0) {
    out -= 360.0;
  }

  return out;
}

vector v_crossproduct(vector const *a, vector const *b) {
  vector prod;

  prod.x = (a->y * b->z) - (a->z * b->y);
  prod.y = (a->z * b->x) - (a->x * b->z);
  prod.z = (a->x * b->y) - (a->y * b->x);

  return prod;
}

float v_dotproduct(vector const *a, vector const *b)
{
  return (a->x * b->x) + (a->y * b->y) + (a->z * b->z);
}

float v_magnitude(vector const *v)
{
  return sqrt(v_dotproduct(v, v));
}