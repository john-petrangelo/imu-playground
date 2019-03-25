vector v_crossproduct(vector const &a, vector const &b) {
  vector prod;

  prod.x = (a.y * b.z) - (a.z * b.y);
  prod.y = (a.z * b.x) - (a.x * b.z);
  prod.z = (a.x * b.y) - (a.y * b.x);

  return prod;
}

float v_dotproduct(vector const &a, vector const &b)
{
  return (a.x * b.x) + (a.y * b.y) + (a.z * b.z);
}

float v_magnitude(vector const &v)
{
  return sqrt(v_dotproduct(v, v));
}

vector v_normalize(vector &v)
{
  float magnitude = v_magnitude(v);
  v.x /= magnitude;
  v.y /= magnitude;
  v.z /= magnitude;

  return v;
}

vector v_opposite(vector const &v)
{
  return (vector){-v.x, -v.y, -v.z};
}

vector v_scale(float scalar, vector const &v)
{
  return (vector){scalar*v.x, scalar*v.y, scalar*v.z};
}

void v_print(vector const &v)
{
  Serial.print("(");
  Serial.print(v.x, 2);
  Serial.print(",");
  Serial.print(v.y, 2);
  Serial.print(",");
  Serial.print(v.z, 2);
  Serial.print(")");
}
