vector_t v_crossproduct(vector_t const &a, vector_t const &b) {
  vector_t prod;

  prod.x = (a.y * b.z) - (a.z * b.y);
  prod.y = (a.z * b.x) - (a.x * b.z);
  prod.z = (a.x * b.y) - (a.y * b.x);

  return prod;
}

float v_dotproduct(vector_t const &a, vector_t const &b)
{
  return (a.x * b.x) + (a.y * b.y) + (a.z * b.z);
}

float v_magnitude(vector_t const &v)
{
  return sqrt(v_dotproduct(v, v));
}

vector_t v_normalize(vector_t const &v)
{
  float magnitude = v_magnitude(v);
  vector_t vout = v;
  vout.x /= magnitude;
  vout.y /= magnitude;
  vout.z /= magnitude;

  return vout;
}

vector_t v_opposite(vector_t const &v)
{
  return (vector_t){-v.x, -v.y, -v.z};
}

vector_t v_scale(float scalar, vector_t const &v)
{
  return (vector_t){scalar*v.x, scalar*v.y, scalar*v.z};
}

void v_print(vector_t const &v, int digits)
{
  Serial.print(v.x, digits);
  Serial.print(",");
  Serial.print(v.y, digits);
  Serial.print(",");
  Serial.print(v.z, digits);
}

void v_print(vector_t const &v)
{
  v_print(v, 2);
}
