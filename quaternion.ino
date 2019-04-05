quaternion q_make(vector const &v)
{
  return (quaternion){0, v.x, v.y, v.z};
}

quaternion q_make(float angle, vector const &v)
{
  vector scaledV = v_scale(sin(angle), v);
  return (quaternion){cos(angle), scaledV.x, scaledV.y, scaledV.z};
}

quaternion q_conjugate(quaternion const &q)
{
  return (quaternion){q.w, -q.x, -q.y, -q.z};
}

quaternion q_multiply(quaternion const &q1, quaternion const &q2)
{
  return (quaternion) {
    q2.w*q1.w - q2.x*q1.x - q2.y*q1.y - q2.z*q1.z,
    q2.w*q1.x + q2.x*q1.w - q2.y*q1.z + q2.z*q1.y,
    q2.w*q1.y + q2.x*q1.z + q2.y*q1.w - q2.z*q1.x,
    q2.w*q1.z - q2.x*q1.y + q2.y*q1.x + q2.z*q1.w
  };
}

vector q_vector(quaternion const &q)
{
  return vector{q.x, q.y, q.z};
}

void q_print(quaternion const &q, int digits)
{
  Serial.print("(");
  Serial.print(q.w, digits);
  Serial.print(",");
  Serial.print(q.x, digits);
  Serial.print(",");
  Serial.print(q.y, digits);
  Serial.print(",");
  Serial.print(q.z, digits);
  Serial.print(")");
}

void q_print(quaternion const &q)
{
  q_print(q, 2);
}