// Custom types

struct vector {
  float x;
  float y;
  float z;
};

struct quaternion {
  float w;
  float x;
  float y;
  float z;
};

struct attitude_t {
  vector euler;
  vector ihat;
  vector jhat;
  vector khat;
  float heading;
  float pitch;
  float roll;
};

// Vector functions
vector v_crossproduct(vector const &a, vector const &b);
float v_dotproduct(vector const &a, vector const &b);

float v_magnitude(vector const &v);
vector v_normalize(vector const &v);
vector v_opposite(vector const &v);
vector v_scale(float scalar, vector const &v);

void v_print(vector const &v, int digits);
void v_print(vector const &v);

// Quaternion functions
quaternion q_make(vector const &v);
quaternion q_make(float angle, vector const &v);

quaternion q_conjugate(quaternion const &q);
quaternion q_multiply(quaternion const &q1, quaternion const &q2);

float q_magnitude(quaternion const &q);
quaternion q_normalize(quaternion &q);

vector q_vector(quaternion const &q);

void q_print(quaternion const &q, int digits);
void q_print(quaternion const &q);
