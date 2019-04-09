// Custom types

struct vector_t {
  float x;
  float y;
  float z;
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

/********************
 * Vector functions *
 ********************/
vector_t v_crossproduct(vector_t const &a, vector_t const &b);
float v_dotproduct(vector_t const &a, vector_t const &b);

float v_magnitude(vector_t const &v);
vector_t v_normalize(vector_t const &v);
vector_t v_opposite(vector_t const &v);
vector_t v_scale(float scalar, vector_t const &v);

void v_print(vector_t const &v, int digits);
void v_print(vector_t const &v);

// Quaternion functions
quaternion_t q_make(vector_t const &v);
quaternion_t q_make(float angle, vector_t const &v);

quaternion_t q_conjugate(quaternion_t const &q);
quaternion_t q_multiply(quaternion_t const &q1, quaternion_t const &q2);

float q_magnitude(quaternion_t const &q);
quaternion_t q_normalize(quaternion_t &q);

vector_t q_vector(quaternion_t const &q);

void q_print(quaternion_t const &q, int digits);
void q_print(quaternion_t const &q);
