//static float heading = 0.0;
//static float pitch = 0.0;
//static float roll = 0.0;
static long lastUpdateTime = 0;

static quaternion q_current_euler;

// Calculate pitch, roll, and heading.
// Pitch/roll calculations take from this app note:
// http://cache.freescale.com/files/sensors/doc/app_note/AN3461.pdf?fpsp=1
// Heading calculations taken from this app note:
// http://www51.honeywell.com/aero/common/documents/myaerospacecatalog-documents/Defense_Brochures-documents/Magnetic__Literature_Application_notes-documents/AN203_Compass_Heading_Using_Magnetometers.pdf

void init_attitude_with_accel_mag(vector_t const &accel, vector_t const &mag)
{
  attitude_t attitude = get_attitude_from_accel_mag(accel, mag);
  q_current_euler = q_make(attitude.euler);
  lastUpdateTime = millis();
}

attitude_t get_attitude_from_accel_mag(vector_t const &accel, vector_t const &mag)
{
  attitude_t attitude;
  
  // NOTE
  // For now we assume that the accelerometer is measuring g (i.e. we're not moving).
  // Later we'll need to address not using accelerometer for g when we're moving.
  
  // khat represents the direction of "up" expressed relative to the sensor.
  attitude.khat = v_normalize(v_opposite(accel));

  // ihat represents the direction of "east" expressed relative to the sensor.
  attitude.ihat = v_normalize(v_crossproduct(mag, attitude.khat));

  // jhat represents the direction of "north" expressed relative to the sensor.
  attitude.jhat = v_normalize(v_crossproduct(attitude.khat, attitude.ihat));

//  Serial.print(" [ijk]hat: ");
//  v_print(ihat);
//  v_print(jhat);
//  v_print(khat);

  // The euler vector is the orientation of the sensor expressed in global coordinates.
  // The euler vector is made up of the projection (i.e. dot product)
  // of the local vector over each of the global unit vectors (i, j, k).
  //   vector localVector = {0.0, 1.0, 0.0};
  //   float euler_x = v_dotproduct(localVector, ihat);
  //   float euler_y = v_dotproduct(localVector, jhat);
  //   float euler_z = v_dotproduct(localVector, khat);

  attitude.euler = v_normalize(vector_t{
    attitude.ihat.y,
    attitude.jhat.y,
    attitude.khat.y
  });

  return attitude;
}

void update_attitude_with_gyro()
{
  long now = millis();
  vector_t const gyro = getGyro();

  // Calculate delta time in seconds.
  float dt = (now - lastUpdateTime) * 0.001;

  float delta_roll = deg2rad(gyro.y * dt);
  float delta_pitch = deg2rad(gyro.x * dt);
  float delta_heading = deg2rad(gyro.z * dt);

  // Create quaternion representing rotation on all three axes.
  quaternion q_roll =    q_make(delta_roll / 2, vector_t{0.0, 1.0, 0.0});
  quaternion q_pitch =   q_make(delta_pitch / 2, vector_t{1.0, 0.0, 0.0});
  quaternion q_heading = q_make(delta_heading / 2, vector_t{0.0, 0.0, 1.0});

  // Combine the individual rotations into a single quaternion.
  quaternion q_rot = q_multiply(q_multiply(q_roll, q_pitch), q_heading);

  // q_euler is the euler vector expressed as a quaternion.
  quaternion q_euler = q_make(q_vector(q_current_euler));

  // rotation = q x euler x q*
  q_current_euler = q_multiply(q_multiply(q_rot, q_euler), q_conjugate(q_rot));
  q_normalize(q_current_euler);

  lastUpdateTime = now;
}

void plot_gyro_attitude()
{
//  Serial.print(heading);
//  Serial.print(" ");
//  Serial.print(pitch);
//  Serial.print(" ");
//  Serial.print(roll);
}
