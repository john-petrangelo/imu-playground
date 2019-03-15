static float heading = 0.0;
static float pitch = 0.0;
static float roll = 0.0;
static long lastUpdateTime = 0;

// Calculate pitch, roll, and heading.
// Pitch/roll calculations take from this app note:
// http://cache.freescale.com/files/sensors/doc/app_note/AN3461.pdf?fpsp=1
// Heading calculations taken from this app note:
// http://www51.honeywell.com/aero/common/documents/myaerospacecatalog-documents/Defense_Brochures-documents/Magnetic__Literature_Application_notes-documents/AN203_Compass_Heading_Using_Magnetometers.pdf
void initAttitude()
{
  vector const accel = getAccel();
  
  heading = getMagHeading();
  pitch = atan2(accel.y, accel.z);
  roll = atan2(accel.x, accel.z);

  pitch = rad2deg(pitch);
  roll = rad2deg(roll);

  lastUpdateTime = millis();
}

void updateGyroAttitude()
{
  long now = millis();
  vector const gyro = getGyro();

  // Calculate delta time in seconds.
  float dt = (now - lastUpdateTime) * 0.001;

  heading = normalizeDeg(heading - gyro.z * dt);
  pitch   = normalizeDeg(pitch   - gyro.x * dt);
  roll    = normalizeDeg(roll    + gyro.y * dt);
  
  lastUpdateTime = now;
}

void printGyroAttitude()
{
  Serial.print("Heading:");
  Serial.print(heading);

//  Serial.print("Heading, Pitch, Roll: ");
//  Serial.print(heading, 2);
//  Serial.print(", ");
//  Serial.print(pitch, 2);
//  Serial.print(", ");
//  Serial.print(roll, 2);
}

void plotGyroAttitude()
{
  Serial.print(heading);
  Serial.print(" ");
  Serial.print(pitch);
  Serial.print(" ");
  Serial.print(roll);
}
