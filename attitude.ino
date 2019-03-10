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

  heading = rad2deg(heading);
  pitch = rad2deg(pitch);
  roll = rad2deg(roll);

  lastUpdateTime = millis();
}

void updateAttitude()
{
  long dt = millis() - lastUpdateTime;
  vector const gyro = getGyro();

  long dz = gyro.z;
  
  heading += dz * dt / 1000.0;
  
  vector const accel = getAccel();
  pitch = atan2(accel.y, accel.z);
  roll = atan2(accel.x, accel.z);

  lastUpdateTime += dt;
}

void printAttitude()
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

void plotAttitude()
{
  Serial.print(heading);
  Serial.print(" ");
  Serial.print(pitch);
  Serial.print(" ");
  Serial.print(roll);
}
