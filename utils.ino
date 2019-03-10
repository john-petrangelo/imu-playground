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
