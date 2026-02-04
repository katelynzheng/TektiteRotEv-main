#ifndef UTIL_H
#define UTIL_H

float reduce_0_to_360(float angle);
float reduce_negative_180_to_180(float angle);
float reduce_negative_90_to_90(float angle);
float to_rad(float angle_deg);
float to_deg(float angle_rad);

#endif