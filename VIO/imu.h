#ifndef __IMU_H__
#define __IMU_H__
#include "main.h"

typedef struct{
				float rol;
				float pit;
				float yaw;
}T_float_angle;

typedef struct{
				float X;
				float Y;
				float Z;
}T_float_xyz;

typedef struct{
				signed short X;
				signed short Y;
				signed short Z;
}T_int16_xyz;

typedef struct{
				signed int X;
				signed int Y;
				signed int Z;
}T_int32_xyz;

typedef struct 
{
  float x;
	float y;
	float z;
}xyz_f_t;

typedef struct
{
    xyz_f_t err;
    xyz_f_t err_tmp;
    xyz_f_t err_lpf;
    xyz_f_t err_Int;
    xyz_f_t g;

}ref_t;

void prepareData(T_int16_xyz *acc_in, T_int16_xyz *acc_out);
void IMUSO3Thread(T_int16_xyz *gyri, T_int16_xyz *acci, T_int16_xyz *magi, T_float_angle *angleo, float timei);

#endif

