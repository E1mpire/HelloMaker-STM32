#ifndef KINEMATICS_H
#define KINEMATICS_H

#include "config.h"
#define RADIUS_WAY  1
class Kinematics
{
  public:
    struct rpm {
      int motor1;
      int motor2;
      int motor3;
      int motor4;
    };
    struct velocities {
      float linear_x;
      float linear_y;
      float angular_z;
    };
	struct pwm
	{
		int motor1;
		int motor2;
		int motor3;
		int motor4;
	};
    Kinematics(int motor_max_rpm, float wheel_diameter,float base_width, float wheels_x_distance, float wheels_y_distance);
    velocities getVelocities(int rpm1, int rpm2);
    rpm getRPM(float linear_x, float linear_y, float angular_z);

  private:
	rpm calculateRPM(float linear_x, float linear_y, float angular_z);
    int max_rpm_;
	float wheel_diameter_;
	float base_width_;
	float wheels_x_distance_;
	float wheels_y_distance_;
	float wheel_circumference_;
};

#endif
