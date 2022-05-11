#include <math.h>

#define STEP_angle 1.8
struct hor_triangle{
    float base_launch;
    float base_rocket;
    float launch_rocket;
    float angle;
};
struct GPS_data{
    float lon;
    float lat;
    float height;
};
struct vert_tri{
    float rocket_ground;
    float base_rocket;
    float rocketground_base;
    float angle;
};

//function macros
struct hor_triangle get_hor_triangle(struct GPS_data lat_long, float init_lat, float init_lon);
int get_move_distance(float pre_angle, float cur_angle, int motor_pot);