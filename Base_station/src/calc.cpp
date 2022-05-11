#include "calc.h"

struct hor_triangle get_hor_triangle(struct GPS_data lat_long, float init_lat, float init_lon){
    int lat_sign;
    struct hor_triangle tri;
    // normalizes the lat and long
    lat_long.lat = lat_long.lat - init_lat;
    lat_long.lon = lat_long.lon - init_lon;
    // calculates the luanch_rocket leg
    tri.launch_rocket = pow(lat_long.lat,2) + pow(lat_long.lon,2);
    tri.launch_rocket = sqrt(tri.launch_rocket);
    if(lat_long.lat > 0.0){
        lat_sign = 1;
    }
    else{
        lat_sign = -1;
    }
    // calculates the base_rocket leg
    tri.base_rocket = pow((tri.base_launch + lat_long.lon),2) + pow(lat_long.lat,2);
    tri.base_rocket = sqrt(tri.base_rocket);
    /* 
    calculates the angle for the triangle
    uses the law of cosines: angle = cos^-1((base_launch^2 + base_rocket^2 - launch_rocket^2) / (2 * base_launch * base_rocket))
    after multiply by lat_sign to make the angle positive or negative
    */
    tri.angle = pow(tri.base_launch,2) + pow(tri.base_rocket,2) - pow(tri.launch_rocket,2);
    tri.angle = tri.angle / (2 * tri.base_launch * tri.base_rocket);
    tri.angle = acos(tri.angle);
    tri.angle = tri.angle * lat_sign;
    return tri;
}

struct vert_tri get_vert_triangle(float base_rocket, struct GPS_data height){
    struct vert_tri tri;
    tri.rocket_ground = height.height;
    tri.rocketground_base = base_rocket;
    tri.base_rocket = pow(tri.rocket_ground,2) + pow(tri.rocketground_base,2);
    tri.base_rocket = sqrt(tri.base_rocket);
    
}

int get_move_distance(float pre_angle, float cur_angle, int motor_pot){
    int step_dis;
    step_dis = (int)((pre_angle - cur_angle) / STEP_angle);
    step_dis = motor_pot + step_dis;
    return step_dis;
}
