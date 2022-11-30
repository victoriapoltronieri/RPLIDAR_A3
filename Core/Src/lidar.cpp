#include <math.h>
//#include "main.h"
#include "lidar.h"
//#include "stm32f4xx_hal.h"

const int WIDTHl = 480;
const int HEIGHTl = 272;
const int CHANNELSl = 4;
const int ORIGIN_Xl = WIDTHl / 2;
const int ORIGIN_Yl = HEIGHTl / 2;


#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))


/*uint32_t make_color(uint8_t r, uint8_t g, uint8_t b)
{
	return (0xff << 24) | (r << 16) | (g << 8) | b;
}*/

void draw_connected_cloud_from_map(lidar_map &map, float scale, int y_offset, float lightness, bool marks) {


	if(map.cnt == 0) return;
	if(map.dmax == 0) return;
	if(scale == 0) scale = (HEIGHT) * 0.7 / map.dmax;


	int cnt = 1;

	float first_pt_x = pt_getX(map.angles[1], map.distances[1], scale);	//auto first_pt = cyl_to_cart(cloud.pts[0], scale);
	float first_pt_y = pt_getY(map.angles[1], map.distances[1], scale);
	float first_angle = map.angles[1];
	float first_dist = map.distances[1];

	float last_pt_x = first_pt_x;	//auto last_pt = first_pt;
	float last_pt_y = first_pt_y;
	float last_angle = first_angle;
	float last_dist = first_dist;

	float pt_max_x = pt_getX(map.amax, map.dmax, scale);
	float pt_max_y = pt_getY(map.amax, map.dmax, scale);

	float max_cart_dist = hypot(pt_max_x - (WIDTH/2), pt_max_y - (HEIGHT/2));

	for(int i = 1; i < map.cnt-5; i+=4) {
		float angle = map.angles[i];
		float distance = map.distances[i];


			float pt_x = pt_getX(angle, distance, scale);
			float pt_y = pt_getY(angle, distance, scale);

			if(distance > 0 && last_dist > 0 && ((angle - last_angle) < MAX_ANGLE_DIFF_IN_LINE))
			draw_colorful_line(float(last_pt_x), float(last_pt_y + y_offset), float(pt_x), float(pt_y) + y_offset, max_cart_dist);

			last_angle = angle;
			last_dist = distance;
			last_pt_x = pt_x;
			last_pt_y = pt_y;
			cnt++;

	}

	if (marks) {
			draw_point( pt_max_x, pt_max_y + y_offset, make_color(255, 255, 255));
			draw_mark(  pt_max_x, pt_max_y+ y_offset, unsigned(map.dmax / 1000), unsigned(map.dmax) % 1000, make_color(255, 255, 255));
	}
}

float pt_getX(float phi, float dist,  float k) {
	return round(dist * std::sin(phi * (acos(-1) / 180.0)) * k) + ORIGIN_Xl;
}


float pt_getY(float phi, float dist, float k){
	return round(dist * std::cos(phi * (acos(-1) / 180.0)) * k) + ORIGIN_Yl;
}


