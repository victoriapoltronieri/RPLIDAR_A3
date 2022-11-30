#pragma once

//#include "main.h"
//#include "stm32f4xx_hal.h"
//#include "lidar_driver.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>
/*
const int WIDTHl = 480;
const int HEIGHTl = 272;
const int CHANNELSl = 4;
const int ORIGIN_Xl = WIDTHl / 2;
const int ORIGIN_Yl = HEIGHTl / 2;*/


//format: ARGB8888
/*const float MAX_ANGLE_DIFF_IN_LINE = 5.0;
const uint32_t COLOR_BACKGROUND = 0xff101018;
const uint32_t COLOR_BACKGROUND_GRID = 0xff242430;
const uint32_t COLOR_GRID = 0xff242430;
const uint32_t COLOR_CLOUD2 = 0xff00ffff;
const uint32_t COLOR_CLOUD1 = 0xffff00ff;
const uint32_t COLOR_CLOUD0 = 0xffffff00;*/


//uint32_t make_color(uint8_t r, uint8_t g, uint8_t b);
void draw_connected_cloud_from_map(lidar_map &map, float scale, int y_offset, float lightness, bool marks);

//Get coordinates from angle and distance.
float pt_getX(float phi, float dist,  float k);
float pt_getY(float phi, float dist, float k);

