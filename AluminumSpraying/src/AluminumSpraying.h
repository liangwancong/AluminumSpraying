#pragma once
#include <iostream>
#include "parameters.h"
using namespace AluminumParameters;

int getMatchMsg(char* output_result_json, char * cloud_front_file, char * cloud_file, char* model_file, double cloud_length = CloudLength, int cloud_pos = CloudPos, float head_offset = HeadOffset, float mid_offset = MidOffset, float tail_offset = TailOffset);
