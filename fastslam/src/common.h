
#pragma once

#include <iostream>
#include <fstream>
#include <map>
#include <sstream>
#include <vector>
#include <set>
#include <tuple>

using std::vector;
using namespace std;

struct OdoReading{
	float r1;
	float t;
	float r2;
};

struct RadarReading {
	long long id;
	float range;
	float bearing;
};
