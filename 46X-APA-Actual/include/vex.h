//vex.h -- This file includes all external files, defines random functions    
//      and defines devices


#define BASE_FILE <C:/Users/woodc/source/VEX CODE
//
//Disable all warnings that exist bc I like using C++17 stuff
#pragma GCC diagnostic ignored "-Wc++17-extensions"
// #pragma GCC diagnostic ignored "-Wunused-variable"
#ifndef VEX_H
#define VEX_H
#define _USE_MATH_DEFINES
//Include a few basic files
#include <list>

#include <math.h>
//#include <cmath>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


#include "v5.h"
#include "v5_vcs.h"

#define ADD_THIS instances.push_back(this)

//Eliminate vex:: and std::
using namespace vex;
using namespace std;

//Make a brain
brain Brain;

//Make a controller and name it Greg
controller Greg = controller();
controller Beethoven = controller(partner);

//Include my epicly epic background files
#define BASE BASE_FILE/libs
#include BASE_FILE/basefile.h>
/*<>*/

//#error "Copy LinkedList.h from source/repos/Maple/Maple to libs"
//A vector to store the bot angles globally
PVector botAngles = PVector(0.0, 0.0, 0.0);
//The Actual Bot Angle
double& glblBotAngle = botAngles.x;
double deltaBotAngle = 0.0;
double avgBotAngle = 0.0;
double tiltAngle = botAngles.z;
double startTilt = 0.0;
bool reverseAuton = false;
ostream& operator<< (ostream& cout, PVector& v){
  cout << VECT_OUT(v);
  return cout;
}
ostream& operator<<(ostream& cout, PVector&& v){
  return operator<<(cout, v);
}

template<class Tp>
class array<Tp, 0>  {
  Tp t = Tp();
public:
  Tp* begin(){
    return &t;
  }
  Tp* end(){
    return &t;
  }
  Tp& operator[](int i){
    return t;
  };
};

//This is necessary because, due to C++ rounding errors, 0.3 * 10 could be 2.9999999
int mostlyFloor(double v) {
  //Get the decimal 
	double decimals = v - floor(v);
	if (decimals > 0.99) {
		return round(v);
	}
	return floor(v);
}
//Converts an int to a string
string toCcp(int v) {
  //The return value
	string ret;
  //Convert single digit integers to characters
	array<char, 10> arr = { '0', '1', '2', '3', '4', '5', '6', '7', '8', '9' };
	bool neg = false;
  //Check if it's negative, if so add a "-" to ret
	if (v < 0) {
		neg = true;
		v *= -1;
		ret = "-";
    ret += toCcp(v);
    return ret;
	}
  //While v is greater than 9
  //This loop converts it into a string backwards
	while (v > 9) {
    //Store a copy of v
    //For example: v = 42
		double copy = v;
    //divide the copy by 10
    //Ex: v = 4.2
		copy /= 10.0;
    //Store the last integer number of v
    //Ex: dif = mosltyFloor((4.2 - 4) * 10) [dif becomes 2]
		int dif = mostlyFloor((copy - floor(copy)) * 10.0);
    //Add the resulting character to ret
    //Ex: ret += '2';
		ret += arr[dif];
    //Store a floor of copy into v
    //Ex: v = 4
		v = floor(copy);
	}
  //Add the last integer to the array
	ret += arr[v > 9 ? 9 : v];

  //Reverse the string
	for (int i = 0; i < ret.size() / 2; i++) {
    //Swap the characters that are i distance from the front
    //And i distance from the back
		char retI = ret[i];
		ret[i] = ret[ret.size() - 1 - i];
		ret[ret.size() - 1 - i] = retI;
	}
	return ret;
}
//Converts double to string
string toCcp(double v) {
	string ret;
	array<char, 10> arr = { '0', '1', '2', '3', '4', '5', '6', '7', '8', '9' };
	bool neg = false;
	if (v < 0) {
		neg = true;
		v *= -1.0;
		ret = "-";
    ret += toCcp(v);
    return ret;
	}
	double orgCopy = v;
	while (v > 9) {
		double copy = v;
		copy /= 10.0;
		int dif = (copy - floor(copy)) * 10;
		ret += arr[dif];
		v = floor(copy);
	}
	ret += arr[(int)(v > 9 ? 9 : v)];
	for (int i = 0; i < ret.size() / 2; i++) {
		char retI = ret[i];
		ret[i] = ret[ret.size() - 1 - i];
		ret[ret.size() - 1 - i] = retI;
	}
	v = orgCopy - floor(orgCopy);
	ret += ".";
	int i = 0;
	do {
		v *= 10.0;
		ret += arr[(int)v];
		v -= floor(v);
	} while (v != 0.0 && i++ < 5);
	return ret;
}
#endif
