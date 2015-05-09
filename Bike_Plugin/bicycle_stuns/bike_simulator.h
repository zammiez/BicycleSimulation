#ifndef Sim_H_
#define Sim_H_

#include <string>
#include <vector>
//#include "Transformation.h"

class Simulator
{
public:
      float m;
	 // mat3 M;
	//  vec3 v;
	  float w;
	  float wa;//pedals
	  float wb;//rear wheel
	  float alpha;//pedal ratio;
	  //vec3 n;//rotational axes;
	  Simulator();
	  Simulator(float m);
	  ~Simulator();


};
#endif