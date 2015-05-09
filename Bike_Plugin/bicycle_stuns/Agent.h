// Agent.h: interface for the CAgent class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AGENT_H)
#define AGENT_H

#include "Transformation.h"
#include "AgentDisplay.h"
#include <vector>

using namespace std;


#ifndef BEHAVIORTYPE
#define BEHAVIORTYPE
#endif

class CAgent  
{
public:

    CAgent();
    virtual ~CAgent();
    //Constructor
    //CAgent(float* color, CEnvironment* env);
    //Initiate the states
    void SetInitState(float pos[2], float angle);
    //Display the agent
    void Display();
    //Sense the environment according to different behaviors
    void Sense(int type);
    //Agent control
    void Control();
    //Agent act: performing EulerStep for this particle
    void FindDeriv();
    void UpdateState();
    void Act(float deltaT);

    //Initiate values for agent properties and other settings
    static void InitValues();
    void SetVTheta(vec2& V);
    void SetV0();
    vec2 LocalToWorld(vec2& l);
    vec2 WorldToLocal(vec2& w);

    

    //Member variables:
    //Current frame number of the animation
    int frameNum;
    //Agent color
    float color[3];
    //Simulation time step
    float deltaT;
    //Environment information
    //Show leader indicator
    bool showLeader;

  
 //   int dimInput;

    //State vector
 //   float state[4];
    //Input vector
 //   float input[2];
    //Derivative vector
 //   float deriv[4];

    //Control inputs:
    //Desired velocity
////    float vd;
    //Desired orientation
    float thetad;

    //Agent global position
//    vec2 GPos;
    //Goal position
//    vec2 goal;

    //Needed in Wander behavior
    //Wander velocity
//    vec2 vWander;
    //Nominal velocity
 //   vec2 v0;

    //Static function and variables:
    //Clear the agent list
 //   static void ClearAgents();
    //Agent list
//    static vector<CAgent*> agents;
    //Radius of the bounding sphere of every agent
//    static float radius;
    //Debug mode indicator
 //   static bool debug;
    //Mass of every agent
 //   static float Mass;
    //Moment of inertia of every agent
 //   static float Inertia;
    //Maximum velocity
 //   static float MaxVelocity;
    //Maximum force
 //   static float MaxForce;
    //Maximum torque
 //   static float MaxTorque;
    //Maximum angular velocity
 //   static float MaxAngVel;


    //Velocity control: f = m * Kv0 * (vd - v)
 /*   static float Kv0;
   
};

inline void Truncate(float& value, float minV, float maxV){
    if (value < minV)
        value = minV;
    else
        if (value > maxV)
            value = maxV;
}
#endif // !defined(AGENT_H)*/





















