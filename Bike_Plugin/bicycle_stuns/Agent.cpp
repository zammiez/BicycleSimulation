// Agent.cpp: implementation of the CAgent class.
//
//////////////////////////////////////////////////////////////////////
/*
#include "Agent.h"

//Construct static variables
//Their real values are set in static function CAgent::InitValues()
vector<CAgent*> CAgent::agents;
bool CAgent::debug = false;
float CAgent::radius = 20.0;
float CAgent::Mass = 1.0;
float CAgent::Inertia = 1.0;
float CAgent::MaxVelocity = 15.0;
float CAgent::MaxForce = 10.0;
/*
float CAgent::MaxTorque = 40.0;
float CAgent::MaxAngVel = 10.0;

float CAgent::Kv0 = 1.0;
float CAgent::Kp1 = 1.0;
float CAgent::Kv1 = 1.0;
float CAgent::KArrival = 1.0;
float CAgent::KDeparture = 1.0;
float CAgent::KNoise = 1.0;
float CAgent::KWander = 1.0;
float CAgent::KAvoid = 1.0;
float CAgent::TAvoid = 1.0;
float CAgent::RNeighborhood = 1.0;
float CAgent::KSeparate = 1.0;
float CAgent::KAlign = 1.0;
float CAgent::KCohesion = 1.0;

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////
vec2 CAgent::WorldToLocal(vec2 &w)
{
	vec2 l;
	float s = sin(state[1]);
	float c = cos(state[1]);
	l[0] = c * w[0] + s * w[1];
	l[1] = -s * w[0] + c * w[1];
	return l;
}

vec2 CAgent::LocalToWorld(vec2 &l)
{
	vec2 w;
	float s = sin(state[1]);
	float c = cos(state[1]);
	w[0] = c * l[0] - s * l[1];
	w[1] = s * l[0] + c * l[1];
	return w;
}

void CAgent::SetVTheta(vec2 &V)
{
	vd = V.Length();
	if (vd < 0.0001){
		thetad = 0.0;
	}
	else{
		if (abs(V[0]) < 0.0001){
			if (V[1] > 0)
				thetad = M_PI / 2.0;
			else
				thetad = M_PI / -2.0;
		}
		else
			thetad = atan2(V[1], V[0]);
	}
	
	ClampAngle(thetad);
}

CAgent::CAgent()
{

}

CAgent::~CAgent()
{

}

void CAgent::Display()
{
	glPushMatrix();//32 unit matrix,push current
	glTranslatef(GPos[0], rootHeight[frameNum], GPos[1]);
	if (showLeader && this == CAgent::agents[0]){
		float red[] = {1.0, 0.0, 0.0};
		glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, red);
		glPushMatrix();
		glTranslatef(0.0, 100.0, 0.0);
		glRotatef(90.0, 1.0, 0.0, 0.0);
		glutSolidCone(10.0, 20.0, 24, 24);
		glPopMatrix();
	}
	
	float specular[] = {1.0, 1.0, 1.0, 1.0};
	glMaterialfv(GL_FRONT, GL_AMBIENT, color);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, color);	
	glMaterialfv(GL_FRONT, GL_SPECULAR, specular);
	glMaterialf(GL_FRONT, GL_SHININESS, 50);
	
	if (CAgent::debug){
		glutSolidSphere(10.0, 24, 24);
		glDisable(GL_LIGHTING);
		glPushMatrix();
		glColor3f(1.0, 0.0, 0.0);
		glRotatef(-state[1] / M_PI * 180.0, 0.0, 1.0, 0.0);
		glLineWidth(3.0);
		glBegin(GL_LINES);
		glVertex3f(0.0, 0.0, 0.0); glVertex3f(12 + state[2], 0.0, 0.0);
		glEnd();
		glPopMatrix();
		glPushMatrix();
		glColor3f(0.0, 1.0, 0.0);
		glRotatef(-thetad / M_PI * 180.0, 0.0, 1.0, 0.0);
		glLineWidth(3.0);
		glBegin(GL_LINES);
		glVertex3f(0.0, 0.0, 0.0); glVertex3f(12 + vd, 0.0, 0.0);
		glEnd();
		glPopMatrix();
		glLineWidth(1.0);
		glEnable(GL_LIGHTING);
	}
	else{
		glRotatef(90.0f - state[1] * Rad2Deg , 0.0, 1.0, 0.0);
		//glutSolidSphere(6.0f, 12, 12);
		glCallList(AgentDisplay::listNames + frameNum);
	}
	glPopMatrix();
}

void CAgent::ClearAgents(){
	for (unsigned int i = 0; i < agents.size(); i++)
		delete CAgent::agents[i];
	CAgent::agents.clear();	
}

CAgent::CAgent(float* color, CEnvironment* env){
	memcpy(this->color, color, sizeof(float) * 3);
	dimState = 4;
	dimInput = 2;
	frameNum = rand() % totalFrames;
	this->env = env;

	float angle=  float(rand() % 360) / 180.0 * M_PI;
	vWander[0] = cos(angle) * CAgent::KWander;
	vWander[1] = sin(angle) * CAgent::KWander;
	angle = float(rand() % 360) / 180.0 * M_PI;
	v0[0] = cos(angle) * CAgent::MaxVelocity / 2.0;
	v0[1] = sin(angle) * CAgent::MaxVelocity / 2.0;
	CAgent::agents.push_back(this);
}

void CAgent::SetInitState(float pos[], float angle)
{
	int i;
	GPos[0] = pos[0];
	GPos[1] = pos[1];
	for (i = 0; i < dimState; i++){
		state[i] = 0.0;
	}
	for (i = 0; i < dimInput; i++)
		input[i] = 0.0;
	state[1] = angle;
}

void CAgent::Sense(int type)
{
	showLeader = (type == LEADER);
	goal = env->goal;
	switch(type) {
	case SEEK: Seek(); break;
	case FLEE: Flee(); break;
	case ARRIVAL: Arrival(); break;
	case DEPARTURE: Departure(); break;
	case WANDER: Wander(); break;
	case AVOID: Avoid(); break;
	case SEPARATION: Separation(); break;
	case ALIGNMENT: Alignment(); break;
	case COHESION: Cohesion(); break;
	case FLOCKING: Flocking(); break;
	case LEADER: Leader(); break;
	}
}

void CAgent::Act(float deltaT)
{
	int dframe;
	this->deltaT = deltaT;
	FindDeriv();
	UpdateState();
	if (abs(state[2]) < 0.001)
		dframe = 0;
	else{
		dframe= int(state[2] / CAgent::MaxVelocity * 2 + 1); 
	}
	frameNum += dframe;
	if (frameNum >= totalFrames){
		frameNum = frameNum - totalFrames;
	}
}

void CAgent::SetV0()
{
	v0 = env->goal - GPos;
	v0 = v0.Normalize() * CAgent::MaxVelocity / 2;
}

/*
 *	Set initial value for control and behavior settings
 *  You need to find out appropriate values for:
 *  CAgent::Kv0, CAgent::Kp1, CAgent::Kv1, CAgent::KArrival, CAgent::KDeparture,
 *  CAgent::KNoise,	CAgent::KWander, CAgent::KAvoid, CAgent::TAvoid, CAgent::RNeighborhood,
 *  CAgent::KSeparate, CAgent::KAlign, CAgent::KCohesion.
 *//*
void CAgent::InitValues()
{
	// Add your code here
	CAgent::Kv0=10.0;//w1=2,w2=4
    CAgent::Kp1= 256.0;
	CAgent::Kv1=32.0;
	KArrival=0.05;
	KDeparture=15;
	KWander=8;
	KAvoid=1;
	TAvoid=20;
	KAlign=20;
	KSeparate=1000;
	KCohesion=0.05;
	RNeighborhood=800;

}

/*
 *	You should apply the control rules given desired velocity vd and desired orientation thetad.
 *  Velocity control: input[0] = f = m * Kv0 * (vd - v)
 *  Heading control: input[1] = tau = I * ( -Kv1 * thetaDot - Kp1 * theta + Kp1 * thetad)
 *  This function sets input[0] and input[1] appropriately after being called.    vec2 LocalToWorld(vec2& l);
    vec2 WorldToLocal(vec2& w);
 *//*
void CAgent::Control()
{
	// Add your code here
	input[0] = Mass * Kv0 * ( vd - state[2]);//f
	input[1] = Inertia * ( Kp1*(thetad-state[1]) - Kv1 * state[3]);//torque
	
}*/

/*
 *	Compute derivative vector given input and state vectors
 *  This function sets derive vector to appropriate values after being called
 */
/*void CAgent::FindDeriv()
{
	// Add your code here
        deriv[0]=state[2];   //v*12.5
		deriv[1]=state[3];   //w*6.25
		deriv[2]=input[0]/Mass;//a*
		deriv[3]=input[1]/Inertia;//w_dot
}
*/
/*
 *	Update the state vector given derivative vector
 *  Compute global position and store it in GPos
 *  Perform validation check to make sure all values are within MAX values
 *//*
void CAgent::UpdateState()
{
	// Add your code here
	
	vec2 v;
	if(abs(state[1]-thetad)<=0.1)
		state[1]=thetad;
	else{
		state[1]+= deriv[1]*deltaT;
	}
	state[0] = deriv[0]*deltaT;//+0.5*deriv[2]*deltaT;
   //theta,6.25
	state[2]+= deriv[2]*deltaT;//v.12.5
	state[3]+= deriv[3]*deltaT;//w

	Truncate(state[2],0, MaxVelocity);
	Truncate(state[3],-MaxAngVel, MaxAngVel);
	//if(state[3]>MaxAngVel)state[3]=MaxAngVel;
	Truncate(input[0], -MaxForce, MaxForce);
	Truncate(input[1],-MaxTorque, MaxTorque);

	GPos+=LocalToWorld(vec2(state[2],0));
	Truncate(GPos[0],-2093,2093);
	Truncate(GPos[1],-2093,2093);
}

*/
/*
 *	Flocking behavior
 *  Utilize the Separation, Arrival behaviors to determine the desired velocity vector
 *  You need to find the leader, who is always the first agent in CAgent::agents
 *  You need to compute the desired velocity and desired orientation
 *  Store them into vd and thetad respectively
 *  return a vec2 that represents the goal velocity with its direction being thetad and its norm being vd
 */
/*
vec2 CAgent::Leader()
{
	// Replace the following with your code
	vec2 tmp;
	vec2 vlf;
	if(GPos[0]== agents[0]->GPos[0]&&(GPos[1] == agents[0]->GPos[1]))
		vlf = 50*Arrival(); 
   else
        vlf = 6*Separation() + 0.8*Arrival(); 

		vd = vlf.Length(); 
		thetad = atan2(vlf[1], vlf[0]); 
		return tmp;
}
*/