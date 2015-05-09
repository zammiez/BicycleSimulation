// MathFuncsDll.cpp : Defines the exported functions for the DLL application.
//
#include "stdafx.h"
#include "odedll.h"
#include <stdexcept>
#include <ode\ode.h>

//#include <drawstuff\drawstuff.h>
//using namespace std;
namespace SimODE{
#define SQRT3 1.732
#define LENGTH 0.7
#define bot_LEN 0.6062//0.6062  LENGTH*SQRT3/2.0
	// chassis length
#define WIDTH 0.03
#define HEIGHT 0.03	// chassis height
#define WHEEL_WIDTH 0.03
#define RADIUS 0.18	// wheel radius
#define STARTZ 0.18	// starting height of chassis
#define CMASS 0.1		// chassis mass
#define WMASS 2	// wheel mass
#define BAR_LEN 0.45
#define pi 3.1415
#define time_step 0.04

static dWorldID world;
static dSpaceID space;
static dBodyID body[10];
static dJointID joint[10];	// joint[0] is the front wheel
static dJointGroupID contactgroup;
static dGeomID ground;
static dSpaceID car_space;
static dGeomID box[6];
static dGeomID cylinder[3];
static dGeomID ground_box;

// things that the user controls
static dReal speed=0.2,steer=0.6;	// user commands
int upFrms = 0;
bool UpWard = false;
static void nearCallback (void *data, dGeomID o1, dGeomID o2){
  int i,n;

  // only collide things with the ground
  int g1 = (o1 == ground || o1 == ground_box);
  int g2 = (o2 == ground || o2 == ground_box);
  if (!(g1 ^ g2)) return;

  const int N = 10;
  dContact contact[N];
  n = dCollide (o1,o2,N,&contact[0].geom,sizeof(dContact));
  if (n > 0) {
    for (i=0; i<n; i++) {
      contact[i].surface.mode = dContactSlip1 | dContactSlip2 |dContactApprox1;
	//dContactSoftERP | dContactSoftCFM | dContactApprox1;
      contact[i].surface.mu = 3;//dInfinity;
      contact[i].surface.slip1 = 1;
      contact[i].surface.slip2 = 1;
      //contact[i].surface.soft_erp = 0.5;
     // contact[i].surface.soft_cfm = 0.3;
      dJointID c = dJointCreateContact (world,contactgroup,&contact[i]);
      dJointAttach (c,
		    dGeomGetBody(contact[i].geom.g1),
		    dGeomGetBody(contact[i].geom.g2));
    }
  }
}

int BikeSimODE::Miao()
{
		dReal miao = 0;
		int aa = miao+3;
		return aa;
}
void get_euler(const dReal * matrix,dReal &kx,dReal &ky,dReal &kz)
{       
const dReal epsilon=0.0000001;

if(matrix[8] < 1-epsilon && matrix[8] > -1+epsilon){
        ky=-asin(matrix[8]);
        dReal c=cos(ky);
        kx= atan2(matrix[9]/c,matrix[10]/c);
        kz= atan2(matrix[4]/c,matrix[0]/c);
        }       
else    
        {       
        kz=0;   
        ky=-atan2(matrix[8],0);
        kx= atan2(-matrix[6],matrix[5]);
        }   
kx*=(180/3.1415926);
ky*=(180/3.1415926);
kz*=(180/3.1415926);

}    

int InitBikeODE()
{
	  int i;
  dMass m;
  dMatrix3 Rt;


  dInitODE2(0);
  world = dWorldCreate();
  space = dHashSpaceCreate (0);
  contactgroup = dJointGroupCreate (0);
  dWorldSetGravity (world,0,0,-0.5);
  ground = dCreatePlane (space,0,0,1,0);
 
  /////////////////////////////////////////
  ////////bar-shuzhede-body[3],box[1]//////
  /////////////////////////////////////////
  dRSetIdentity (Rt);
  dRFromAxisAndAngle (Rt,0,1,0,pi/3.0);
  body[3]=dBodyCreate(world);
  //dBodySetPosition (body[3],0.5*LENGTH-RADIUS/2.0-0.015,0,STARTZ+BAR_LEN*sin(pi/3.0)/2.0-0.015);//(body[1],0.5*LENGTH,0,STARTZ);
  dBodySetPosition (body[3],0.5*LENGTH-LENGTH/4.0,0,STARTZ+LENGTH*sin(pi/3.0)/2.0);
  dBodySetRotation (body[3],Rt);
  dMassSetBox (&m,1,LENGTH,WIDTH,HEIGHT);
  dMassAdjust (&m,CMASS);
  dBodySetMass (body[3],&m);
  box[1] = dCreateBox (0,BAR_LEN,WIDTH,HEIGHT);
  dGeomSetBody (box[1],body[3]);
  ///////////////////////////////////////////
  ////////////bar-hengzhede-body[4],box[2]///
  ///////////////////////////////////////////
  dRSetIdentity (Rt);
  dRFromAxisAndAngle (Rt,0,0,1,pi/2);//
  body[4]=dBodyCreate(world);
  const dReal *p0 = dBodyGetPosition (body[3]);
  float px0=p0[0];
  float pz0=p0[2];
  px0-=p0[0]/2.0;

  pz0+=p0[2]/2.0;
  dBodySetPosition(body[4],0.5*LENGTH-LENGTH/2.0,0,STARTZ+LENGTH*sin(pi/3.0));;//STARTZ+(BAR_LEN*sin(pi/3.0)/2.0-0.015)*2
  
  dBodySetRotation (body[4],Rt);
  dMassSetBox (&m,1,HEIGHT,WIDTH,BAR_LEN);
  dMassAdjust (&m,CMASS);
  dBodySetMass (body[4],&m);
  box[2] = dCreateBox (0,HEIGHT,WIDTH,BAR_LEN);
  dGeomSetBody (box[2],body[4]);
  /////////////////////////////////
  ////////hand_1/////////////////
  ///////////////////////////////
  body[5]=dBodyCreate(world);
  const dReal *p_bar = dBodyGetPosition (body[4]);

  dBodySetPosition(body[5],p_bar[0],p_bar[0]+BAR_LEN/2.0,p_bar[2]);;//STARTZ+(BAR_LEN*sin(pi/3.0)/2.0-0.015)*2
  dMassSetBox (&m,1,0.03,0.03,0.03);
  dMassAdjust (&m,CMASS);
  dBodySetMass (body[5],&m);
  box[3] = dCreateBox (0,0.03,0.03,0.03);
  dGeomSetBody (box[3],body[5]);
  /////////////////////////////////
  ////////hand_2/////////////////
  ///////////////////////////////
  body[6]=dBodyCreate(world);

  dBodySetPosition(body[6],p_bar[0],p_bar[0]-BAR_LEN/2.0,p_bar[2]);;//STARTZ+(BAR_LEN*sin(pi/3.0)/2.0-0.015)*2
  dMassSetBox (&m,1,0.03,0.03,0.03);
  dMassAdjust (&m,CMASS);
  dBodySetMass (body[6],&m);
  box[4] = dCreateBox (0,0.03,0.03,0.03);
  dGeomSetBody (box[4],body[6]);
  //////////////////////////////////////////////////
  /////// wheel bodies body[1]-body[2]//////////////
  /////////////////////////////////////////////////
  for (i=1; i<=2; i++) {
    body[i] = dBodyCreate (world);
    dQuaternion q;
    dQFromAxisAndAngle (q,1,0,0,M_PI*0.5);
    dBodySetQuaternion (body[i],q);
    dMassSetCylinder (&m,1,2,RADIUS,WHEEL_WIDTH);////////////
	
    dMassAdjust (&m,WMASS*i);
    dBodySetMass (body[i],&m);
    cylinder[i-1] = dCreateCCylinder(0,RADIUS,WHEEL_WIDTH);
    dGeomSetBody (cylinder[i-1],body[i]);
  }
  dBodySetPosition (body[1],0.5*LENGTH,0,STARTZ);//WIDTH*0.5
  dBodySetPosition (body[2],-0.5*LENGTH, 0,STARTZ);
    ////////////////////////////////////
  // chassis body body[0]-box[0]-bottom;
  ///////////////////////////////////////

  dRSetIdentity (Rt);
  dRFromAxisAndAngle (Rt,0,1,0,-pi/6.0);
  body[0] = dBodyCreate (world);
  dBodySetRotation (body[0],Rt);
 // const dReal *p0 = dBodyGetPosition (body[3]);
  const dReal *p1 = dBodyGetPosition (body[1]);
  float bot_temp0=p0[0]-p1[0];
  float bot_temp1=p0[1]-p1[1];
  float bot_temp2=p0[2]+p1[2];
 // dBodySetPosition (body[0],0.35*1.732-0.7,0,STARTZ+0.35*1.732/3);
  dBodySetPosition(body[0],bot_temp0/2.0,0,bot_temp2/2.0);
  dMassSetBox (&m,1,bot_LEN,WIDTH,HEIGHT);
  dMassAdjust (&m,CMASS);
  dBodySetMass (body[0],&m);
  box[0] = dCreateBox (0,bot_LEN,WIDTH,HEIGHT);
  dGeomSetBody (box[0],body[0]);

  /////////////////////////////////////////
  /////////hinge///////////////////////
  ///////////////////////////////////////
    const dReal *a0 = dBodyGetPosition (body[1]);//self
	const dReal *a1 = dBodyGetPosition (body[3]);
	const dReal *a2 = dBodyGetPosition (body[4]);
	const dReal *a3 = dBodyGetPosition (body[5]);
	//const dReal *deta= *a2-*a1;
	float tem1=a2[0]-a1[0];
	float tem2=a2[1]-a1[1];
	float tem3=a2[2]-a1[2];
	float tem4=a3[0]-a2[0];
	float tem5=a3[1]-a2[1];
	float tem6=a3[2]-a2[2];
  joint[2] = dJointCreateHinge(world,0);
  dJointAttach (joint[2] , body[3], body[0]);
  dJointSetHingeAxis (joint[2],tem1,tem2,tem3);
  const dReal *a_joint2 = dBodyGetPosition (body[3]);
  dJointSetHingeAnchor (joint[2],a_joint2[0],a_joint2[1],a_joint2[2]);

  //////////////////////////////////////////////////////////////////
  joint[3] = dJointCreateFixed (world,0);
  dJointAttach (joint[3] , body[3], body[4]);
  dJointSetFixed (joint[3] );
  
  joint[4] = dJointCreateFixed (world,0);
  dJointAttach (joint[4] , body[4], body[5]);
  dJointSetFixed (joint[4] );

  joint[5] = dJointCreateFixed (world,0);
  dJointAttach (joint[5] , body[4], body[6]);
  dJointSetFixed (joint[5] );
   //////////////////////////////////////
  ////front and back wheel hinges////////
  //////////////////////////////////////
    joint[1] = dJointCreateHinge2 (world,0);
    dJointAttach (joint[1],body[0],body[2]);
    const dReal *a = dBodyGetPosition (body[2]);
    dJointSetHinge2Anchor (joint[1],a[0],a[1],a[2]);
    dJointSetHinge2Axis1 (joint[1],0,0,1);
    dJointSetHinge2Axis2 (joint[1],0,1,0);
    dJointSetHinge2Param (joint[1],dParamSuspensionERP,0.4);
    dJointSetHinge2Param (joint[1],dParamSuspensionCFM,0.8);
    dJointSetHinge2Param (joint[1],dParamLoStop,0);
    dJointSetHinge2Param (joint[1],dParamHiStop,0);
	
	joint[0] = dJointCreateHinge2 (world,0);
    dJointAttach (joint[0],body[1],body[3]);
  
    dJointSetHinge2Anchor (joint[0],a0[0],a0[1],a0[2]);
   // dJointSetHinge2Axis1 (joint[0],tem1,tem2,tem3);
	//dJointSetHinge2Axis1 (joint[1],0,0,1);
   //dJointSetHinge2Axis2 (joint[0],tem4,tem5,tem6);
    dJointSetHinge2Param (joint[0],dParamSuspensionERP,0.4);
    dJointSetHinge2Param (joint[0],dParamSuspensionCFM,0.8);
    dJointSetHinge2Param (joint[0],dParamLoStop,0);
    dJointSetHinge2Param (joint[0],dParamHiStop,0);


    car_space = dSimpleSpaceCreate (space);
    dSpaceSetCleanup (car_space,0);
    dSpaceAdd (car_space,box[0]);
    dSpaceAdd (car_space,box[1]);
    dSpaceAdd (car_space,box[2]);
	dSpaceAdd (car_space,box[3]);
    dSpaceAdd (car_space,cylinder[0]);
  dSpaceAdd (car_space,cylinder[1]);
 

  // environment
  ground_box = dCreateBox (space,2,1.5,1);
  dMatrix3 R;
  dRFromAxisAndAngle (R,0,1,0,-0.15);
  dGeomSetPosition (ground_box,5,0,-0.34);
  dGeomSetRotation (ground_box,R);
  return 0;
}

static dReal thetad = 0;
static dReal vd = 0;
static dReal theta = 0;
static dReal state[4] = {0,0,0,0};
static dReal tau = 0;
static dReal thetaDDot = 0;
static dReal thetaDot = 0;
//consts
static dReal Kv1 = 32;
static dReal Kp1 = 256;
static dReal Kt1 = 0;

void Seek()
{
	//Seek()
	const dReal * goal = dGeomGetPosition(ground_box);
	const dReal * FPos = dBodyGetPosition(body[1]);
	const dReal * FVel = dBodyGetLinearVel(body[1]);

	dReal e[2];
	e[0] = goal[0] - FPos[0];
	e[1] = goal[1] - FPos[1];

	dReal v[2];
	v[0] = FVel[0];
	v[1] = FVel[1];

	dReal dist = sqrt(e[0]*e[0]+e[1]*e[1]);
	dReal _v_ = sqrt(v[0]*v[0]+v[1]*v[1]);
	if(_v_==0)
	{
		thetad = 0;
		return;
	}
	float ex = (e[0]*v[0]+e[1]*v[1])/_v_;
	float ey2 = e[0]*e[0]+e[1]*e[1] - ex*ex;
	if(ey2<0)
	{
		thetad = 0;
		return;
	}

	float ey = sqrt(ey2);
	thetad = atan2(e[1],e[0]);
	//thetad = atan2(ey,ex);
	dReal theta;
	theta = atan2(v[1],v[0]);

	dReal the_D = thetad*180/M_PI;
	dReal The = theta*180/M_PI;

	float thetaDot = state[3];
	//theta = 0;
	tau =  ( -Kv1 * thetaDot - Kp1 * theta + Kp1 * thetad);
	dReal MaxTau = 0.05;
	if(tau>MaxTau) tau = MaxTau;
	if(tau<-MaxTau) tau = -MaxTau;
	state[3] = tau;
	tau*=time_step;
	steer += tau;
	return ;
	//printf("%d\n",thetad);
}
//frame
static double Frame_Px;
static double Frame_Py;
static double Frame_Pz;

//front
static double Front_Px;
static double Front_Py;
static double Front_Pz;

static double Front_Rx;
static double Front_Ry;
static double Front_Rz;

//back
static double Back_Px;
static double Back_Py;
static double Back_Pz;

void simLoop (int pause)
{
	//body0: Frame Update
	const dReal * frameBody0 = dBodyGetPosition(body[0]);
	Frame_Px = frameBody0[0];
	Frame_Py = frameBody0[1];
	Frame_Pz = frameBody0[2];

	//body1: Front Wheel update
	const dReal * frontBody1 = dBodyGetPosition(body[1]);
	Front_Px = frontBody1[0];
	Front_Py = frontBody1[1];
	Front_Pz = frontBody1[2];
	const dReal * frontRBody1 = dBodyGetRotation(body[1]);
	get_euler(frontRBody1,Front_Rx,Front_Ry,Front_Rz);

	//body2: Back Wheel update
	const dReal * backBody2 = dBodyGetPosition(body[2]);
	Back_Px = backBody2[0];
	Back_Py = backBody2[1];
	Back_Pz = backBody2[2];



	int i;
  if (true) {

	const dReal *a0 = dBodyGetPosition (body[1]);//self
	const dReal *a1 = dBodyGetPosition (body[3]);
	const dReal *a2 = dBodyGetPosition (body[4]);
	const dReal *a3 = dBodyGetPosition (body[5]);

	float tem1=a2[0]-a1[0];
	float tem2=a2[1]-a1[1];
	float tem3=a2[2]-a1[2];
	float tem4=a3[0]-a2[0];
	float tem5=a3[1]-a2[1];
	float tem6=a3[2]-a2[2];
	dJointSetHinge2Axis1 (joint[0],tem1,tem2,tem3);
    dJointSetHinge2Axis2 (joint[0],tem4,tem5,tem6);
    // motor

	dJointSetHinge2Param (joint[0],dParamVel2,speed);
    dJointSetHinge2Param (joint[0],dParamFMax2,0.1);
	dReal Aim = M_PI/3;
    // steering
	int a;
	a = 0;
	const dReal * Vel = dBodyGetLinearVel(body[1]);
	dReal _Vel_ = sqrt(Vel[0]*Vel[0]+Vel[1]*Vel[1]);
	//Go to dest
	//if(speed>0||_Vel_<=0)
	//	Seek();
    dReal v = steer - dJointGetHingeAngle (joint[2]);
    if (v > 0.1) v = 0.1;
    if (v < -0.1) v = -0.1;
    v *= 10.0;
    dJointSetHingeParam (joint[2],dParamVel,v);
    dJointSetHingeParam (joint[2],dParamFMax,0.2);
    dJointSetHingeParam (joint[2],dParamLoStop,-1.75);
    dJointSetHingeParam (joint[2],dParamHiStop,1.75);
    dJointSetHingeParam (joint[2],dParamFudgeFactor,0.1);

	//Up Stunt
	if(UpWard)
	{
		float dur = 40;
		JumpOver(upFrms,20,dur/2/time_step,dur/time_step);
		upFrms++;
		if(upFrms>=(dur/time_step))
		{
			upFrms = 0;
			UpWard = false;
		}
	}

    dSpaceCollide (space,0,&nearCallback);
    dWorldStep (world,time_step);

    // remove all contact joints
    dJointGroupEmpty (contactgroup);
   
 }
}

void JumpOver(int crnt,int start, int middle, int end)
{
	dReal H_r = 1;
	dReal W_r = 1;
	if(crnt>=start)
	{
		if(crnt<=middle)
		{
			H_r = 1 - (float)crnt/(float)middle;
			W_r = H_r*0.6;
			//if(crnt<=(float)middle*2/3) W_r*=0.6;
			//else{W_r*=1.4; H_r*=0.6;};
		}
		else if(crnt<=end)
		{
			H_r = 0;
			W_r = 0;
			//H_r = (float)(crnt-middle)/(float)(end-middle);
			//W_r = H_r;
		}
	}
	else 
	{
		H_r = 0.5;
		W_r = 0.0;
	}
	H_r*=0.8;
	W_r*=0.8;
	dReal H_Force = CMASS*10*4*H_r; 
	dReal W_Force = WMASS*0.8 *W_r;
	dBodyAddForce(body[4],0.0, 0.0,H_Force );
	dBodyAddForce(body[2],0.0, 0.0,W_Force );
}

double GetTimeStep()
{
	double timedt = time_step;
	return timedt;
}


//Frame
double getFrame_Px(){return Frame_Px;}
double getFrame_Py(){return Frame_Py;}
double getFrame_Pz(){return Frame_Pz;}

//Front wheel
double getFront_Px(){return Front_Px;}
double getFront_Py(){return Front_Py;}
double getFront_Pz(){return Front_Pz;}

double getFront_Rx(){return Front_Rx;}
double getFront_Ry(){return Front_Ry;}
double getFront_Rz(){return Front_Rz;}

//Back wheel
double getBack_Px(){return Back_Px;}
double getBack_Py(){return Back_Py;}
double getBack_Pz(){return Back_Pz;}

}