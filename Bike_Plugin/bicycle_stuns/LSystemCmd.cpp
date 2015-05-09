#include "LSystemCmd.h"


#include <maya/MGlobal.h>

#include <maya/MSyntax.h>
#include <maya/MArgDatabase.h>
#include <list>
#include "LSystem.h"


LSystemCmd::LSystemCmd() : MPxCommand()
{
}

LSystemCmd::~LSystemCmd() 
{
}

MSyntax LSystemCmd::newSyntax()
{
	MSyntax syntax;

	return syntax;
}

MStatus LSystemCmd::doIt( const MArgList& args )
{
	//From Frame a to Frame b;
	double FrameEnd = 100;
	int tt = SimODE::BikeSimODE::Miao();
	SimODE::InitBikeODE();
	cout<<"MIAO_001:"<<tt<<endl;
	//double x = SimODE::GetFramePos_Body0x();
	//cout <<x <<endl;
	int t_i = 0;
	for(double time = 0;time<=FrameEnd;time+=SimODE::GetTimeStep(),t_i++)
	{
		SimODE::simLoop(0);
		//Set Time
		if((t_i)%10!=0) continue;
		MString crntTime;
		crntTime.set(time);
		MGlobal::executeCommand("currentTime "+crntTime+";");

		//Keyframe Frame_body0
		MString FramePosx,FramePosy,FramePosz;
		FramePosx.set(SimODE::getFrame_Px());
		FramePosy.set(SimODE::getFrame_Py());
		FramePosz.set(SimODE::getFrame_Pz());
		MString MoveKeyCommand = ("setAttr \"frame.translate\" " + FramePosx +" "+FramePosy +" "+FramePosz +";");	
		MGlobal::executeCommand(MoveKeyCommand);
		MGlobal::executeCommand("setKeyframe frame.translate;");
		//Keyframe FrontWheel_body1
		MString FrontPx,FrontPy,FrontPz;
		FrontPx.set(SimODE::getFront_Px());
		FrontPy.set(SimODE::getFront_Py());
		FrontPz.set(SimODE::getFront_Pz());
		 MoveKeyCommand = ("setAttr \"front_wheel.translate\" " + FrontPx +" "+FrontPy +" "+FrontPz +";");	
		MGlobal::executeCommand(MoveKeyCommand);
		MGlobal::executeCommand("setKeyframe front_wheel.translate;");

		MString FrontRx,FrontRy,FrontRz;
		FrontRx.set(SimODE::getFront_Rx());
		FrontRy.set(SimODE::getFront_Ry());
		FrontRz.set(SimODE::getFront_Rz());
		 MoveKeyCommand = ("setAttr \"front_wheel.rotate\" " + FrontRx +" "+FrontRy +" "+FrontRz +";");	
		MGlobal::executeCommand(MoveKeyCommand);
		MGlobal::executeCommand("setKeyframe front_wheel.rotate;");

		//Keyframe BackWheel Position.



		cout<<"set:"<<time<<endl;
	}

	return MStatus::kSuccess;
}

