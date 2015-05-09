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
		cout<<"Time: "<< time <<endl;
		cout<<"- frame:\t"<<SimODE::GetFramePos_Body0x()<<"\t"<<SimODE::GetFramePos_Body0y()<<endl;

		//Keyframe Frame_body0
		MString FramePosx;
		FramePosx.set(SimODE::GetFramePos_Body0x());
		MString FramePosy;
		FramePosy.set(SimODE::GetFramePos_Body0y());
		MString FramePosz;
		FramePosz.set(SimODE::GetFramePos_Body0z());
		MString MoveKeyCommand = ("select -r frame; move " + FramePosx +" "+FramePosz +" "+FramePosy +";");
		MGlobal::executeCommand(MoveKeyCommand);
		MGlobal::executeCommand("setKeyframe;");
		cout<<MoveKeyCommand.asChar()<<endl;
		
		//Keyframe FrontWheel_body1
		MString T0,T1,T2,T3,T4,T5,T6,T7,T8,T9,T10,T11,T12,T13,T14,T15;
		T0.set(SimODE::getFrontTrans_0());
		T1.set(SimODE::getFrontTrans_1());
		T2.set(SimODE::getFrontTrans_2());
		T3.set(SimODE::getFrontTrans_3());
		T4.set(SimODE::getFrontTrans_4());
		T5.set(SimODE::getFrontTrans_5());
		T6.set(SimODE::getFrontTrans_6());
		T7.set(SimODE::getFrontTrans_7());
		T8.set(SimODE::getFrontTrans_8());
		T9.set(SimODE::getFrontTrans_9());
		T10.set(SimODE::getFrontTrans_10());
		T11.set(SimODE::getFrontTrans_11());
		T12.set(SimODE::getFrontTrans_12());
		T13.set(SimODE::getFrontTrans_13());
		T14.set(SimODE::getFrontTrans_14());
		T15.set(SimODE::getFrontTrans_15());
		MoveKeyCommand = "xform -matrix " + 
									T0 + " " + T1 +" " + T2 +" " + T3 +" " +
									T4 + " " + T5 +" " + T6 +" " + T7 +" " +
									T8 + " " + T9 +" " + T10+" " + T11+" " +
									T12+ " " + T13+" " + T14+" " + T15+" ;" 
									;
		MoveKeyCommand = "select -r front_wheel;" + MoveKeyCommand;

		MGlobal::executeCommand(MoveKeyCommand);
		MGlobal::executeCommand("setKeyframe;");
		/*MString Px,Py,Pz;
		Px.set(SimODE::GetFrontWheelPos_Body1x());
		Py.set(SimODE::GetFrontWheelPos_Body1y());
		Pz.set(SimODE::GetFrontWheelPos_Body1z());

		MString R00,R01,R02,
				R10,R11,R12,
				R20,R21,R22;
		R00.set(SimODE::getFront_R00());	R01.set(SimODE::getFront_R01());	R02.set(SimODE::getFront_R02());
		R10.set(SimODE::getFront_R10());	R11.set(SimODE::getFront_R11());	R12.set(SimODE::getFront_R12());
		R20.set(SimODE::getFront_R20());	R21.set(SimODE::getFront_R21());	R22.set(SimODE::getFront_R22());

		MoveKeyCommand = "xform -matrix " + 
									R00 + " " + R01 +" " + R02 +" 0 " +
									R10 + " " + R11 +" " + R12 +" 0 " +
									R20 + " " + R21 +" " + R22 +" 0 " +
									Px  + " " + Py  +" " + Pz  +" 1 ;";
		MoveKeyCommand = "select -r front_wheel;" + MoveKeyCommand;

		MGlobal::executeCommand(MoveKeyCommand);
		MGlobal::executeCommand("setKeyframe;");
		*/
		//Keyframe BackWheel_body2
		MString BackWheelPosx;
		BackWheelPosx.set(SimODE::GetBackWheelPos_Body2x());
		MString BackWheelPosy;
		BackWheelPosy.set(SimODE::GetBackWheelPos_Body2y());
		MString BackWheelPosz;
		BackWheelPosz.set(SimODE::GetBackWheelPos_Body2z());
		MoveKeyCommand = ("select -r back_wheel; move " + BackWheelPosx +" "+BackWheelPosy +" "+BackWheelPosz +";");
		MGlobal::executeCommand(MoveKeyCommand);
		MGlobal::executeCommand("setKeyframe;");

		cout<<"set:"<<time<<endl;
	}

	return MStatus::kSuccess;
}

