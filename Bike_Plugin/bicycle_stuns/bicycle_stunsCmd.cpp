//
// Copyright (C) 
// 
// File: bicycle_stunsCmd.cpp
//
// MEL Command: bicycle_stuns
//
// Author: Maya Plug-in Wizard 2.0
//

// Includes everything needed to register a simple MEL command with Maya.
// 
#include <maya/MSimple.h>
#include <maya/MGlobal.h>
#include <maya/MSyntax.h>
#include <direct.h>
#include <maya/MArgDatabase.h>
#include <string>
#include "odedll.h"
//#include "Transformation.h"
// Use helper macro to register a command with Maya.  It creates and
// registers a command that does not support undo or redo.  The 
// created class derives off of MPxCommand.
//
DeclareSimpleCommand( bicycle_stuns, "BikeYo", "20150506");

MStatus bicycle_stuns::doIt( const MArgList& args )

{
	cout<<"Implement Me!"<<endl;
	int mie = MyMathFuncs::Miao();
	if(mie==3) cout<<"Yoye"<<mie<<endl;
	else cout<<"wuwu"<<mie<<endl;
	MStatus stat = MS::kSuccess;
	MSyntax syntax;
	double hip1,hip2,hip3;
	const char* agentFlag0 ="-hip",*agentLongflag0="_Hips";
	const char* agentFlag1_r ="-rul",*agentLongflag1_r="_RightUpLeg";
	const char* agentFlag2_r ="-rl",*agentLongflag2_r="_RightLeg";
	const char* agentFlag3_r ="-rf",*agentLongflag3_r="_RightFoot";
	const char* agentFlag4_r ="-rt",*agentLongflag4_r="_RightToes";

	const char* agentFlag1_l ="-lul",*agentLongFlag1_l="_LightUpLeg";
	const char* agentFlag2_l ="-ll",*agentLongFlag2_l="_LightLeg";
	const char* agentFlag3_l ="-lf",*agentLongFlag3_l="_LightFoot";
	const char* agentFlag4_l ="-lt",*agentLongFlag4_l="_LightToes";

	const char* agentFlag5_l ="-ls",*agentLongFlag5_l="_LeftShoulder";
	const char* agentFlag6_l ="-la",*agentLongFlag6_l="_LeftArm";
	const char* agentFlag7_l ="-lfa",*agentLongFlag7_l="_LeftForeArm";
	const char* agentFlag8_l ="-lh",*agentLongFlag8_l="_LeftHand";

	const char* agentFlag5_r ="-rs",*agentLongFlag5_r="_ReftShoulder";
	const char* agentFlag6_r ="-ra",*agentLongFlag6_r="_ReftArm";
	const char* agentFlag7_r ="-rfa",*agentLongFlag7_r="_ReftForeArm";
	const char* agentFlag8_r ="-rh",*agentLongFlag8_r="_ReftHand";

	const char* agentFlag_1 ="-head",*agentLongFlag_1="_Head";
	const char* agentFlag_2 ="-neck",*agentLongFlag_2="_Neck";
	const char* agentFlag_3 ="-chest",*agentLongFlag_3="Ypos_Head";



	syntax.addFlag(agentFlag0,agentLongflag0,MSyntax::kDouble,MSyntax::kDouble,MSyntax::kDouble);
	MArgDatabase argData(syntax,args);
	if(argData.isFlagSet(agentFlag0))
		argData.getFlagArgument(agentFlag0,0,hip1);
	    argData.getFlagArgument(agentFlag0,1,hip2);
		argData.getFlagArgument(agentFlag0,2,hip3);

	 //MGlobal::executeCommand("source\"E://Control.mel\"");
	// MGlobal::executeCommand("source  $(Configuration)/Control.mel");

	 //MFnPlugin plugin( bicycle_stuns, char ,char,char);
	 setResult( "bicycle_stuns command executed!\n" );
	 MString circle ("circle -ch on -o on -nr 0 0 1 -r 0.1 -name nurbsCircle1;");
    
	return stat;
}
