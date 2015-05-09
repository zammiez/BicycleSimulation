
#include"LSystemNode.h"

MStatus returnStatus;

#define McheckErr(stat,msg)                     \
        if ( MS::kSuccess != stat ) {   \
                cerr << msg;                            \
                return MS::kFailure;            \
        }


MObject LSystemNode::time;
MObject LSystemNode::outputMesh;
MTypeId LSystemNode::id( 0x0000001 );

MObject LSystemNode::angle;//(i) default angle
MObject LSystemNode::step; //(ii) step size
MObject LSystemNode::grammar;//()grammar

void* LSystemNode::creator()
{
        return new LSystemNode;
}

MStatus LSystemNode::initialize()
{
        MFnUnitAttribute unitAttr;
        MFnTypedAttribute typedAttr;
		MFnNumericAttribute numericAttr;

        MStatus returnStatus;
		//(i)angle
		LSystemNode::angle = numericAttr.create("angle","ag",MFnNumericData::kDouble,90.0,&returnStatus);
		McheckErr(returnStatus, "ERROR creating LSystemNode angle attribute\n");
		//(ii)size
		step = numericAttr.create("step","st",MFnNumericData::kDouble,1.0, &returnStatus);
		McheckErr(returnStatus, "ERROR creating LSystemNode step attribute\n");//(iv)time
        //(iii)grammar
		LSystemNode::grammar = typedAttr.create( "grammar", "grm", MFnData::kString, &returnStatus );
		McheckErr(returnStatus, "ERROR creating LSystemNode grammar attribute\n");
		//(iv)time
		LSystemNode::time = unitAttr.create( "time", "tm", MFnUnitAttribute::kTime, 0.0, &returnStatus );
        McheckErr(returnStatus, "ERROR creating LSystemNode time attribute\n");
		//(v)output
        LSystemNode::outputMesh = typedAttr.create( "outputMesh", "out", MFnData::kMesh, &returnStatus ); 
        McheckErr(returnStatus, "ERROR creating LSystemNode output attribute\n");
        typedAttr.setStorable(false);


		//(i)angle
		returnStatus = addAttribute(LSystemNode::angle);
		McheckErr(returnStatus, "ERROR adding angle attribute\n");
		//(ii)step
		returnStatus = addAttribute(LSystemNode::step);
		McheckErr(returnStatus, "ERROR adding step attribute\n");
		//(iii)grammar
		returnStatus = addAttribute(LSystemNode::grammar);
		McheckErr(returnStatus, "ERROR adding grammar attribute\n");
		//(iv)time
        returnStatus = addAttribute(LSystemNode::time);
        McheckErr(returnStatus, "ERROR adding time attribute\n");
		//(v)output
        returnStatus = addAttribute(LSystemNode::outputMesh);
        McheckErr(returnStatus, "ERROR adding outputMesh attribute\n");

		returnStatus = attributeAffects(LSystemNode::angle,LSystemNode::outputMesh);
        McheckErr(returnStatus, "ERROR in attributeAffects\n");
		returnStatus = attributeAffects(LSystemNode::step,LSystemNode::outputMesh);
        McheckErr(returnStatus, "ERROR in attributeAffects\n");
		returnStatus = attributeAffects(LSystemNode::grammar,LSystemNode::outputMesh);
        McheckErr(returnStatus, "ERROR in attributeAffects\n");
        returnStatus = attributeAffects(LSystemNode::time,LSystemNode::outputMesh);
        McheckErr(returnStatus, "ERROR in attributeAffects\n");

        return MS::kSuccess;
}

MObject LSystemNode::createMesh(const double & angle, const double & step, const MString & grammar,
			const MTime& time, MObject& outData, MStatus& stat)
{

        //int	numVertices, frame;
        //float	cubeSize;
        //MFloatPointArray points;
		MPointArray points;
		MIntArray faceCounts;
		MIntArray faceConnects;

        MFnMesh	meshFS;

		LSystem lsys;
		lsys.setDefaultAngle(angle);		//angle
		lsys.setDefaultStep(step);		//step size
		string gram = grammar.asChar();			//grammar
		lsys.loadProgramFromString(gram);

		std::vector<LSystem::Branch> branches;
		lsys.process((int)time.value(),branches);
		CylinderMesh*cm;
		for(int j = 0;j<branches.size();j++)
		{
			vec3 Bstart = branches[j].first;
			vec3 Bend = branches[j].second;
			MPoint b_start(Bstart[0],Bstart[1],Bstart[2]);
			MPoint b_end(Bend[0],Bend[1],Bend[2]);
			cm = new CylinderMesh(b_start,b_end);
			cm->appendToMesh(points,faceCounts,faceConnects);
		}

		/*
        // Scale the cube on the frame number, wrap every 10 frames.
        frame = (int)time.as( MTime::kFilm );
        if (frame == 0)
          frame = 1;
        cubeSize                                = 0.5f * (float)( frame % 10);

        const int numFaces                      = 6;
        numVertices                                     = 8;
        const int numFaceConnects       = 24;*/
		
		/*
        MFloatPoint vtx_1( -cubeSize, -cubeSize, -cubeSize );
        MFloatPoint vtx_2(  cubeSize, -cubeSize, -cubeSize );
        MFloatPoint vtx_3(  cubeSize, -cubeSize,  cubeSize );
        MFloatPoint vtx_4( -cubeSize, -cubeSize,  cubeSize );
        MFloatPoint vtx_5( -cubeSize,  cubeSize, -cubeSize );
        MFloatPoint vtx_6( -cubeSize,  cubeSize,  cubeSize );
        MFloatPoint vtx_7(  cubeSize,  cubeSize,  cubeSize );
        MFloatPoint vtx_8(  cubeSize,  cubeSize, -cubeSize );
        points.append( vtx_1 );
        points.append( vtx_2 );
        points.append( vtx_3 );
        points.append( vtx_4 );
        points.append( vtx_5 );
        points.append( vtx_6 );
        points.append( vtx_7 );
        points.append( vtx_8 );

        // Set up an array containing the number of vertices
        // for each of the 6 cube faces (4 verticies per face)
        //
        int face_counts[numFaces] = { 4, 4, 4, 4, 4, 4 };
        MIntArray faceCounts( face_counts, numFaces );

        // Set up and array to assign vertices from points to each face 
        //
        int face_connects[ numFaceConnects ] = 
		{	0, 1, 2, 3,
			4, 5, 6, 7,
			3, 2, 6, 5,
			0, 3, 5, 4,
			0, 4, 7, 1,
			1, 7, 6, 2  
		};
        MIntArray faceConnects( face_connects, numFaceConnects );
		
        MObject newMesh = meshFS.create(numVertices, numFaces, points, faceCounts, faceConnects, outData, &stat);

        return newMesh;*/
		MObject newMesh = meshFS.create(points.length()
										,faceCounts.length(),points,faceCounts
										,faceConnects,outData,&stat);
		return newMesh;
}

MStatus LSystemNode::compute(const MPlug& plug, MDataBlock& data)
{
        MStatus returnStatus;

        if (plug == outputMesh) {
			//angle
			MDataHandle angleData = data.inputValue(angle,&returnStatus);
			McheckErr(returnStatus, "Error getting angle data handle\n");
			double angle = angleData.asDouble();
			//step
			MDataHandle stepData = data.inputValue(step,&returnStatus);
			McheckErr(returnStatus, "Error getting step data handle\n");
			double step = stepData.asDouble();
			//grammar
			MDataHandle grammarData = data.inputValue(grammar,&returnStatus);
			McheckErr(returnStatus, "Error getting grammar data handle\n");
			MString grammar = grammarData.asString();

                /* Get time */
                MDataHandle timeData = data.inputValue( time, &returnStatus ); 
                McheckErr(returnStatus, "Error getting time data handle\n");
                MTime time = timeData.asTime();

                /* Get output object */

                MDataHandle outputHandle = data.outputValue(outputMesh, &returnStatus);
                McheckErr(returnStatus, "ERROR getting polygon data handle\n");

                MFnMeshData dataCreator;
                MObject newOutputData = dataCreator.create(&returnStatus);
                McheckErr(returnStatus, "ERROR creating outputData");

                createMesh(angle, step, grammar, time, newOutputData, returnStatus);
                McheckErr(returnStatus, "ERROR creating new Cube");

                outputHandle.set(newOutputData);
                data.setClean( plug );
        } else
        return MS::kUnknownParameter;
		return MS::kSuccess;
}
