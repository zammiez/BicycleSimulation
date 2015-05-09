//-
// ==========================================================================
// Copyright 1995,2006,2008 Autodesk, Inc. All rights reserved.
//
// Use of this software is subject to the terms of the Autodesk
// license agreement provided at the time of installation or download,
// or which otherwise accompanies this software in either electronic
// or hard copy form.
// ==========================================================================
//+

#include <maya/MTime.h>
#include <maya/MFnMesh.h>
#include <maya/MPoint.h>
#include <maya/MFloatPoint.h>
#include <maya/MFloatPointArray.h>
#include <maya/MIntArray.h>
#include <maya/MDoubleArray.h>

#include <maya/MFnNumericAttribute.h>
#include <maya/MFnUnitAttribute.h>
#include <maya/MFnTypedAttribute.h>
//#include <maya/MFnPlugin.h>

#include <maya/MPxNode.h>
#include <maya/MObject.h>
#include <maya/MPlug.h>
#include <maya/MDataBlock.h>
#include <maya/MFnMeshData.h>

#include <maya/MIOStream.h>

#include "LSystem.h"
#include "cylinder.h"


class LSystemNode : public MPxNode
{
public:
        LSystemNode() {};//a) constructor
        virtual ~LSystemNode() {};//a)-destructor
        virtual MStatus compute(const MPlug& plug, MDataBlock& data);
        static  void*   creator();//a) -creator
        static  MStatus initialize();
		
        static MObject  time;//(iv)
        static MObject  outputMesh;//(v)
        static MTypeId  id;//a)-id
		//???
		static MObject angle;//(i) default angle
		static MObject step; //(ii) step size
		static MObject grammar;//(iii)

protected:
        MObject createMesh(const double & angle, const double & step, const MString & grammar,
			const MTime& time, MObject& outData, MStatus& stat);
};