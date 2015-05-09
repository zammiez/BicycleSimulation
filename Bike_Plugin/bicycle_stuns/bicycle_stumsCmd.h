#ifndef CreateBiCmd_H_
#define CreateBiCmd_H_

#include <maya/MSimple.h>
#include <maya/MGlobal.h>
#include <maya/MSyntax.h>
#include <direct.h>
#include <maya/MArgDatabase.h>
#include <string>
#include "odedll.h"

class bicycle_stunsCmd : public MPxCommand
{
	public:
    bicycle_stunsCmd();
    virtual ~bicycle_stunsCmd();
    static void* creator() { return new bicycle_stunsCmd(); }
    MStatus doIt( const MArgList& args );
	static MSyntax newSyntax();
}
#endif