#ifndef CreateLSystemCmd_H_
#define CreateLSystemCmd_H_

#include <maya/MSyntax.h>
#include <maya/MArgDatabase.h>
#include <maya/MPxCommand.h>
#include <string>
#include "odedll.h"
class LSystemCmd : public MPxCommand
{
public:
    LSystemCmd();
    virtual ~LSystemCmd();
    static void* creator() { return new LSystemCmd(); }
    MStatus doIt( const MArgList& args );
	static MSyntax newSyntax();
};

#endif