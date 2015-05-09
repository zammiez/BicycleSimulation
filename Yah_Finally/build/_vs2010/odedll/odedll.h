#ifdef MATHFUNCSDLL_EXPORTS
#define MATHFUNCSDLL_API __declspec(dllexport) 
#else
#define MATHFUNCSDLL_API __declspec(dllimport) 
#endif

namespace SimODE{
class BikeSimODE
    {
    public: 
		static MATHFUNCSDLL_API int Miao(); 
   };
MATHFUNCSDLL_API int InitBikeODE();
MATHFUNCSDLL_API void Seek();
MATHFUNCSDLL_API void simLoop(int pause);
MATHFUNCSDLL_API void JumpOver(int crnt,int start, int middle, int end);

//Frame
MATHFUNCSDLL_API double getFrame_Px();
MATHFUNCSDLL_API double getFrame_Py();
MATHFUNCSDLL_API double getFrame_Pz();

MATHFUNCSDLL_API double getFrame_Rx();
MATHFUNCSDLL_API double getFrame_Ry();
MATHFUNCSDLL_API double getFrame_Rz();

//Front wheel
MATHFUNCSDLL_API double getFront_Px();
MATHFUNCSDLL_API double getFront_Py();
MATHFUNCSDLL_API double getFront_Pz();

MATHFUNCSDLL_API double getFront_Rx();
MATHFUNCSDLL_API double getFront_Ry();
MATHFUNCSDLL_API double getFront_Rz();

//Back wheel
MATHFUNCSDLL_API double getBack_Px();
MATHFUNCSDLL_API double getBack_Py();
MATHFUNCSDLL_API double getBack_Pz();

MATHFUNCSDLL_API double getBack_Rx();
MATHFUNCSDLL_API double getBack_Ry();
MATHFUNCSDLL_API double getBack_Rz();

MATHFUNCSDLL_API double GetTimeStep();
}
