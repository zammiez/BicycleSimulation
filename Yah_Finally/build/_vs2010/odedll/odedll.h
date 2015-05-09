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

//Body 0 : Frame
MATHFUNCSDLL_API double GetFramePos_Body0x();
MATHFUNCSDLL_API double GetFramePos_Body0y();
MATHFUNCSDLL_API double GetFramePos_Body0z();

MATHFUNCSDLL_API double GetFrameRot_Body0x();
MATHFUNCSDLL_API double GetFrameRot_Body0y();
MATHFUNCSDLL_API double GetFrameRot_Body0z();

//Body 1 : Front Wheel
MATHFUNCSDLL_API double GetFrontWheelPos_Body1x();
MATHFUNCSDLL_API double GetFrontWheelPos_Body1y();
MATHFUNCSDLL_API double GetFrontWheelPos_Body1z();
/*
MATHFUNCSDLL_API double getFront_R00();
MATHFUNCSDLL_API double getFront_R01();
MATHFUNCSDLL_API double getFront_R02();
MATHFUNCSDLL_API double getFront_R10();
MATHFUNCSDLL_API double getFront_R11();
MATHFUNCSDLL_API double getFront_R12();
MATHFUNCSDLL_API double getFront_R20();
MATHFUNCSDLL_API double getFront_R21();
MATHFUNCSDLL_API double getFront_R22();*/

MATHFUNCSDLL_API double getFrontTrans_0();
MATHFUNCSDLL_API double getFrontTrans_1();
MATHFUNCSDLL_API double getFrontTrans_2();
MATHFUNCSDLL_API double getFrontTrans_3();
MATHFUNCSDLL_API double getFrontTrans_4();
MATHFUNCSDLL_API double getFrontTrans_5();
MATHFUNCSDLL_API double getFrontTrans_6();
MATHFUNCSDLL_API double getFrontTrans_7();
MATHFUNCSDLL_API double getFrontTrans_8();
MATHFUNCSDLL_API double getFrontTrans_9();
MATHFUNCSDLL_API double getFrontTrans_10();
MATHFUNCSDLL_API double getFrontTrans_11();
MATHFUNCSDLL_API double getFrontTrans_12();
MATHFUNCSDLL_API double getFrontTrans_13();
MATHFUNCSDLL_API double getFrontTrans_14();
MATHFUNCSDLL_API double getFrontTrans_15();


//Body 2 : Back Wheel
MATHFUNCSDLL_API double GetBackWheelPos_Body2x();
MATHFUNCSDLL_API double GetBackWheelPos_Body2y();
MATHFUNCSDLL_API double GetBackWheelPos_Body2z();

MATHFUNCSDLL_API double GetBackWheelRot_Body2x();
MATHFUNCSDLL_API double GetBackWheelRot_Body2y();
MATHFUNCSDLL_API double GetBackWheelRot_Body2z();
//Body 3 : (Handle) Bar
MATHFUNCSDLL_API double GetBarPos_Body3x();
MATHFUNCSDLL_API double GetBarPos_Body3y();
MATHFUNCSDLL_API double GetBarPos_Body3z();

MATHFUNCSDLL_API double GetBarRot_Body3x();
MATHFUNCSDLL_API double GetBarRot_Body3y();
MATHFUNCSDLL_API double GetBarRot_Body3z();
//Body 4 : Handle
MATHFUNCSDLL_API double GetHandle_Body4x();
MATHFUNCSDLL_API double GetHandle_Body4y();
MATHFUNCSDLL_API double GetHandle_Body4z();
//_useless_Body 5 : LeftHand
//_useless_Body 6 : RightHand
//_useless_Body 7 : ...yellow?
//_useless_Body 8 : ...yellow?
//_to be added_ Body 7 : ?? 7 again. ??
//_to be added_ Body 8 : ?? Seat

MATHFUNCSDLL_API double GetTimeStep();
}
