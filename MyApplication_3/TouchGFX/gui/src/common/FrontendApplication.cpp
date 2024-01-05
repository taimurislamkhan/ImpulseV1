#include <gui/common/FrontendApplication.hpp>

FrontendApplication::FrontendApplication(Model& m, FrontendHeap& heap)
    : FrontendApplicationBase(m, heap)
{

}

//#include <gui/common/FrontendApplication.hpp>
//#include <TouchGFXHAL.hpp>
//
//extern "C" int yourFlag; // Replace "yourFlag" with the actual name of your flag
//
//// Declare and define the inactivity counter
//extern "C" int inactivityCounter;
//
//FrontendApplication::FrontendApplication(Model& m, FrontendHeap& heap)
//    : FrontendApplicationBase(m, heap)
//{
//}
//
//void FrontendApplication::handleTickEvent()
//{
//    model.tick();
//    FrontendApplicationBase::handleTickEvent();
//
//    if (yourFlag == 1)
//    {
//        yourFlag = 0;
//        gotoHOMEScreenNoTransition();
//    }
//}




