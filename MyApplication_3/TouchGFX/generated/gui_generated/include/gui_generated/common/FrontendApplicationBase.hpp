/*********************************************************************************/
/********** THIS FILE IS GENERATED BY TOUCHGFX DESIGNER, DO NOT MODIFY ***********/
/*********************************************************************************/
#ifndef FRONTENDAPPLICATIONBASE_HPP
#define FRONTENDAPPLICATIONBASE_HPP

#include <mvp/MVPApplication.hpp>
#include <gui/model/Model.hpp>

class FrontendHeap;

class FrontendApplicationBase : public touchgfx::MVPApplication
{
public:
    FrontendApplicationBase(Model& m, FrontendHeap& heap);
    virtual ~FrontendApplicationBase() { }

    virtual void changeToStartScreen()
    {
        gotoHOMEScreenNoTransition();
    }

    // HOME
    void gotoHOMEScreenNoTransition();

    // SETTINGS_MENU
    void gotoSETTINGS_MENUScreenNoTransition();

    void gotoSETTINGS_MENUScreenSlideTransitionEast();

    // SETTINGS_WORK_POSITION
    void gotoSETTINGS_WORK_POSITIONScreenSlideTransitionEast();

    void gotoSETTINGS_WORK_POSITIONScreenNoTransition();

    // SETTINGS
    void gotoSETTINGSScreenNoTransition();

    // SETTINGS_GLOBAL
    void gotoSETTINGS_GLOBALScreenNoTransition();

    // SETTINGS_COOLING_TIME
    void gotoSETTINGS_COOLING_TIMEScreenNoTransition();

    // MANUAL
    void gotoMANUALScreenNoTransition();

    // MONITOR
    void gotoMONITORScreenNoTransition();

    // TIP_1
    void gotoTIP_1ScreenNoTransition();

    // TIP_2
    void gotoTIP_2ScreenNoTransition();

    // TIP_3
    void gotoTIP_3ScreenNoTransition();

    // TIP_4
    void gotoTIP_4ScreenSlideTransitionEast();

    // OFFSETS
    void gotoOFFSETSScreenSlideTransitionEast();

    // TEST_PRESS
    void gotoTEST_PRESSScreenNoTransition();

    void gotoTEST_PRESSScreenSlideTransitionEast();

protected:
    touchgfx::Callback<FrontendApplicationBase> transitionCallback;
    FrontendHeap& frontendHeap;
    Model& model;

    // HOME
    void gotoHOMEScreenNoTransitionImpl();

    // SETTINGS_MENU
    void gotoSETTINGS_MENUScreenNoTransitionImpl();

    void gotoSETTINGS_MENUScreenSlideTransitionEastImpl();

    // SETTINGS_WORK_POSITION
    void gotoSETTINGS_WORK_POSITIONScreenSlideTransitionEastImpl();

    void gotoSETTINGS_WORK_POSITIONScreenNoTransitionImpl();

    // SETTINGS
    void gotoSETTINGSScreenNoTransitionImpl();

    // SETTINGS_GLOBAL
    void gotoSETTINGS_GLOBALScreenNoTransitionImpl();

    // SETTINGS_COOLING_TIME
    void gotoSETTINGS_COOLING_TIMEScreenNoTransitionImpl();

    // MANUAL
    void gotoMANUALScreenNoTransitionImpl();

    // MONITOR
    void gotoMONITORScreenNoTransitionImpl();

    // TIP_1
    void gotoTIP_1ScreenNoTransitionImpl();

    // TIP_2
    void gotoTIP_2ScreenNoTransitionImpl();

    // TIP_3
    void gotoTIP_3ScreenNoTransitionImpl();

    // TIP_4
    void gotoTIP_4ScreenSlideTransitionEastImpl();

    // OFFSETS
    void gotoOFFSETSScreenSlideTransitionEastImpl();

    // TEST_PRESS
    void gotoTEST_PRESSScreenNoTransitionImpl();

    void gotoTEST_PRESSScreenSlideTransitionEastImpl();
};

#endif // FRONTENDAPPLICATIONBASE_HPP
