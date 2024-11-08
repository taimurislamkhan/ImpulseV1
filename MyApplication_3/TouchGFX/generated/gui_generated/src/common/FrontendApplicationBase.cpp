/*********************************************************************************/
/********** THIS FILE IS GENERATED BY TOUCHGFX DESIGNER, DO NOT MODIFY ***********/
/*********************************************************************************/
#include <new>
#include <gui_generated/common/FrontendApplicationBase.hpp>
#include <gui/common/FrontendHeap.hpp>
#include <touchgfx/transitions/NoTransition.hpp>
#include <texts/TextKeysAndLanguages.hpp>
#include <touchgfx/Texts.hpp>
#include <touchgfx/hal/HAL.hpp>
#include <platform/driver/lcd/LCD16bpp.hpp>
#include <gui/home_screen/HOMEView.hpp>
#include <gui/home_screen/HOMEPresenter.hpp>
#include <gui/settings_menu_screen/SETTINGS_MENUView.hpp>
#include <gui/settings_menu_screen/SETTINGS_MENUPresenter.hpp>
#include <gui/settings_work_position_screen/SETTINGS_WORK_POSITIONView.hpp>
#include <gui/settings_work_position_screen/SETTINGS_WORK_POSITIONPresenter.hpp>
#include <gui/settings_screen/SETTINGSView.hpp>
#include <gui/settings_screen/SETTINGSPresenter.hpp>
#include <gui/settings_global_screen/SETTINGS_GLOBALView.hpp>
#include <gui/settings_global_screen/SETTINGS_GLOBALPresenter.hpp>
#include <gui/settings_cooling_time_screen/SETTINGS_COOLING_TIMEView.hpp>
#include <gui/settings_cooling_time_screen/SETTINGS_COOLING_TIMEPresenter.hpp>
#include <gui/manual_screen/MANUALView.hpp>
#include <gui/manual_screen/MANUALPresenter.hpp>
#include <gui/monitor_screen/MONITORView.hpp>
#include <gui/monitor_screen/MONITORPresenter.hpp>
#include <gui/alarm_history_screen/ALARM_HISTORYView.hpp>
#include <gui/alarm_history_screen/ALARM_HISTORYPresenter.hpp>
#include <gui/tip_1_screen/TIP_1View.hpp>
#include <gui/tip_1_screen/TIP_1Presenter.hpp>
#include <gui/tip_2_screen/TIP_2View.hpp>
#include <gui/tip_2_screen/TIP_2Presenter.hpp>
#include <gui/tip_3_screen/TIP_3View.hpp>
#include <gui/tip_3_screen/TIP_3Presenter.hpp>
#include <gui/tip_4_screen/TIP_4View.hpp>
#include <gui/tip_4_screen/TIP_4Presenter.hpp>
#include <gui/offsets_screen/OFFSETSView.hpp>
#include <gui/offsets_screen/OFFSETSPresenter.hpp>
#include <gui/test_press_screen/TEST_PRESSView.hpp>
#include <gui/test_press_screen/TEST_PRESSPresenter.hpp>

using namespace touchgfx;

FrontendApplicationBase::FrontendApplicationBase(Model& m, FrontendHeap& heap)
    : touchgfx::MVPApplication(),
      transitionCallback(),
      frontendHeap(heap),
      model(m)
{
    touchgfx::HAL::getInstance()->setDisplayOrientation(touchgfx::ORIENTATION_LANDSCAPE);
    touchgfx::Texts::setLanguage(GB);
    reinterpret_cast<touchgfx::LCD16bpp&>(touchgfx::HAL::lcd()).enableTextureMapperAll();
}

/*
 * Screen Transition Declarations
 */

// HOME

void FrontendApplicationBase::gotoHOMEScreenNoTransition()
{
    transitionCallback = touchgfx::Callback<FrontendApplicationBase>(this, &FrontendApplication::gotoHOMEScreenNoTransitionImpl);
    pendingScreenTransitionCallback = &transitionCallback;
}

void FrontendApplicationBase::gotoHOMEScreenNoTransitionImpl()
{
    touchgfx::makeTransition<HOMEView, HOMEPresenter, touchgfx::NoTransition, Model >(&currentScreen, &currentPresenter, frontendHeap, &currentTransition, &model);
}

// SETTINGS_MENU

void FrontendApplicationBase::gotoSETTINGS_MENUScreenNoTransition()
{
    transitionCallback = touchgfx::Callback<FrontendApplicationBase>(this, &FrontendApplication::gotoSETTINGS_MENUScreenNoTransitionImpl);
    pendingScreenTransitionCallback = &transitionCallback;
}

void FrontendApplicationBase::gotoSETTINGS_MENUScreenNoTransitionImpl()
{
    touchgfx::makeTransition<SETTINGS_MENUView, SETTINGS_MENUPresenter, touchgfx::NoTransition, Model >(&currentScreen, &currentPresenter, frontendHeap, &currentTransition, &model);
}

void FrontendApplicationBase::gotoSETTINGS_MENUScreenSlideTransitionEast()
{
    transitionCallback = touchgfx::Callback<FrontendApplicationBase>(this, &FrontendApplication::gotoSETTINGS_MENUScreenSlideTransitionEastImpl);
    pendingScreenTransitionCallback = &transitionCallback;
}

void FrontendApplicationBase::gotoSETTINGS_MENUScreenSlideTransitionEastImpl()
{
    touchgfx::makeTransition<SETTINGS_MENUView, SETTINGS_MENUPresenter, touchgfx::SlideTransition<EAST>, Model >(&currentScreen, &currentPresenter, frontendHeap, &currentTransition, &model);
}

// SETTINGS_WORK_POSITION

void FrontendApplicationBase::gotoSETTINGS_WORK_POSITIONScreenSlideTransitionEast()
{
    transitionCallback = touchgfx::Callback<FrontendApplicationBase>(this, &FrontendApplication::gotoSETTINGS_WORK_POSITIONScreenSlideTransitionEastImpl);
    pendingScreenTransitionCallback = &transitionCallback;
}

void FrontendApplicationBase::gotoSETTINGS_WORK_POSITIONScreenSlideTransitionEastImpl()
{
    touchgfx::makeTransition<SETTINGS_WORK_POSITIONView, SETTINGS_WORK_POSITIONPresenter, touchgfx::SlideTransition<EAST>, Model >(&currentScreen, &currentPresenter, frontendHeap, &currentTransition, &model);
}

void FrontendApplicationBase::gotoSETTINGS_WORK_POSITIONScreenNoTransition()
{
    transitionCallback = touchgfx::Callback<FrontendApplicationBase>(this, &FrontendApplication::gotoSETTINGS_WORK_POSITIONScreenNoTransitionImpl);
    pendingScreenTransitionCallback = &transitionCallback;
}

void FrontendApplicationBase::gotoSETTINGS_WORK_POSITIONScreenNoTransitionImpl()
{
    touchgfx::makeTransition<SETTINGS_WORK_POSITIONView, SETTINGS_WORK_POSITIONPresenter, touchgfx::NoTransition, Model >(&currentScreen, &currentPresenter, frontendHeap, &currentTransition, &model);
}

// SETTINGS

void FrontendApplicationBase::gotoSETTINGSScreenNoTransition()
{
    transitionCallback = touchgfx::Callback<FrontendApplicationBase>(this, &FrontendApplication::gotoSETTINGSScreenNoTransitionImpl);
    pendingScreenTransitionCallback = &transitionCallback;
}

void FrontendApplicationBase::gotoSETTINGSScreenNoTransitionImpl()
{
    touchgfx::makeTransition<SETTINGSView, SETTINGSPresenter, touchgfx::NoTransition, Model >(&currentScreen, &currentPresenter, frontendHeap, &currentTransition, &model);
}

// SETTINGS_GLOBAL

void FrontendApplicationBase::gotoSETTINGS_GLOBALScreenNoTransition()
{
    transitionCallback = touchgfx::Callback<FrontendApplicationBase>(this, &FrontendApplication::gotoSETTINGS_GLOBALScreenNoTransitionImpl);
    pendingScreenTransitionCallback = &transitionCallback;
}

void FrontendApplicationBase::gotoSETTINGS_GLOBALScreenNoTransitionImpl()
{
    touchgfx::makeTransition<SETTINGS_GLOBALView, SETTINGS_GLOBALPresenter, touchgfx::NoTransition, Model >(&currentScreen, &currentPresenter, frontendHeap, &currentTransition, &model);
}

// SETTINGS_COOLING_TIME

void FrontendApplicationBase::gotoSETTINGS_COOLING_TIMEScreenNoTransition()
{
    transitionCallback = touchgfx::Callback<FrontendApplicationBase>(this, &FrontendApplication::gotoSETTINGS_COOLING_TIMEScreenNoTransitionImpl);
    pendingScreenTransitionCallback = &transitionCallback;
}

void FrontendApplicationBase::gotoSETTINGS_COOLING_TIMEScreenNoTransitionImpl()
{
    touchgfx::makeTransition<SETTINGS_COOLING_TIMEView, SETTINGS_COOLING_TIMEPresenter, touchgfx::NoTransition, Model >(&currentScreen, &currentPresenter, frontendHeap, &currentTransition, &model);
}

// MANUAL

void FrontendApplicationBase::gotoMANUALScreenNoTransition()
{
    transitionCallback = touchgfx::Callback<FrontendApplicationBase>(this, &FrontendApplication::gotoMANUALScreenNoTransitionImpl);
    pendingScreenTransitionCallback = &transitionCallback;
}

void FrontendApplicationBase::gotoMANUALScreenNoTransitionImpl()
{
    touchgfx::makeTransition<MANUALView, MANUALPresenter, touchgfx::NoTransition, Model >(&currentScreen, &currentPresenter, frontendHeap, &currentTransition, &model);
}

// MONITOR

void FrontendApplicationBase::gotoMONITORScreenNoTransition()
{
    transitionCallback = touchgfx::Callback<FrontendApplicationBase>(this, &FrontendApplication::gotoMONITORScreenNoTransitionImpl);
    pendingScreenTransitionCallback = &transitionCallback;
}

void FrontendApplicationBase::gotoMONITORScreenNoTransitionImpl()
{
    touchgfx::makeTransition<MONITORView, MONITORPresenter, touchgfx::NoTransition, Model >(&currentScreen, &currentPresenter, frontendHeap, &currentTransition, &model);
}

// TIP_1

void FrontendApplicationBase::gotoTIP_1ScreenNoTransition()
{
    transitionCallback = touchgfx::Callback<FrontendApplicationBase>(this, &FrontendApplication::gotoTIP_1ScreenNoTransitionImpl);
    pendingScreenTransitionCallback = &transitionCallback;
}

void FrontendApplicationBase::gotoTIP_1ScreenNoTransitionImpl()
{
    touchgfx::makeTransition<TIP_1View, TIP_1Presenter, touchgfx::NoTransition, Model >(&currentScreen, &currentPresenter, frontendHeap, &currentTransition, &model);
}

// TIP_2

void FrontendApplicationBase::gotoTIP_2ScreenNoTransition()
{
    transitionCallback = touchgfx::Callback<FrontendApplicationBase>(this, &FrontendApplication::gotoTIP_2ScreenNoTransitionImpl);
    pendingScreenTransitionCallback = &transitionCallback;
}

void FrontendApplicationBase::gotoTIP_2ScreenNoTransitionImpl()
{
    touchgfx::makeTransition<TIP_2View, TIP_2Presenter, touchgfx::NoTransition, Model >(&currentScreen, &currentPresenter, frontendHeap, &currentTransition, &model);
}

// TIP_3

void FrontendApplicationBase::gotoTIP_3ScreenNoTransition()
{
    transitionCallback = touchgfx::Callback<FrontendApplicationBase>(this, &FrontendApplication::gotoTIP_3ScreenNoTransitionImpl);
    pendingScreenTransitionCallback = &transitionCallback;
}

void FrontendApplicationBase::gotoTIP_3ScreenNoTransitionImpl()
{
    touchgfx::makeTransition<TIP_3View, TIP_3Presenter, touchgfx::NoTransition, Model >(&currentScreen, &currentPresenter, frontendHeap, &currentTransition, &model);
}

// TIP_4

void FrontendApplicationBase::gotoTIP_4ScreenSlideTransitionEast()
{
    transitionCallback = touchgfx::Callback<FrontendApplicationBase>(this, &FrontendApplication::gotoTIP_4ScreenSlideTransitionEastImpl);
    pendingScreenTransitionCallback = &transitionCallback;
}

void FrontendApplicationBase::gotoTIP_4ScreenSlideTransitionEastImpl()
{
    touchgfx::makeTransition<TIP_4View, TIP_4Presenter, touchgfx::SlideTransition<EAST>, Model >(&currentScreen, &currentPresenter, frontendHeap, &currentTransition, &model);
}

// OFFSETS

void FrontendApplicationBase::gotoOFFSETSScreenSlideTransitionEast()
{
    transitionCallback = touchgfx::Callback<FrontendApplicationBase>(this, &FrontendApplication::gotoOFFSETSScreenSlideTransitionEastImpl);
    pendingScreenTransitionCallback = &transitionCallback;
}

void FrontendApplicationBase::gotoOFFSETSScreenSlideTransitionEastImpl()
{
    touchgfx::makeTransition<OFFSETSView, OFFSETSPresenter, touchgfx::SlideTransition<EAST>, Model >(&currentScreen, &currentPresenter, frontendHeap, &currentTransition, &model);
}

// TEST_PRESS

void FrontendApplicationBase::gotoTEST_PRESSScreenNoTransition()
{
    transitionCallback = touchgfx::Callback<FrontendApplicationBase>(this, &FrontendApplication::gotoTEST_PRESSScreenNoTransitionImpl);
    pendingScreenTransitionCallback = &transitionCallback;
}

void FrontendApplicationBase::gotoTEST_PRESSScreenNoTransitionImpl()
{
    touchgfx::makeTransition<TEST_PRESSView, TEST_PRESSPresenter, touchgfx::NoTransition, Model >(&currentScreen, &currentPresenter, frontendHeap, &currentTransition, &model);
}

void FrontendApplicationBase::gotoTEST_PRESSScreenSlideTransitionEast()
{
    transitionCallback = touchgfx::Callback<FrontendApplicationBase>(this, &FrontendApplication::gotoTEST_PRESSScreenSlideTransitionEastImpl);
    pendingScreenTransitionCallback = &transitionCallback;
}

void FrontendApplicationBase::gotoTEST_PRESSScreenSlideTransitionEastImpl()
{
    touchgfx::makeTransition<TEST_PRESSView, TEST_PRESSPresenter, touchgfx::SlideTransition<EAST>, Model >(&currentScreen, &currentPresenter, frontendHeap, &currentTransition, &model);
}
