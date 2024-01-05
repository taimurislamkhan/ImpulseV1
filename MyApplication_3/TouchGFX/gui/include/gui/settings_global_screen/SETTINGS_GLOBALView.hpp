#ifndef SETTINGS_GLOBALVIEW_HPP
#define SETTINGS_GLOBALVIEW_HPP

#include <gui_generated/settings_global_screen/SETTINGS_GLOBALViewBase.hpp>
#include <gui/settings_global_screen/SETTINGS_GLOBALPresenter.hpp>

class SETTINGS_GLOBALView : public SETTINGS_GLOBALViewBase
{
public:
    SETTINGS_GLOBALView();
    virtual ~SETTINGS_GLOBALView() {}
    virtual void setupScreen();
    virtual void tearDownScreen();
    virtual void handleTickEvent();
    virtual void encoder_bypass_clicked();
    virtual void PREHEAT_ENABLE_PRESSED();
//    virtual void BOSS_HEIGHT_SLIDER_CHANGED();
//    virtual void PREHEAT_SLIDER_CHANGED();
//    virtual void PLATEN_SPEED_CURVE_CHANGED();
    virtual void afterTransition();
    void updateHomeBannerColor();
protected:
};

#endif // SETTINGS_GLOBALVIEW_HPP
