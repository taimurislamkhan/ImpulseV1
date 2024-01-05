#ifndef SETTINGS_COOLING_TIMEVIEW_HPP
#define SETTINGS_COOLING_TIMEVIEW_HPP

#include <gui_generated/settings_cooling_time_screen/SETTINGS_COOLING_TIMEViewBase.hpp>
#include <gui/settings_cooling_time_screen/SETTINGS_COOLING_TIMEPresenter.hpp>

class SETTINGS_COOLING_TIMEView : public SETTINGS_COOLING_TIMEViewBase
{
public:
    SETTINGS_COOLING_TIMEView();
    virtual ~SETTINGS_COOLING_TIMEView() {}
    virtual void setupScreen();
    virtual void tearDownScreen();
    virtual void handleTickEvent();
    void updateHomeBannerColor();
protected:
};

#endif // SETTINGS_COOLING_TIMEVIEW_HPP
