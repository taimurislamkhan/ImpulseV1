#ifndef SETTINGS_WORK_POSITIONVIEW_HPP
#define SETTINGS_WORK_POSITIONVIEW_HPP

#include <gui_generated/settings_work_position_screen/SETTINGS_WORK_POSITIONViewBase.hpp>
#include <gui/settings_work_position_screen/SETTINGS_WORK_POSITIONPresenter.hpp>

class SETTINGS_WORK_POSITIONView : public SETTINGS_WORK_POSITIONViewBase
{
public:
    SETTINGS_WORK_POSITIONView();
    virtual ~SETTINGS_WORK_POSITIONView() {}
    virtual void setupScreen();
    virtual void tearDownScreen();
    virtual void handleTickEvent();
    void updateHomeBannerColor();
protected:
};

#endif // SETTINGS_WORK_POSITIONVIEW_HPP
