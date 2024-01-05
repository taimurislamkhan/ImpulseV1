#ifndef SETTINGSVIEW_HPP
#define SETTINGSVIEW_HPP

#include <gui_generated/settings_screen/SETTINGSViewBase.hpp>
#include <gui/settings_screen/SETTINGSPresenter.hpp>

class SETTINGSView : public SETTINGSViewBase
{
public:
    SETTINGSView();
    virtual ~SETTINGSView() {}
    virtual void setupScreen();
    virtual void tearDownScreen();
    virtual void handleTickEvent();
    void updateHomeBannerColor();
protected:
};

#endif // SETTINGSVIEW_HPP
