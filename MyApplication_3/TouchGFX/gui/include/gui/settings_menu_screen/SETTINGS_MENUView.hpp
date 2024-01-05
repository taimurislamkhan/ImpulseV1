#ifndef SETTINGS_MENUVIEW_HPP
#define SETTINGS_MENUVIEW_HPP

#include <gui_generated/settings_menu_screen/SETTINGS_MENUViewBase.hpp>
#include <gui/settings_menu_screen/SETTINGS_MENUPresenter.hpp>

class SETTINGS_MENUView : public SETTINGS_MENUViewBase
{
public:
    SETTINGS_MENUView();
    virtual ~SETTINGS_MENUView() {}
    virtual void setupScreen();
    virtual void tearDownScreen();
    virtual void handleTickEvent();
    void updateHomeBannerColor();
protected:
};

#endif // SETTINGS_MENUVIEW_HPP
