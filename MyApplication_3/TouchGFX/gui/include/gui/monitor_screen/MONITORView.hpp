#ifndef MONITORVIEW_HPP
#define MONITORVIEW_HPP

#include <gui_generated/monitor_screen/MONITORViewBase.hpp>
#include <gui/monitor_screen/MONITORPresenter.hpp>

class MONITORView : public MONITORViewBase
{
public:
    MONITORView();
    virtual ~MONITORView() {}
    virtual void setupScreen();
    virtual void tearDownScreen();
    virtual void handleTickEvent();
    void updateHomeBannerColor();
    virtual void afterTransition();
    virtual void Brass_Mode_Pressed();
protected:
};

#endif // MONITORVIEW_HPP
