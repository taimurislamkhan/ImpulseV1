#ifndef TIP_4VIEW_HPP
#define TIP_4VIEW_HPP

#include <gui_generated/tip_4_screen/TIP_4ViewBase.hpp>
#include <gui/tip_4_screen/TIP_4Presenter.hpp>

class TIP_4View : public TIP_4ViewBase
{
public:
    TIP_4View();
    virtual ~TIP_4View() {}
    virtual void setupScreen();
    virtual void tearDownScreen();
    virtual void handleTickEvent();
    virtual void afterTransition();
    virtual void Tip_4_Pressed();
protected:
};

#endif // TIP_4VIEW_HPP
