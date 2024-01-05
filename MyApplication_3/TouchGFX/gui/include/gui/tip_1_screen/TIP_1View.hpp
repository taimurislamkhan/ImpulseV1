#ifndef TIP_1VIEW_HPP
#define TIP_1VIEW_HPP

#include <gui_generated/tip_1_screen/TIP_1ViewBase.hpp>
#include <gui/tip_1_screen/TIP_1Presenter.hpp>

class TIP_1View : public TIP_1ViewBase
{
public:
    TIP_1View();
    virtual ~TIP_1View() {}
    virtual void setupScreen();
    virtual void tearDownScreen();
    virtual void handleTickEvent();
    virtual void afterTransition();
    virtual void Tip_1_Pressed();
protected:
};

#endif // TIP_1VIEW_HPP
