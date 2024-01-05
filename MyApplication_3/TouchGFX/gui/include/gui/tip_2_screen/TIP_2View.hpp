#ifndef TIP_2VIEW_HPP
#define TIP_2VIEW_HPP

#include <gui_generated/tip_2_screen/TIP_2ViewBase.hpp>
#include <gui/tip_2_screen/TIP_2Presenter.hpp>

class TIP_2View : public TIP_2ViewBase
{
public:
    TIP_2View();
    virtual ~TIP_2View() {}
    virtual void setupScreen();
    virtual void tearDownScreen();
    virtual void handleTickEvent();
    virtual void afterTransition();
    virtual void Tip_2_Pressed();
protected:
};

#endif // TIP_2VIEW_HPP
