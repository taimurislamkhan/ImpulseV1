#ifndef TIP_3VIEW_HPP
#define TIP_3VIEW_HPP

#include <gui_generated/tip_3_screen/TIP_3ViewBase.hpp>
#include <gui/tip_3_screen/TIP_3Presenter.hpp>

class TIP_3View : public TIP_3ViewBase
{
public:
    TIP_3View();
    virtual ~TIP_3View() {}
    virtual void setupScreen();
    virtual void tearDownScreen();
    virtual void handleTickEvent();
    virtual void afterTransition();
    virtual void Tip_3_Pressed();
protected:
};

#endif // TIP_3VIEW_HPP
