#ifndef MANUALVIEW_HPP
#define MANUALVIEW_HPP

#include <gui_generated/manual_screen/MANUALViewBase.hpp>
#include <gui/manual_screen/MANUALPresenter.hpp>

class MANUALView : public MANUALViewBase
{
public:
    MANUALView();
    virtual ~MANUALView() {}
    virtual void setupScreen();
    virtual void tearDownScreen();
    virtual void handleTickEvent();
    virtual void afterTransition();
    void updateHomeBannerColor();
protected:
};

#endif // MANUALVIEW_HPP
