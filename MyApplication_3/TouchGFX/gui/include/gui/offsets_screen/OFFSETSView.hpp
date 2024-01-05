#ifndef OFFSETSVIEW_HPP
#define OFFSETSVIEW_HPP

#include <gui_generated/offsets_screen/OFFSETSViewBase.hpp>
#include <gui/offsets_screen/OFFSETSPresenter.hpp>

class OFFSETSView : public OFFSETSViewBase
{
public:
    OFFSETSView();
    virtual ~OFFSETSView() {}
    virtual void setupScreen();
    virtual void tearDownScreen();
    virtual void handleTickEvent();
protected:
};

#endif // OFFSETSVIEW_HPP
