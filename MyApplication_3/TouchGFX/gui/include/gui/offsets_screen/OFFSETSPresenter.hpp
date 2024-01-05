#ifndef OFFSETSPRESENTER_HPP
#define OFFSETSPRESENTER_HPP

#include <gui/model/ModelListener.hpp>
#include <mvp/Presenter.hpp>

using namespace touchgfx;

class OFFSETSView;

class OFFSETSPresenter : public touchgfx::Presenter, public ModelListener
{
public:
    OFFSETSPresenter(OFFSETSView& v);

    /**
     * The activate function is called automatically when this screen is "switched in"
     * (ie. made active). Initialization logic can be placed here.
     */
    virtual void activate();

    /**
     * The deactivate function is called automatically when this screen is "switched out"
     * (ie. made inactive). Teardown functionality can be placed here.
     */
    virtual void deactivate();

    virtual ~OFFSETSPresenter() {};

private:
    OFFSETSPresenter();

    OFFSETSView& view;
};

#endif // OFFSETSPRESENTER_HPP
