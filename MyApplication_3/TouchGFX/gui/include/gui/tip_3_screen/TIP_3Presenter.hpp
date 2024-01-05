#ifndef TIP_3PRESENTER_HPP
#define TIP_3PRESENTER_HPP

#include <gui/model/ModelListener.hpp>
#include <mvp/Presenter.hpp>

using namespace touchgfx;

class TIP_3View;

class TIP_3Presenter : public touchgfx::Presenter, public ModelListener
{
public:
    TIP_3Presenter(TIP_3View& v);

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

    virtual ~TIP_3Presenter() {};

private:
    TIP_3Presenter();

    TIP_3View& view;
};

#endif // TIP_3PRESENTER_HPP
