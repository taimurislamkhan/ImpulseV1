#ifndef TIP_2PRESENTER_HPP
#define TIP_2PRESENTER_HPP

#include <gui/model/ModelListener.hpp>
#include <mvp/Presenter.hpp>

using namespace touchgfx;

class TIP_2View;

class TIP_2Presenter : public touchgfx::Presenter, public ModelListener
{
public:
    TIP_2Presenter(TIP_2View& v);

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

    virtual ~TIP_2Presenter() {};

private:
    TIP_2Presenter();

    TIP_2View& view;
};

#endif // TIP_2PRESENTER_HPP
