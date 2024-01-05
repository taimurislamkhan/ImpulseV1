#ifndef TIP_4PRESENTER_HPP
#define TIP_4PRESENTER_HPP

#include <gui/model/ModelListener.hpp>
#include <mvp/Presenter.hpp>

using namespace touchgfx;

class TIP_4View;

class TIP_4Presenter : public touchgfx::Presenter, public ModelListener
{
public:
    TIP_4Presenter(TIP_4View& v);

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

    virtual ~TIP_4Presenter() {};

private:
    TIP_4Presenter();

    TIP_4View& view;
};

#endif // TIP_4PRESENTER_HPP
