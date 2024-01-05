#ifndef SETTINGS_COOLING_TIMEPRESENTER_HPP
#define SETTINGS_COOLING_TIMEPRESENTER_HPP

#include <gui/model/ModelListener.hpp>
#include <mvp/Presenter.hpp>

using namespace touchgfx;

class SETTINGS_COOLING_TIMEView;

class SETTINGS_COOLING_TIMEPresenter : public touchgfx::Presenter, public ModelListener
{
public:
    SETTINGS_COOLING_TIMEPresenter(SETTINGS_COOLING_TIMEView& v);

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

    virtual ~SETTINGS_COOLING_TIMEPresenter() {};

private:
    SETTINGS_COOLING_TIMEPresenter();

    SETTINGS_COOLING_TIMEView& view;
};

#endif // SETTINGS_COOLING_TIMEPRESENTER_HPP
