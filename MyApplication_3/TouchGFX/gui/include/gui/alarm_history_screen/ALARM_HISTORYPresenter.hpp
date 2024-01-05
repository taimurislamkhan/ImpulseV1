#ifndef ALARM_HISTORYPRESENTER_HPP
#define ALARM_HISTORYPRESENTER_HPP

#include <gui/model/ModelListener.hpp>
#include <mvp/Presenter.hpp>

using namespace touchgfx;

class ALARM_HISTORYView;

class ALARM_HISTORYPresenter : public touchgfx::Presenter, public ModelListener
{
public:
    ALARM_HISTORYPresenter(ALARM_HISTORYView& v);

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

    virtual ~ALARM_HISTORYPresenter() {};

private:
    ALARM_HISTORYPresenter();

    ALARM_HISTORYView& view;
};

#endif // ALARM_HISTORYPRESENTER_HPP
