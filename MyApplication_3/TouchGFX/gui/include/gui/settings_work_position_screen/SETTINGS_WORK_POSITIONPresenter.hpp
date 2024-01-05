#ifndef SETTINGS_WORK_POSITIONPRESENTER_HPP
#define SETTINGS_WORK_POSITIONPRESENTER_HPP

#include <gui/model/ModelListener.hpp>
#include <mvp/Presenter.hpp>

using namespace touchgfx;

class SETTINGS_WORK_POSITIONView;

class SETTINGS_WORK_POSITIONPresenter : public touchgfx::Presenter, public ModelListener
{
public:
    SETTINGS_WORK_POSITIONPresenter(SETTINGS_WORK_POSITIONView& v);

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

    virtual ~SETTINGS_WORK_POSITIONPresenter() {};

private:
    SETTINGS_WORK_POSITIONPresenter();

    SETTINGS_WORK_POSITIONView& view;
};

#endif // SETTINGS_WORK_POSITIONPRESENTER_HPP
