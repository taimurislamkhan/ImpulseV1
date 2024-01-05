#ifndef SETTINGS_GLOBALPRESENTER_HPP
#define SETTINGS_GLOBALPRESENTER_HPP

#include <gui/model/ModelListener.hpp>
#include <mvp/Presenter.hpp>

using namespace touchgfx;

class SETTINGS_GLOBALView;

class SETTINGS_GLOBALPresenter : public touchgfx::Presenter, public ModelListener
{
public:
    SETTINGS_GLOBALPresenter(SETTINGS_GLOBALView& v);

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

    virtual ~SETTINGS_GLOBALPresenter() {};

private:
    SETTINGS_GLOBALPresenter();

    SETTINGS_GLOBALView& view;
};

#endif // SETTINGS_GLOBALPRESENTER_HPP
