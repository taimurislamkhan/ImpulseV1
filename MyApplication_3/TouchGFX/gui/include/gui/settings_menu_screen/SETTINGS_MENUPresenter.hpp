#ifndef SETTINGS_MENUPRESENTER_HPP
#define SETTINGS_MENUPRESENTER_HPP

#include <gui/model/ModelListener.hpp>
#include <mvp/Presenter.hpp>

using namespace touchgfx;

class SETTINGS_MENUView;

class SETTINGS_MENUPresenter : public touchgfx::Presenter, public ModelListener
{
public:
    SETTINGS_MENUPresenter(SETTINGS_MENUView& v);

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

    virtual ~SETTINGS_MENUPresenter() {};

private:
    SETTINGS_MENUPresenter();

    SETTINGS_MENUView& view;
};

#endif // SETTINGS_MENUPRESENTER_HPP
