#ifndef MANUALPRESENTER_HPP
#define MANUALPRESENTER_HPP

#include <gui/model/ModelListener.hpp>
#include <mvp/Presenter.hpp>

using namespace touchgfx;

class MANUALView;

class MANUALPresenter : public touchgfx::Presenter, public ModelListener
{
public:
    MANUALPresenter(MANUALView& v);

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

    virtual ~MANUALPresenter() {};

private:
    MANUALPresenter();

    MANUALView& view;
};

#endif // MANUALPRESENTER_HPP
