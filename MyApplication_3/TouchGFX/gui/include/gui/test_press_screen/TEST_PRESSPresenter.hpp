#ifndef TEST_PRESSPRESENTER_HPP
#define TEST_PRESSPRESENTER_HPP

#include <gui/model/ModelListener.hpp>
#include <mvp/Presenter.hpp>

using namespace touchgfx;

class TEST_PRESSView;

class TEST_PRESSPresenter : public touchgfx::Presenter, public ModelListener
{
public:
    TEST_PRESSPresenter(TEST_PRESSView& v);

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

    virtual ~TEST_PRESSPresenter() {};

private:
    TEST_PRESSPresenter();

    TEST_PRESSView& view;
};

#endif // TEST_PRESSPRESENTER_HPP
