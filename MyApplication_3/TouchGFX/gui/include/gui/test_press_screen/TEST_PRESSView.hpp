#ifndef TEST_PRESSVIEW_HPP
#define TEST_PRESSVIEW_HPP

#include <gui_generated/test_press_screen/TEST_PRESSViewBase.hpp>
#include <gui/test_press_screen/TEST_PRESSPresenter.hpp>

class TEST_PRESSView : public TEST_PRESSViewBase
{
public:
    TEST_PRESSView();
    virtual ~TEST_PRESSView() {}
    virtual void setupScreen();
    virtual void tearDownScreen();
    virtual void handleTickEvent();
protected:
};

#endif // TEST_PRESSVIEW_HPP
