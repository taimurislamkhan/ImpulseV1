/*********************************************************************************/
/********** THIS FILE IS GENERATED BY TOUCHGFX DESIGNER, DO NOT MODIFY ***********/
/*********************************************************************************/
#ifndef TEST_PRESSVIEWBASE_HPP
#define TEST_PRESSVIEWBASE_HPP

#include <gui/common/FrontendApplication.hpp>
#include <mvp/View.hpp>
#include <gui/test_press_screen/TEST_PRESSPresenter.hpp>
#include <touchgfx/widgets/Box.hpp>
#include <touchgfx/widgets/Image.hpp>
#include <touchgfx/widgets/Button.hpp>
#include <touchgfx/widgets/canvas/Line.hpp>
#include <touchgfx/widgets/canvas/PainterRGB565.hpp>
#include <touchgfx/widgets/ButtonWithIcon.hpp>
#include <touchgfx/widgets/TextArea.hpp>
#include <touchgfx/widgets/TextAreaWithWildcard.hpp>
#include <touchgfx/widgets/BoxWithBorder.hpp>
#include <touchgfx/containers/buttons/Buttons.hpp>
#include <touchgfx/widgets/ButtonWithLabel.hpp>

class TEST_PRESSViewBase : public touchgfx::View<TEST_PRESSPresenter>
{
public:
    TEST_PRESSViewBase();
    virtual ~TEST_PRESSViewBase();
    virtual void setupScreen();

protected:
    FrontendApplication& application() {
        return *static_cast<FrontendApplication*>(touchgfx::Application::getInstance());
    }

    /*
     * Member Declarations
     */
    touchgfx::Box __background;
    touchgfx::Image image1;
    touchgfx::Image image2_1;
    touchgfx::Button button1_1_2;
    touchgfx::Line line1_1;
    touchgfx::PainterRGB565 line1_1Painter;
    touchgfx::ButtonWithIcon TEST_PRESS_PLATEN_DOWN;
    touchgfx::ButtonWithIcon TEST_PRESS_PLATEN_UP;
    touchgfx::ButtonWithIcon TEST_PRESS_SAVE_UP;
    touchgfx::ButtonWithIcon TEST_PRESS_SAVE_DOWN;
    touchgfx::TextArea textArea1_3_1;
    touchgfx::TextArea textArea1_4;
    touchgfx::TextAreaWithOneWildcard TEST_PRESS_PLATEN_DISTANCE;
    touchgfx::TextArea textArea1;
    touchgfx::Box box3;
    touchgfx::Box box1;
    touchgfx::BoxWithBorder boxWithBorder1;
    touchgfx::TextArea textArea1_2;
    touchgfx::Button button1_1;
    touchgfx::TextArea textArea1_1;
    touchgfx::BoxWithBorderButtonStyle< touchgfx::ClickButtonTrigger >  flexButton1;
    touchgfx::TextAreaWithOneWildcard TEST_PRESS_UP_POSITION;
    touchgfx::TextArea textArea2_2;
    touchgfx::TextArea textArea2_2_1;
    touchgfx::TextArea textArea2_2_1_1;
    touchgfx::TextAreaWithOneWildcard TEST_PRESS_DOWN_POSITION;
    touchgfx::TextArea textArea2_1_1;
    touchgfx::ButtonWithLabel TEST_PRESS_START_CYCLE;

    /*
     * Wildcard Buffers
     */
    static const uint16_t TEST_PRESS_PLATEN_DISTANCE_SIZE = 10;
    touchgfx::Unicode::UnicodeChar TEST_PRESS_PLATEN_DISTANCEBuffer[TEST_PRESS_PLATEN_DISTANCE_SIZE];
    static const uint16_t TEST_PRESS_UP_POSITION_SIZE = 10;
    touchgfx::Unicode::UnicodeChar TEST_PRESS_UP_POSITIONBuffer[TEST_PRESS_UP_POSITION_SIZE];
    static const uint16_t TEST_PRESS_DOWN_POSITION_SIZE = 10;
    touchgfx::Unicode::UnicodeChar TEST_PRESS_DOWN_POSITIONBuffer[TEST_PRESS_DOWN_POSITION_SIZE];

private:

    /*
     * Canvas Buffer Size
     */
    static const uint32_t CANVAS_BUFFER_SIZE = 12000;
    uint8_t canvasBuffer[CANVAS_BUFFER_SIZE];

    /*
     * Callback Declarations
     */
    touchgfx::Callback<TEST_PRESSViewBase, const touchgfx::AbstractButtonContainer&> flexButtonCallback;

    /*
     * Callback Handler Declarations
     */
    void flexButtonCallbackHandler(const touchgfx::AbstractButtonContainer& src);

};

#endif // TEST_PRESSVIEWBASE_HPP