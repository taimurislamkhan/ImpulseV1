/*********************************************************************************/
/********** THIS FILE IS GENERATED BY TOUCHGFX DESIGNER, DO NOT MODIFY ***********/
/*********************************************************************************/
#ifndef ALARM_HISTORYVIEWBASE_HPP
#define ALARM_HISTORYVIEWBASE_HPP

#include <gui/common/FrontendApplication.hpp>
#include <mvp/View.hpp>
#include <gui/alarm_history_screen/ALARM_HISTORYPresenter.hpp>
#include <touchgfx/widgets/Box.hpp>
#include <touchgfx/containers/buttons/Buttons.hpp>
#include <touchgfx/widgets/Image.hpp>
#include <touchgfx/widgets/TextArea.hpp>
#include <touchgfx/widgets/BoxWithBorder.hpp>
#include <touchgfx/widgets/Button.hpp>
#include <touchgfx/containers/ScrollableContainer.hpp>
#include <touchgfx/widgets/canvas/Line.hpp>
#include <touchgfx/widgets/canvas/PainterRGB565.hpp>
#include <touchgfx/containers/clock/DigitalClock.hpp>

class ALARM_HISTORYViewBase : public touchgfx::View<ALARM_HISTORYPresenter>
{
public:
    ALARM_HISTORYViewBase();
    virtual ~ALARM_HISTORYViewBase();
    virtual void setupScreen();

protected:
    FrontendApplication& application() {
        return *static_cast<FrontendApplication*>(touchgfx::Application::getInstance());
    }

    /*
     * Member Declarations
     */
    touchgfx::Box __background;
    touchgfx::BoxWithBorderButtonStyle< touchgfx::ClickButtonTrigger >  flexButton1;
    touchgfx::Image image1;
    touchgfx::Box box3;
    touchgfx::TextArea textArea1_1;
    touchgfx::BoxWithBorder boxWithBorder1;
    touchgfx::Button button1_1;
    touchgfx::TextArea textArea1_2;
    touchgfx::Box box1;
    touchgfx::TextArea textArea1_4;
    touchgfx::Image image2;
    touchgfx::TextArea textArea1_2_1;
    touchgfx::TextArea textArea1_2_1_1;
    touchgfx::TextArea textArea1_2_1_1_2;
    touchgfx::ScrollableContainer scrollableContainer1;
    touchgfx::Line line1_2_1_1;
    touchgfx::PainterRGB565 line1_2_1_1Painter;
    touchgfx::Line line1_2_1;
    touchgfx::PainterRGB565 line1_2_1Painter;
    touchgfx::Line line1_3;
    touchgfx::PainterRGB565 line1_3Painter;
    touchgfx::Line line1_2;
    touchgfx::PainterRGB565 line1_2Painter;
    touchgfx::Line line1_1;
    touchgfx::PainterRGB565 line1_1Painter;
    touchgfx::Line line1;
    touchgfx::PainterRGB565 line1Painter;
    touchgfx::DigitalClock digitalClock1;
    touchgfx::DigitalClock digitalClock1_1;
    touchgfx::TextArea textArea1_2_1_1_1_1_1;
    touchgfx::TextArea textArea1_2_1_1_1_2;
    touchgfx::DigitalClock digitalClock1_2;
    touchgfx::TextArea textArea1_2_1_1_1_1_2;
    touchgfx::TextArea textArea1_2_1_1_1_3;
    touchgfx::TextArea textArea1_2_1_1_1_3_1;
    touchgfx::TextArea textArea1_2_1_1_1_1_2_1;
    touchgfx::DigitalClock digitalClock1_2_1;
    touchgfx::DigitalClock digitalClock1_2_1_1;
    touchgfx::TextArea textArea1_2_1_1_1_1_2_1_1;
    touchgfx::TextArea textArea1_2_1_1_1_3_1_1;
    touchgfx::TextArea textArea1_2_1_1_1_3_1_1_1;
    touchgfx::TextArea textArea1_2_1_1_1_1_2_1_1_1;
    touchgfx::DigitalClock digitalClock1_2_1_1_1;
    touchgfx::DigitalClock digitalClock1_2_1_2;
    touchgfx::TextArea textArea1_2_1_1_1_1_2_1_2;
    touchgfx::TextArea textArea1_2_1_1_1_3_1_2;
    touchgfx::TextArea textArea1_2_1_1_1_3_2;
    touchgfx::TextArea textArea1_2_1_1_1_1_2_2;
    touchgfx::DigitalClock digitalClock1_2_2;
    touchgfx::TextArea textArea1_2_1_1_1_2_1;
    touchgfx::TextArea textArea1_2_1_1_1_1_1_1;
    touchgfx::DigitalClock digitalClock1_1_1;
    touchgfx::Line line1_1_1;
    touchgfx::PainterRGB565 line1_1_1Painter;
    touchgfx::Line line1_2_2;
    touchgfx::PainterRGB565 line1_2_2Painter;
    touchgfx::Line line1_2_1_2;
    touchgfx::PainterRGB565 line1_2_1_2Painter;
    touchgfx::Line line1_2_1_1_1;
    touchgfx::PainterRGB565 line1_2_1_1_1Painter;
    touchgfx::TextArea textArea1_2_1_1_1;
    touchgfx::DigitalClock digitalClock1_3;
    touchgfx::TextArea textArea1_2_1_1_1_1_3;
    touchgfx::TextArea textArea1_2_1_1_1_4;

private:

    /*
     * Canvas Buffer Size
     */
    static const uint32_t CANVAS_BUFFER_SIZE = 12000;
    uint8_t canvasBuffer[CANVAS_BUFFER_SIZE];

    /*
     * Callback Declarations
     */
    touchgfx::Callback<ALARM_HISTORYViewBase, const touchgfx::AbstractButtonContainer&> flexButtonCallback;

    /*
     * Callback Handler Declarations
     */
    void flexButtonCallbackHandler(const touchgfx::AbstractButtonContainer& src);

};

#endif // ALARM_HISTORYVIEWBASE_HPP