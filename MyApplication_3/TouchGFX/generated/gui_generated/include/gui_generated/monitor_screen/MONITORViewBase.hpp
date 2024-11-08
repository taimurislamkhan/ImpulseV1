/*********************************************************************************/
/********** THIS FILE IS GENERATED BY TOUCHGFX DESIGNER, DO NOT MODIFY ***********/
/*********************************************************************************/
#ifndef MONITORVIEWBASE_HPP
#define MONITORVIEWBASE_HPP

#include <gui/common/FrontendApplication.hpp>
#include <mvp/View.hpp>
#include <gui/monitor_screen/MONITORPresenter.hpp>
#include <touchgfx/widgets/Box.hpp>
#include <touchgfx/containers/buttons/Buttons.hpp>
#include <touchgfx/widgets/Image.hpp>
#include <touchgfx/widgets/TextArea.hpp>
#include <touchgfx/widgets/ButtonWithLabel.hpp>
#include <touchgfx/widgets/BoxWithBorder.hpp>
#include <touchgfx/widgets/TextAreaWithWildcard.hpp>
#include <touchgfx/widgets/ToggleButton.hpp>

class MONITORViewBase : public touchgfx::View<MONITORPresenter>
{
public:
    MONITORViewBase();
    virtual ~MONITORViewBase();
    virtual void setupScreen();

    /*
     * Virtual Action Handlers
     */
    virtual void Brass_Mode_Pressed()
    {
        // Override and implement this function in MONITOR
    }

protected:
    FrontendApplication& application() {
        return *static_cast<FrontendApplication*>(touchgfx::Application::getInstance());
    }

    /*
     * Member Declarations
     */
    touchgfx::Box __background;
    touchgfx::BoxWithBorderButtonStyle< touchgfx::ClickButtonTrigger >  MONITOR_HOME_BUTTON;
    touchgfx::Image image1;
    touchgfx::Box box3;
    touchgfx::TextArea textArea1_2_1_2_1;
    touchgfx::ButtonWithLabel buttonWithLabel1_1;
    touchgfx::TextArea textArea1_1;
    touchgfx::BoxWithBorder HOME_BANNER_BG;
    touchgfx::TextAreaWithOneWildcard HOME_BANNER;
    touchgfx::TextArea textArea1_2_1;
    touchgfx::TextArea textArea1_2_1_1;
    touchgfx::Box box1;
    touchgfx::TextArea textArea1_4;
    touchgfx::Image image2;
    touchgfx::Image LEFT_START_RED;
    touchgfx::Image OVER_TRAVEL_RED;
    touchgfx::TextArea textArea1_2_1_1_1;
    touchgfx::Image E_STOP_CLEAR_RED;
    touchgfx::Image E_STOP_CLEAR_GREEN;
    touchgfx::Image RIGHT_START_RED;
    touchgfx::Image RIGHT_START_GREEN;
    touchgfx::Image LEFT_START_GREEN;
    touchgfx::Image OVER_TRAVEL_GREEN;
    touchgfx::TextArea textArea1_2_1_2;
    touchgfx::ToggleButton BRASS_MODE_TOGGLE;

    /*
     * Wildcard Buffers
     */
    static const uint16_t HOME_BANNER_SIZE = 50;
    touchgfx::Unicode::UnicodeChar HOME_BANNERBuffer[HOME_BANNER_SIZE];

private:

    /*
     * Callback Declarations
     */
    touchgfx::Callback<MONITORViewBase, const touchgfx::AbstractButtonContainer&> flexButtonCallback;
    touchgfx::Callback<MONITORViewBase, const touchgfx::AbstractButton&> buttonCallback;

    /*
     * Callback Handler Declarations
     */
    void flexButtonCallbackHandler(const touchgfx::AbstractButtonContainer& src);
    void buttonCallbackHandler(const touchgfx::AbstractButton& src);

};

#endif // MONITORVIEWBASE_HPP
