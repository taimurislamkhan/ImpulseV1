/*********************************************************************************/
/********** THIS FILE IS GENERATED BY TOUCHGFX DESIGNER, DO NOT MODIFY ***********/
/*********************************************************************************/
#ifndef SETTINGS_GLOBALVIEWBASE_HPP
#define SETTINGS_GLOBALVIEWBASE_HPP

#include <gui/common/FrontendApplication.hpp>
#include <mvp/View.hpp>
#include <gui/settings_global_screen/SETTINGS_GLOBALPresenter.hpp>
#include <touchgfx/widgets/Box.hpp>
#include <touchgfx/widgets/Image.hpp>
#include <touchgfx/widgets/BoxWithBorder.hpp>
#include <touchgfx/widgets/TextAreaWithWildcard.hpp>
#include <touchgfx/widgets/TextArea.hpp>
#include <touchgfx/containers/buttons/Buttons.hpp>
#include <touchgfx/widgets/ToggleButton.hpp>
#include <touchgfx/containers/Slider.hpp>

class SETTINGS_GLOBALViewBase : public touchgfx::View<SETTINGS_GLOBALPresenter>
{
public:
    SETTINGS_GLOBALViewBase();
    virtual ~SETTINGS_GLOBALViewBase();
    virtual void setupScreen();

    /*
     * Custom Actions
     */
    virtual void action1()
    {
        // Override and implement this function in Screen1
    }
    

    /*
     * Virtual Action Handlers
     */
    virtual void encoder_bypass_clicked()
    {
        // Override and implement this function in SETTINGS_GLOBAL
    }
    virtual void BOSS_HEIGHT_SLIDER_CHANGED(int value)
    {
        // Override and implement this function in SETTINGS_GLOBAL
    }
    virtual void PREHEAT_SLIDER_CHANGED(int value)
    {
        // Override and implement this function in SETTINGS_GLOBAL
    }
    virtual void PLATEN_SPEED_CURVE_CHANGED(int value)
    {
        // Override and implement this function in SETTINGS_GLOBAL
    }
    virtual void PREHEAT_ENABLE_PRESSED()
    {
        // Override and implement this function in SETTINGS_GLOBAL
    }

protected:
    FrontendApplication& application() {
        return *static_cast<FrontendApplication*>(touchgfx::Application::getInstance());
    }

    /*
     * Member Declarations
     */
    touchgfx::Box __background;
    touchgfx::Image image1_1;
    touchgfx::Image image2;
    touchgfx::BoxWithBorder HOME_BANNER_BG;
    touchgfx::TextAreaWithOneWildcard HOME_BANNER;
    touchgfx::TextArea textArea1_1;
    touchgfx::Box box1_1;
    touchgfx::BoxWithBorderButtonStyle< touchgfx::ClickButtonTrigger >  HOME_SETTINGS;
    touchgfx::TextArea textArea1_2;
    touchgfx::TextArea textArea1;
    touchgfx::ToggleButton ENCODER_BYPASS_TOGGLE;
    touchgfx::TextArea textArea1_2_2_1;
    touchgfx::TextArea textArea1_2_2;
    touchgfx::TextAreaWithOneWildcard PREHEAT_DISTANCE_SLIDER_VAL;
    touchgfx::TextArea textArea1_2_2_2_1;
    touchgfx::Slider PREHEAT_DISTANCE_SLIDER;
    touchgfx::Slider PLATEN_SPEED_CURVE_SLIDER;
    touchgfx::Slider PREHEAT_ENERGY_SLIDER;
    touchgfx::Slider BOSS_HEIGHT_SLIDER;
    touchgfx::TextAreaWithOneWildcard SPEED_CURVE_SLIDER_VAL;
    touchgfx::TextArea textArea1_2_2_2;
    touchgfx::TextAreaWithOneWildcard PREHEAT_ENERGY_SLIDER_VAL;
    touchgfx::TextAreaWithOneWildcard BOSS_HEIGHT_SLIDER_VAL;
    touchgfx::ToggleButton PREHEAT_TOGGLE;

    /*
     * Wildcard Buffers
     */
    static const uint16_t HOME_BANNER_SIZE = 50;
    touchgfx::Unicode::UnicodeChar HOME_BANNERBuffer[HOME_BANNER_SIZE];
    static const uint16_t PREHEAT_DISTANCE_SLIDER_VAL_SIZE = 10;
    touchgfx::Unicode::UnicodeChar PREHEAT_DISTANCE_SLIDER_VALBuffer[PREHEAT_DISTANCE_SLIDER_VAL_SIZE];
    static const uint16_t SPEED_CURVE_SLIDER_VAL_SIZE = 10;
    touchgfx::Unicode::UnicodeChar SPEED_CURVE_SLIDER_VALBuffer[SPEED_CURVE_SLIDER_VAL_SIZE];
    static const uint16_t PREHEAT_ENERGY_SLIDER_VAL_SIZE = 10;
    touchgfx::Unicode::UnicodeChar PREHEAT_ENERGY_SLIDER_VALBuffer[PREHEAT_ENERGY_SLIDER_VAL_SIZE];
    static const uint16_t BOSS_HEIGHT_SLIDER_VAL_SIZE = 10;
    touchgfx::Unicode::UnicodeChar BOSS_HEIGHT_SLIDER_VALBuffer[BOSS_HEIGHT_SLIDER_VAL_SIZE];

private:

    /*
     * Callback Declarations
     */
    touchgfx::Callback<SETTINGS_GLOBALViewBase, const touchgfx::AbstractButtonContainer&> flexButtonCallback;
    touchgfx::Callback<SETTINGS_GLOBALViewBase, const touchgfx::AbstractButton&> buttonCallback;
    touchgfx::Callback<SETTINGS_GLOBALViewBase, const touchgfx::Slider&, int> sliderValueChangedCallback;

    /*
     * Callback Handler Declarations
     */
    void flexButtonCallbackHandler(const touchgfx::AbstractButtonContainer& src);
    void buttonCallbackHandler(const touchgfx::AbstractButton& src);
    void sliderValueChangedCallbackHandler(const touchgfx::Slider& src, int value);

};

#endif // SETTINGS_GLOBALVIEWBASE_HPP