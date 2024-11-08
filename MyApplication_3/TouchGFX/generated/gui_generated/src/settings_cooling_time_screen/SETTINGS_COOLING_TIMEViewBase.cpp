/*********************************************************************************/
/********** THIS FILE IS GENERATED BY TOUCHGFX DESIGNER, DO NOT MODIFY ***********/
/*********************************************************************************/
#include <gui_generated/settings_cooling_time_screen/SETTINGS_COOLING_TIMEViewBase.hpp>
#include <touchgfx/Color.hpp>
#include <images/BitmapDatabase.hpp>
#include <texts/TextKeysAndLanguages.hpp>

SETTINGS_COOLING_TIMEViewBase::SETTINGS_COOLING_TIMEViewBase() :
    flexButtonCallback(this, &SETTINGS_COOLING_TIMEViewBase::flexButtonCallbackHandler)
{
    __background.setPosition(0, 0, 800, 480);
    __background.setColor(touchgfx::Color::getColorFromRGB(0, 0, 0));
    add(__background);

    image1_1.setBitmap(touchgfx::Bitmap(BITMAP_DARK_THEME_IMAGES_BACKGROUNDS_1024X600_RAYS_ID));
    add(image1_1);

    image2.setXY(731, 16);
    image2.setBitmap(touchgfx::Bitmap(BITMAP_ICON_THEME_IMAGES_ACTION_HOME_FILLED_50_50_BDB5B5_SVG_ID));
    add(image2);

    HOME_BANNER_BG.setPosition(0, 83, 800, 37);
    HOME_BANNER_BG.setColor(touchgfx::Color::getColorFromRGB(0, 21, 255));
    HOME_BANNER_BG.setBorderColor(touchgfx::Color::getColorFromRGB(133, 133, 133));
    HOME_BANNER_BG.setBorderSize(4);
    HOME_BANNER_BG.setAlpha(116);
    add(HOME_BANNER_BG);

    HOME_BANNER.setPosition(9, 89, 781, 31);
    HOME_BANNER.setColor(touchgfx::Color::getColorFromRGB(255, 255, 255));
    HOME_BANNER.setLinespacing(0);
    Unicode::snprintf(HOME_BANNERBuffer, HOME_BANNER_SIZE, "%s", touchgfx::TypedText(T___SINGLEUSE_0098).getText());
    HOME_BANNER.setWildcard(HOME_BANNERBuffer);
    HOME_BANNER.setTypedText(touchgfx::TypedText(T___SINGLEUSE_4689));
    add(HOME_BANNER);

    textArea1_1.setXY(260, 18);
    textArea1_1.setColor(touchgfx::Color::getColorFromRGB(255, 255, 255));
    textArea1_1.setLinespacing(0);
    textArea1_1.setTypedText(touchgfx::TypedText(T___SINGLEUSE_LEB8));
    add(textArea1_1);

    box1_1.setWidth(800);
    box1_1.setHeight(83);
    box1_1.setColor(touchgfx::Color::getColorFromRGB(255, 255, 255));
    box1_1.setAlpha(25);
    add(box1_1);

    HOME_SETTINGS_COOLING.setBoxWithBorderPosition(0, 0, 86, 83);
    HOME_SETTINGS_COOLING.setBorderSize(5);
    HOME_SETTINGS_COOLING.setBoxWithBorderColors(touchgfx::Color::getColorFromRGB(0, 102, 153), touchgfx::Color::getColorFromRGB(0, 153, 204), touchgfx::Color::getColorFromRGB(0, 51, 102), touchgfx::Color::getColorFromRGB(51, 102, 153));
    HOME_SETTINGS_COOLING.setAlpha(0);
    HOME_SETTINGS_COOLING.setAction(flexButtonCallback);
    HOME_SETTINGS_COOLING.setPosition(714, 0, 86, 83);
    add(HOME_SETTINGS_COOLING);

    textArea1.setXY(90, 142);
    textArea1.setColor(touchgfx::Color::getColorFromRGB(255, 255, 255));
    textArea1.setLinespacing(0);
    textArea1.setTypedText(touchgfx::TypedText(T___SINGLEUSE_UQ6X));
    add(textArea1);

    SETTINGS_COOLING_TIME.setPosition(107, 300, 214, 37);
    SETTINGS_COOLING_TIME.setColor(touchgfx::Color::getColorFromRGB(0, 83, 207));
    SETTINGS_COOLING_TIME.setLinespacing(0);
    Unicode::snprintf(SETTINGS_COOLING_TIMEBuffer, SETTINGS_COOLING_TIME_SIZE, "%s", touchgfx::TypedText(T___SINGLEUSE_O4WA).getText());
    SETTINGS_COOLING_TIME.setWildcard(SETTINGS_COOLING_TIMEBuffer);
    SETTINGS_COOLING_TIME.setTypedText(touchgfx::TypedText(T___SINGLEUSE_6JGK));
    add(SETTINGS_COOLING_TIME);

    SETTINGS_COOLING_TIME_UP.setXY(78, 179);
    SETTINGS_COOLING_TIME_UP.setBitmaps(touchgfx::Bitmap(BITMAP_GLASS_THEME_IMAGES_WIDGETS_BUTTON_REGULAR_HEIGHT_60_SMALL_ROUNDED_NORMAL_ID), touchgfx::Bitmap(BITMAP_GLASS_THEME_IMAGES_WIDGETS_BUTTON_REGULAR_HEIGHT_60_SMALL_ROUNDED_PRESSED_ID), touchgfx::Bitmap(BITMAP_ICON_THEME_IMAGES_HARDWARE_KEYBOARD_ARROW_UP_80_80_E8F6FB_SVG_ID), touchgfx::Bitmap(BITMAP_ICON_THEME_IMAGES_HARDWARE_KEYBOARD_ARROW_UP_80_80_E8F6FB_SVG_ID));
    SETTINGS_COOLING_TIME_UP.setIconXY(101, 14);
    add(SETTINGS_COOLING_TIME_UP);

    SETTINGS_COOLING_TIME_DOWN.setXY(78, 356);
    SETTINGS_COOLING_TIME_DOWN.setBitmaps(touchgfx::Bitmap(BITMAP_GLASS_THEME_IMAGES_WIDGETS_BUTTON_REGULAR_HEIGHT_60_SMALL_ROUNDED_NORMAL_ID), touchgfx::Bitmap(BITMAP_GLASS_THEME_IMAGES_WIDGETS_BUTTON_REGULAR_HEIGHT_60_SMALL_ROUNDED_PRESSED_ID), touchgfx::Bitmap(BITMAP_ICON_THEME_IMAGES_HARDWARE_KEYBOARD_ARROW_DOWN_80_80_E8F6FB_SVG_ID), touchgfx::Bitmap(BITMAP_ICON_THEME_IMAGES_HARDWARE_KEYBOARD_ARROW_DOWN_80_80_E8F6FB_SVG_ID));
    SETTINGS_COOLING_TIME_DOWN.setIconXY(101, 14);
    add(SETTINGS_COOLING_TIME_DOWN);

    image3.setXY(422, 120);
    image3.setBitmap(touchgfx::Bitmap(BITMAP_A62_1_ID));
    add(image3);

    COOL_SETTINGS_BACK.setBoxWithBorderPosition(0, 0, 86, 83);
    COOL_SETTINGS_BACK.setBorderSize(5);
    COOL_SETTINGS_BACK.setBoxWithBorderColors(touchgfx::Color::getColorFromRGB(0, 102, 153), touchgfx::Color::getColorFromRGB(0, 153, 204), touchgfx::Color::getColorFromRGB(0, 51, 102), touchgfx::Color::getColorFromRGB(51, 102, 153));
    COOL_SETTINGS_BACK.setAlpha(0);
    COOL_SETTINGS_BACK.setAction(flexButtonCallback);
    COOL_SETTINGS_BACK.setPosition(0, 0, 86, 83);
    add(COOL_SETTINGS_BACK);

    image4_1_1.setXY(17, 16);
    image4_1_1.setBitmap(touchgfx::Bitmap(BITMAP_ICON_THEME_IMAGES_NAVIGATION_ARROW_BACK_IOS_50_50_BDB5B5_SVG_ID));
    add(image4_1_1);
}

SETTINGS_COOLING_TIMEViewBase::~SETTINGS_COOLING_TIMEViewBase()
{

}

void SETTINGS_COOLING_TIMEViewBase::setupScreen()
{

}

void SETTINGS_COOLING_TIMEViewBase::flexButtonCallbackHandler(const touchgfx::AbstractButtonContainer& src)
{
    if (&src == &HOME_SETTINGS_COOLING)
    {
        //Interaction1
        //When HOME_SETTINGS_COOLING clicked change screen to HOME
        //Go to HOME with no screen transition
        application().gotoHOMEScreenNoTransition();
    }
    if (&src == &COOL_SETTINGS_BACK)
    {
        //Interaction2
        //When COOL_SETTINGS_BACK clicked change screen to SETTINGS_MENU
        //Go to SETTINGS_MENU with screen transition towards East
        application().gotoSETTINGS_MENUScreenSlideTransitionEast();
    }
}
