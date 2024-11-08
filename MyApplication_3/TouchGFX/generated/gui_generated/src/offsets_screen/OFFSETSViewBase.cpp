/*********************************************************************************/
/********** THIS FILE IS GENERATED BY TOUCHGFX DESIGNER, DO NOT MODIFY ***********/
/*********************************************************************************/
#include <gui_generated/offsets_screen/OFFSETSViewBase.hpp>
#include <touchgfx/Color.hpp>
#include <images/BitmapDatabase.hpp>
#include <texts/TextKeysAndLanguages.hpp>

OFFSETSViewBase::OFFSETSViewBase() :
    flexButtonCallback(this, &OFFSETSViewBase::flexButtonCallbackHandler)
{
    __background.setPosition(0, 0, 800, 480);
    __background.setColor(touchgfx::Color::getColorFromRGB(0, 0, 0));
    add(__background);

    image1.setBitmap(touchgfx::Bitmap(BITMAP_DARK_THEME_IMAGES_BACKGROUNDS_1024X600_RAYS_ID));
    add(image1);

    box1.setWidth(800);
    box1.setHeight(83);
    box1.setColor(touchgfx::Color::getColorFromRGB(255, 255, 255));
    box1.setAlpha(25);
    add(box1);

    button1_1.setXY(18, 16);
    button1_1.setBitmaps(touchgfx::Bitmap(BITMAP_ALTERNATE_THEME_IMAGES_WIDGETS_BUTTON_ICON_ROUNDED_TINY_FILL_NORMAL_ID), touchgfx::Bitmap(BITMAP_ALTERNATE_THEME_IMAGES_WIDGETS_BUTTON_ICON_ROUNDED_TINY_FILL_PRESSED_ID));
    button1_1.setAlpha(0);
    add(button1_1);

    box1_1_1.setPosition(0, 2, 800, 478);
    box1_1_1.setColor(touchgfx::Color::getColorFromRGB(36, 35, 35));
    box1_1_1.setAlpha(100);
    add(box1_1_1);

    Tip_1_Energy_Down_3.setBoxWithBorderPosition(0, 0, 129, 57);
    Tip_1_Energy_Down_3.setBorderSize(5);
    Tip_1_Energy_Down_3.setBoxWithBorderColors(touchgfx::Color::getColorFromRGB(0, 102, 153), touchgfx::Color::getColorFromRGB(0, 153, 204), touchgfx::Color::getColorFromRGB(0, 51, 102), touchgfx::Color::getColorFromRGB(51, 102, 153));
    Tip_1_Energy_Down_3.setAlpha(0);
    Tip_1_Energy_Down_3.setPosition(392, 395, 129, 57);
    add(Tip_1_Energy_Down_3);

    Tip_3_Offset_Up.setBoxWithBorderPosition(0, 0, 120, 81);
    Tip_3_Offset_Up.setBorderSize(5);
    Tip_3_Offset_Up.setBoxWithBorderColors(touchgfx::Color::getColorFromRGB(0, 102, 153), touchgfx::Color::getColorFromRGB(0, 153, 204), touchgfx::Color::getColorFromRGB(0, 51, 102), touchgfx::Color::getColorFromRGB(51, 102, 153));
    Tip_3_Offset_Up.setAlpha(0);
    Tip_3_Offset_Up.setPosition(437, 177, 120, 81);
    add(Tip_3_Offset_Up);

    textArea2_1_1_3.setXY(455, 130);
    textArea2_1_1_3.setColor(touchgfx::Color::getColorFromRGB(255, 255, 255));
    textArea2_1_1_3.setLinespacing(0);
    textArea2_1_1_3.setTypedText(touchgfx::TypedText(T___SINGLEUSE_0QN6));
    add(textArea2_1_1_3);

    image3_4.setXY(472, 201);
    image3_4.setBitmap(touchgfx::Bitmap(BITMAP_ICON_THEME_IMAGES_NAVIGATION_EXPAND_LESS_50_50_C7C7C7_SVG_ID));
    add(image3_4);

    image3_1_3.setXY(475, 339);
    image3_1_3.setBitmap(touchgfx::Bitmap(BITMAP_ICON_THEME_IMAGES_NAVIGATION_EXPAND_MORE_50_50_C7C7C7_SVG_ID));
    add(image3_1_3);

    Tip_3_Offset.setPosition(451, 267, 162, 49);
    Tip_3_Offset.setColor(touchgfx::Color::getColorFromRGB(250, 162, 0));
    Tip_3_Offset.setLinespacing(0);
    Unicode::snprintf(Tip_3_OffsetBuffer, TIP_3_OFFSET_SIZE, "%s", touchgfx::TypedText(T_3).getText());
    Tip_3_Offset.setWildcard(Tip_3_OffsetBuffer);
    Tip_3_Offset.setTypedText(touchgfx::TypedText(T___SINGLEUSE_GDT1));
    add(Tip_3_Offset);

    Tip_2_Offset_Down.setBoxWithBorderPosition(0, 0, 129, 87);
    Tip_2_Offset_Down.setBorderSize(5);
    Tip_2_Offset_Down.setBoxWithBorderColors(touchgfx::Color::getColorFromRGB(0, 102, 153), touchgfx::Color::getColorFromRGB(0, 153, 204), touchgfx::Color::getColorFromRGB(0, 51, 102), touchgfx::Color::getColorFromRGB(51, 102, 153));
    Tip_2_Offset_Down.setAlpha(0);
    Tip_2_Offset_Down.setPosition(221, 336, 129, 87);
    add(Tip_2_Offset_Down);

    Tip_2_Offset_Up.setBoxWithBorderPosition(0, 0, 115, 81);
    Tip_2_Offset_Up.setBorderSize(5);
    Tip_2_Offset_Up.setBoxWithBorderColors(touchgfx::Color::getColorFromRGB(0, 102, 153), touchgfx::Color::getColorFromRGB(0, 153, 204), touchgfx::Color::getColorFromRGB(0, 51, 102), touchgfx::Color::getColorFromRGB(51, 102, 153));
    Tip_2_Offset_Up.setAlpha(0);
    Tip_2_Offset_Up.setPosition(225, 177, 115, 81);
    add(Tip_2_Offset_Up);

    textArea2_1_1_2.setXY(245, 130);
    textArea2_1_1_2.setColor(touchgfx::Color::getColorFromRGB(255, 255, 255));
    textArea2_1_1_2.setLinespacing(0);
    textArea2_1_1_2.setTypedText(touchgfx::TypedText(T___SINGLEUSE_88J2));
    add(textArea2_1_1_2);

    image3_3.setXY(260, 201);
    image3_3.setBitmap(touchgfx::Bitmap(BITMAP_ICON_THEME_IMAGES_NAVIGATION_EXPAND_LESS_50_50_C7C7C7_SVG_ID));
    add(image3_3);

    image3_1_2.setXY(260, 339);
    image3_1_2.setBitmap(touchgfx::Bitmap(BITMAP_ICON_THEME_IMAGES_NAVIGATION_EXPAND_MORE_50_50_C7C7C7_SVG_ID));
    add(image3_1_2);

    Tip_2_Offset.setPosition(237, 267, 175, 49);
    Tip_2_Offset.setColor(touchgfx::Color::getColorFromRGB(250, 162, 0));
    Tip_2_Offset.setLinespacing(0);
    Unicode::snprintf(Tip_2_OffsetBuffer, TIP_2_OFFSET_SIZE, "%s", touchgfx::TypedText(T_3).getText());
    Tip_2_Offset.setWildcard(Tip_2_OffsetBuffer);
    Tip_2_Offset.setTypedText(touchgfx::TypedText(T___SINGLEUSE_I4UK));
    add(Tip_2_Offset);

    Tip_1_Energy_Down_1.setBoxWithBorderPosition(0, 0, 129, 76);
    Tip_1_Energy_Down_1.setBorderSize(5);
    Tip_1_Energy_Down_1.setBoxWithBorderColors(touchgfx::Color::getColorFromRGB(0, 102, 153), touchgfx::Color::getColorFromRGB(0, 153, 204), touchgfx::Color::getColorFromRGB(0, 51, 102), touchgfx::Color::getColorFromRGB(51, 102, 153));
    Tip_1_Energy_Down_1.setAlpha(0);
    Tip_1_Energy_Down_1.setPosition(633, 339, 129, 76);
    add(Tip_1_Energy_Down_1);

    Tip_4_Offset_Up.setBoxWithBorderPosition(0, 0, 120, 81);
    Tip_4_Offset_Up.setBorderSize(5);
    Tip_4_Offset_Up.setBoxWithBorderColors(touchgfx::Color::getColorFromRGB(0, 102, 153), touchgfx::Color::getColorFromRGB(0, 153, 204), touchgfx::Color::getColorFromRGB(0, 51, 102), touchgfx::Color::getColorFromRGB(51, 102, 153));
    Tip_4_Offset_Up.setAlpha(0);
    Tip_4_Offset_Up.setPosition(633, 177, 120, 81);
    add(Tip_4_Offset_Up);

    textArea2_1_1_1.setXY(653, 130);
    textArea2_1_1_1.setColor(touchgfx::Color::getColorFromRGB(255, 255, 255));
    textArea2_1_1_1.setLinespacing(0);
    textArea2_1_1_1.setTypedText(touchgfx::TypedText(T___SINGLEUSE_HH3O));
    add(textArea2_1_1_1);

    image3_2.setXY(668, 201);
    image3_2.setBitmap(touchgfx::Bitmap(BITMAP_ICON_THEME_IMAGES_NAVIGATION_EXPAND_LESS_50_50_C7C7C7_SVG_ID));
    add(image3_2);

    image3_1_1.setXY(668, 339);
    image3_1_1.setBitmap(touchgfx::Bitmap(BITMAP_ICON_THEME_IMAGES_NAVIGATION_EXPAND_MORE_50_50_C7C7C7_SVG_ID));
    add(image3_1_1);

    Tip_4_Offset.setPosition(639, 267, 147, 49);
    Tip_4_Offset.setColor(touchgfx::Color::getColorFromRGB(250, 162, 0));
    Tip_4_Offset.setLinespacing(0);
    Unicode::snprintf(Tip_4_OffsetBuffer, TIP_4_OFFSET_SIZE, "%s", touchgfx::TypedText(T_3).getText());
    Tip_4_Offset.setWildcard(Tip_4_OffsetBuffer);
    Tip_4_Offset.setTypedText(touchgfx::TypedText(T___SINGLEUSE_BYXO));
    add(Tip_4_Offset);

    Tip_1_Energy_Up_1_3.setBoxWithBorderPosition(0, 0, 120, 50);
    Tip_1_Energy_Up_1_3.setBorderSize(5);
    Tip_1_Energy_Up_1_3.setBoxWithBorderColors(touchgfx::Color::getColorFromRGB(0, 102, 153), touchgfx::Color::getColorFromRGB(0, 153, 204), touchgfx::Color::getColorFromRGB(0, 51, 102), touchgfx::Color::getColorFromRGB(51, 102, 153));
    Tip_1_Energy_Up_1_3.setAlpha(0);
    Tip_1_Energy_Up_1_3.setPosition(412, 160, 120, 50);
    add(Tip_1_Energy_Up_1_3);

    Tip_3_Offset_Down.setBoxWithBorderPosition(0, 0, 129, 79);
    Tip_3_Offset_Down.setBorderSize(5);
    Tip_3_Offset_Down.setBoxWithBorderColors(touchgfx::Color::getColorFromRGB(0, 102, 153), touchgfx::Color::getColorFromRGB(0, 153, 204), touchgfx::Color::getColorFromRGB(0, 51, 102), touchgfx::Color::getColorFromRGB(51, 102, 153));
    Tip_3_Offset_Down.setAlpha(0);
    Tip_3_Offset_Down.setPosition(436, 336, 129, 79);
    add(Tip_3_Offset_Down);

    Tip_4_Offset_Down.setBoxWithBorderPosition(0, 0, 129, 57);
    Tip_4_Offset_Down.setBorderSize(5);
    Tip_4_Offset_Down.setBoxWithBorderColors(touchgfx::Color::getColorFromRGB(0, 102, 153), touchgfx::Color::getColorFromRGB(0, 153, 204), touchgfx::Color::getColorFromRGB(0, 51, 102), touchgfx::Color::getColorFromRGB(51, 102, 153));
    Tip_4_Offset_Down.setAlpha(0);
    Tip_4_Offset_Down.setPosition(609, 326, 129, 57);
    add(Tip_4_Offset_Down);

    Tip_1_Offset.setPosition(33, 267, 167, 49);
    Tip_1_Offset.setColor(touchgfx::Color::getColorFromRGB(250, 162, 0));
    Tip_1_Offset.setLinespacing(0);
    Unicode::snprintf(Tip_1_OffsetBuffer, TIP_1_OFFSET_SIZE, "%s", touchgfx::TypedText(T_3).getText());
    Tip_1_Offset.setWildcard(Tip_1_OffsetBuffer);
    Tip_1_Offset.setTypedText(touchgfx::TypedText(T___SINGLEUSE_74WZ));
    add(Tip_1_Offset);

    image3_1.setXY(59, 339);
    image3_1.setBitmap(touchgfx::Bitmap(BITMAP_ICON_THEME_IMAGES_NAVIGATION_EXPAND_MORE_50_50_C7C7C7_SVG_ID));
    add(image3_1);

    image3.setXY(59, 201);
    image3.setBitmap(touchgfx::Bitmap(BITMAP_ICON_THEME_IMAGES_NAVIGATION_EXPAND_LESS_50_50_C7C7C7_SVG_ID));
    add(image3);

    textArea2_1_2.setXY(325, 22);
    textArea2_1_2.setColor(touchgfx::Color::getColorFromRGB(255, 255, 255));
    textArea2_1_2.setLinespacing(0);
    textArea2_1_2.setTypedText(touchgfx::TypedText(T___SINGLEUSE_HTVU));
    add(textArea2_1_2);

    textArea2_1_1.setXY(44, 130);
    textArea2_1_1.setColor(touchgfx::Color::getColorFromRGB(255, 255, 255));
    textArea2_1_1.setLinespacing(0);
    textArea2_1_1.setTypedText(touchgfx::TypedText(T___SINGLEUSE_XSV1));
    add(textArea2_1_1);

    Tip_1_Offset_Up.setBoxWithBorderPosition(0, 0, 125, 74);
    Tip_1_Offset_Up.setBorderSize(5);
    Tip_1_Offset_Up.setBoxWithBorderColors(touchgfx::Color::getColorFromRGB(0, 102, 153), touchgfx::Color::getColorFromRGB(0, 153, 204), touchgfx::Color::getColorFromRGB(0, 51, 102), touchgfx::Color::getColorFromRGB(51, 102, 153));
    Tip_1_Offset_Up.setAlpha(0);
    Tip_1_Offset_Up.setPosition(22, 189, 125, 74);
    add(Tip_1_Offset_Up);

    Tip_1_Offset_Down.setBoxWithBorderPosition(0, 0, 129, 93);
    Tip_1_Offset_Down.setBorderSize(5);
    Tip_1_Offset_Down.setBoxWithBorderColors(touchgfx::Color::getColorFromRGB(0, 102, 153), touchgfx::Color::getColorFromRGB(0, 153, 204), touchgfx::Color::getColorFromRGB(0, 51, 102), touchgfx::Color::getColorFromRGB(51, 102, 153));
    Tip_1_Offset_Down.setAlpha(0);
    Tip_1_Offset_Down.setPosition(22, 318, 129, 93);
    add(Tip_1_Offset_Down);

    PASSWORD_MODAL.setShadeColor(touchgfx::Color::getColorFromRGB(0, 0, 0));
    PASSWORD_MODAL.hide();
    add(PASSWORD_MODAL);

    boxWithBorder1.setPosition(169, 89, 458, 62);
    boxWithBorder1.setColor(touchgfx::Color::getColorFromRGB(255, 255, 255));
    boxWithBorder1.setBorderColor(touchgfx::Color::getColorFromRGB(92, 92, 92));
    boxWithBorder1.setBorderSize(5);
    boxWithBorder1.setAlpha(172);
    boxWithBorder1.setVisible(false);
    add(boxWithBorder1);

    MODAL_NUM_1.setXY(167, 172);
    MODAL_NUM_1.setBitmaps(touchgfx::Bitmap(BITMAP_DARK_THEME_IMAGES_WIDGETS_BUTTON_REGULAR_HEIGHT_60_TINY_ROUNDED_NORMAL_ID), touchgfx::Bitmap(BITMAP_DARK_THEME_IMAGES_WIDGETS_BUTTON_REGULAR_HEIGHT_60_TINY_ROUNDED_PRESSED_ID));
    MODAL_NUM_1.setLabelText(touchgfx::TypedText(T___SINGLEUSE_Z7BL));
    MODAL_NUM_1.setLabelColor(touchgfx::Color::getColorFromRGB(255, 255, 255));
    MODAL_NUM_1.setLabelColorPressed(touchgfx::Color::getColorFromRGB(255, 255, 255));
    MODAL_NUM_1.setVisible(false);
    add(MODAL_NUM_1);

    MODAL_NUM_2.setXY(328, 172);
    MODAL_NUM_2.setBitmaps(touchgfx::Bitmap(BITMAP_DARK_THEME_IMAGES_WIDGETS_BUTTON_REGULAR_HEIGHT_60_TINY_ROUNDED_NORMAL_ID), touchgfx::Bitmap(BITMAP_DARK_THEME_IMAGES_WIDGETS_BUTTON_REGULAR_HEIGHT_60_TINY_ROUNDED_PRESSED_ID));
    MODAL_NUM_2.setLabelText(touchgfx::TypedText(T___SINGLEUSE_JF2D));
    MODAL_NUM_2.setLabelColor(touchgfx::Color::getColorFromRGB(255, 255, 255));
    MODAL_NUM_2.setLabelColorPressed(touchgfx::Color::getColorFromRGB(255, 255, 255));
    MODAL_NUM_2.setVisible(false);
    add(MODAL_NUM_2);

    MODAL_NUM_3.setXY(491, 172);
    MODAL_NUM_3.setBitmaps(touchgfx::Bitmap(BITMAP_DARK_THEME_IMAGES_WIDGETS_BUTTON_REGULAR_HEIGHT_60_TINY_ROUNDED_NORMAL_ID), touchgfx::Bitmap(BITMAP_DARK_THEME_IMAGES_WIDGETS_BUTTON_REGULAR_HEIGHT_60_TINY_ROUNDED_PRESSED_ID));
    MODAL_NUM_3.setLabelText(touchgfx::TypedText(T___SINGLEUSE_6NCO));
    MODAL_NUM_3.setLabelColor(touchgfx::Color::getColorFromRGB(255, 255, 255));
    MODAL_NUM_3.setLabelColorPressed(touchgfx::Color::getColorFromRGB(255, 255, 255));
    MODAL_NUM_3.setVisible(false);
    add(MODAL_NUM_3);

    MODAL_NUM_9.setXY(492, 333);
    MODAL_NUM_9.setBitmaps(touchgfx::Bitmap(BITMAP_DARK_THEME_IMAGES_WIDGETS_BUTTON_REGULAR_HEIGHT_60_TINY_ROUNDED_NORMAL_ID), touchgfx::Bitmap(BITMAP_DARK_THEME_IMAGES_WIDGETS_BUTTON_REGULAR_HEIGHT_60_TINY_ROUNDED_PRESSED_ID));
    MODAL_NUM_9.setLabelText(touchgfx::TypedText(T___SINGLEUSE_1UQG));
    MODAL_NUM_9.setLabelColor(touchgfx::Color::getColorFromRGB(255, 255, 255));
    MODAL_NUM_9.setLabelColorPressed(touchgfx::Color::getColorFromRGB(255, 255, 255));
    MODAL_NUM_9.setVisible(false);
    add(MODAL_NUM_9);

    MODAL_NUM_6.setXY(491, 248);
    MODAL_NUM_6.setBitmaps(touchgfx::Bitmap(BITMAP_DARK_THEME_IMAGES_WIDGETS_BUTTON_REGULAR_HEIGHT_60_TINY_ROUNDED_NORMAL_ID), touchgfx::Bitmap(BITMAP_DARK_THEME_IMAGES_WIDGETS_BUTTON_REGULAR_HEIGHT_60_TINY_ROUNDED_PRESSED_ID));
    MODAL_NUM_6.setLabelText(touchgfx::TypedText(T___SINGLEUSE_5BN0));
    MODAL_NUM_6.setLabelColor(touchgfx::Color::getColorFromRGB(255, 255, 255));
    MODAL_NUM_6.setLabelColorPressed(touchgfx::Color::getColorFromRGB(255, 255, 255));
    MODAL_NUM_6.setVisible(false);
    add(MODAL_NUM_6);

    MODAL_NUM_5.setXY(330, 248);
    MODAL_NUM_5.setBitmaps(touchgfx::Bitmap(BITMAP_DARK_THEME_IMAGES_WIDGETS_BUTTON_REGULAR_HEIGHT_60_TINY_ROUNDED_NORMAL_ID), touchgfx::Bitmap(BITMAP_DARK_THEME_IMAGES_WIDGETS_BUTTON_REGULAR_HEIGHT_60_TINY_ROUNDED_PRESSED_ID));
    MODAL_NUM_5.setLabelText(touchgfx::TypedText(T___SINGLEUSE_95BH));
    MODAL_NUM_5.setLabelColor(touchgfx::Color::getColorFromRGB(255, 255, 255));
    MODAL_NUM_5.setLabelColorPressed(touchgfx::Color::getColorFromRGB(255, 255, 255));
    MODAL_NUM_5.setVisible(false);
    add(MODAL_NUM_5);

    MODAL_NUM_4.setXY(168, 248);
    MODAL_NUM_4.setBitmaps(touchgfx::Bitmap(BITMAP_DARK_THEME_IMAGES_WIDGETS_BUTTON_REGULAR_HEIGHT_60_TINY_ROUNDED_NORMAL_ID), touchgfx::Bitmap(BITMAP_DARK_THEME_IMAGES_WIDGETS_BUTTON_REGULAR_HEIGHT_60_TINY_ROUNDED_PRESSED_ID));
    MODAL_NUM_4.setLabelText(touchgfx::TypedText(T___SINGLEUSE_5MB9));
    MODAL_NUM_4.setLabelColor(touchgfx::Color::getColorFromRGB(255, 255, 255));
    MODAL_NUM_4.setLabelColorPressed(touchgfx::Color::getColorFromRGB(255, 255, 255));
    MODAL_NUM_4.setVisible(false);
    add(MODAL_NUM_4);

    MODAL_NUM_8.setXY(331, 333);
    MODAL_NUM_8.setBitmaps(touchgfx::Bitmap(BITMAP_DARK_THEME_IMAGES_WIDGETS_BUTTON_REGULAR_HEIGHT_60_TINY_ROUNDED_NORMAL_ID), touchgfx::Bitmap(BITMAP_DARK_THEME_IMAGES_WIDGETS_BUTTON_REGULAR_HEIGHT_60_TINY_ROUNDED_PRESSED_ID));
    MODAL_NUM_8.setLabelText(touchgfx::TypedText(T___SINGLEUSE_HA26));
    MODAL_NUM_8.setLabelColor(touchgfx::Color::getColorFromRGB(255, 255, 255));
    MODAL_NUM_8.setLabelColorPressed(touchgfx::Color::getColorFromRGB(255, 255, 255));
    MODAL_NUM_8.setVisible(false);
    add(MODAL_NUM_8);

    MODAL_NUM_7.setXY(169, 333);
    MODAL_NUM_7.setBitmaps(touchgfx::Bitmap(BITMAP_DARK_THEME_IMAGES_WIDGETS_BUTTON_REGULAR_HEIGHT_60_TINY_ROUNDED_NORMAL_ID), touchgfx::Bitmap(BITMAP_DARK_THEME_IMAGES_WIDGETS_BUTTON_REGULAR_HEIGHT_60_TINY_ROUNDED_PRESSED_ID));
    MODAL_NUM_7.setLabelText(touchgfx::TypedText(T___SINGLEUSE_4T7H));
    MODAL_NUM_7.setLabelColor(touchgfx::Color::getColorFromRGB(255, 255, 255));
    MODAL_NUM_7.setLabelColorPressed(touchgfx::Color::getColorFromRGB(255, 255, 255));
    MODAL_NUM_7.setVisible(false);
    add(MODAL_NUM_7);

    MODAL_PASSWORD_TEXT.setPosition(192, 101, 441, 50);
    MODAL_PASSWORD_TEXT.setColor(touchgfx::Color::getColorFromRGB(0, 0, 0));
    MODAL_PASSWORD_TEXT.setLinespacing(0);
    Unicode::snprintf(MODAL_PASSWORD_TEXTBuffer, MODAL_PASSWORD_TEXT_SIZE, "%s", touchgfx::TypedText(T___SINGLEUSE_PMJI).getText());
    MODAL_PASSWORD_TEXT.setWildcard(MODAL_PASSWORD_TEXTBuffer);
    MODAL_PASSWORD_TEXT.setTypedText(touchgfx::TypedText(T___SINGLEUSE_WR55));
    MODAL_PASSWORD_TEXT.setAlpha(190);
    add(MODAL_PASSWORD_TEXT);

    image2.setXY(720, 15);
    image2.setBitmap(touchgfx::Bitmap(BITMAP_ICON_THEME_IMAGES_ACTION_HOME_FILLED_50_50_BDB5B5_SVG_ID));
    add(image2);

    flexButton1.setBoxWithBorderPosition(0, 0, 81, 74);
    flexButton1.setBorderSize(5);
    flexButton1.setBoxWithBorderColors(touchgfx::Color::getColorFromRGB(0, 102, 153), touchgfx::Color::getColorFromRGB(0, 153, 204), touchgfx::Color::getColorFromRGB(0, 51, 102), touchgfx::Color::getColorFromRGB(51, 102, 153));
    flexButton1.setAlpha(0);
    flexButton1.setAction(flexButtonCallback);
    flexButton1.setPosition(705, 15, 81, 74);
    add(flexButton1);
}

OFFSETSViewBase::~OFFSETSViewBase()
{

}

void OFFSETSViewBase::setupScreen()
{

}

void OFFSETSViewBase::flexButtonCallbackHandler(const touchgfx::AbstractButtonContainer& src)
{
    if (&src == &flexButton1)
    {
        //Interaction1
        //When flexButton1 clicked change screen to HOME
        //Go to HOME with no screen transition
        application().gotoHOMEScreenNoTransition();
    }
}
