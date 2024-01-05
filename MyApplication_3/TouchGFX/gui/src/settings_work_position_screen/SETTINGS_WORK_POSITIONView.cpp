#include <gui/settings_work_position_screen/SETTINGS_WORK_POSITIONView.hpp>
#include <touchgfx/Color.hpp>
#include <cstring>

char SCREEN_BANNER_TEXT6[] = "SETTINGS";
extern uint8_t FLAG_E_STOP_ACTIVE;
extern char HMI_HOME_BANNER_TEXT[50];
Unicode::UnicodeChar HOME_BANNER_TEXT4[50];
extern uint8_t HMI_BANNER_COLOR;
extern uint8_t  Home_Machine;
extern uint8_t FLAG_REQUIRE_HOME;
extern double HMI_TEST_PRESS_DOWN_POSITION;


SETTINGS_WORK_POSITIONView::SETTINGS_WORK_POSITIONView()
{

}

void SETTINGS_WORK_POSITIONView::setupScreen()
{
    SETTINGS_WORK_POSITIONViewBase::setupScreen();
}

void SETTINGS_WORK_POSITIONView::tearDownScreen()
{
    SETTINGS_WORK_POSITIONViewBase::tearDownScreen();
}

extern uint16_t HMI_SETTINGS_PLATEN_UP;
extern uint8_t HMI_SETTINGS_PLATEN_DOWN;
extern uint8_t HMI_SETTINGS_WORK_POSITION;
extern uint8_t HMI_AUTOFIND;

//void SETTINGS_WORK_POSITIONView::updateHomeBannerColor()
//{
//        if (HMI_BANNER_COLOR==1)
//        {
//            HOME_BANNER_BG.setColor(touchgfx::Color::getColorFromRGB(230, 5, 5)); // red
//        }
//        else if (HMI_BANNER_COLOR==2)
//        {
//        	HOME_BANNER_BG.setColor(touchgfx::Color::getColorFromRGB(255, 217, 0)); // yellow
//        }
//        else if (HMI_BANNER_COLOR==0)
//        {
//        	HOME_BANNER_BG.setColor(touchgfx::Color::getColorFromRGB(0, 235, 5)); // green
//        }
//        else if (HMI_BANNER_COLOR==3)
//        {
//        	HOME_BANNER_BG.setColor(touchgfx::Color::getColorFromRGB(87, 82, 82)); // green
//        }
//
//        HOME_BANNER_BG.invalidate();
//}

void SETTINGS_WORK_POSITIONView::handleTickEvent()
{
	HMI_SETTINGS_PLATEN_UP = SETTINGS_PLATEN_UP.getPressedState();
	HMI_SETTINGS_PLATEN_DOWN = SETTINGS_PLATEN_DOWN.getPressedState();
	HMI_SETTINGS_WORK_POSITION = SETTINGS_SET_WORK_POSITION.getPressedState();
	HMI_AUTOFIND = HMI_SETTINGS_AUTOFIND.getPressed();

//	updateHomeBannerColor();
//	Unicode::strncpy(HOME_BANNER_TEXT4, HMI_HOME_BANNER_TEXT, 50);
//	Unicode::snprintf(HOME_BANNERBuffer, HOME_BANNER_SIZE, "%s",HOME_BANNER_TEXT4);
//	HOME_BANNER.invalidate();
//
//	if (FLAG_REQUIRE_HOME)
//	{
//		Home_Machine = HOME_MACHINE.getPressed();
//	}
//	if (HMI_TEST_PRESS_DOWN_POSITION < 10 && FLAG_REQUIRE_HOME ==0 )
//	{
//		SET_WORK_HEIGHT.setTouchable(1);
//	}
//	else
//	{
//		SET_WORK_HEIGHT.setTouchable(0);
//	}

	if (FLAG_E_STOP_ACTIVE)
	{
		HOME_BANNER_BG.setColor(touchgfx::Color::getColorFromRGB(230, 5, 5)); // red
		HOME_BANNER_BG.invalidate();
		Unicode::strncpy(HOME_BANNER_TEXT4, HMI_HOME_BANNER_TEXT, 50);
		Unicode::snprintf(HOME_BANNERBuffer, HOME_BANNER_SIZE, "%s",HOME_BANNER_TEXT4);
		HOME_BANNER.invalidate();

	}

	else
	{
		HOME_BANNER_BG.setColor(touchgfx::Color::getColorFromRGB(0, 21, 255)); // blue
		HOME_BANNER_BG.invalidate();
		Unicode::strncpy(HOME_BANNER_TEXT4, SCREEN_BANNER_TEXT6, strlen(SCREEN_BANNER_TEXT6)+1);
		Unicode::snprintf(HOME_BANNERBuffer, HOME_BANNER_SIZE, "%s",HOME_BANNER_TEXT4);
		HOME_BANNER.invalidate();

	}
}
