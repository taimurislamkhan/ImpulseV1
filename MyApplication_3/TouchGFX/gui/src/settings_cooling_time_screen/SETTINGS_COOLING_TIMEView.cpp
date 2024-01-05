#include <gui/settings_cooling_time_screen/SETTINGS_COOLING_TIMEView.hpp>
#include <touchgfx/Color.hpp>
#include <cstring>

char SCREEN_BANNER_TEXT3[] = "SETTINGS";
extern uint8_t FLAG_E_STOP_ACTIVE;
extern char HMI_HOME_BANNER_TEXT[50];
Unicode::UnicodeChar HOME_BANNER_TEXT8[50];
extern uint8_t HMI_BANNER_COLOR;
extern uint8_t  Home_Machine;
extern uint8_t FLAG_REQUIRE_HOME;
extern double HMI_TEST_PRESS_DOWN_POSITION;

SETTINGS_COOLING_TIMEView::SETTINGS_COOLING_TIMEView()
{

}

void SETTINGS_COOLING_TIMEView::setupScreen()
{
    SETTINGS_COOLING_TIMEViewBase::setupScreen();
}

void SETTINGS_COOLING_TIMEView::tearDownScreen()
{
    SETTINGS_COOLING_TIMEViewBase::tearDownScreen();
}

extern uint8_t HMI_COOLING_TIME_UP;
extern uint8_t HMI_COOLING_TIME_DOWN;
extern double HMI_COOLING_TIME_SET;

//void SETTINGS_COOLING_TIMEView::updateHomeBannerColor()
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

void SETTINGS_COOLING_TIMEView::handleTickEvent()
{
	HMI_COOLING_TIME_UP = SETTINGS_COOLING_TIME_UP.getPressedState();
	HMI_COOLING_TIME_DOWN = SETTINGS_COOLING_TIME_DOWN.getPressedState();
	Unicode::snprintfFloat(SETTINGS_COOLING_TIMEBuffer, SETTINGS_COOLING_TIME_SIZE, "%.2f",HMI_COOLING_TIME_SET/10 );
	SETTINGS_COOLING_TIME.invalidate();

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
//
//	updateHomeBannerColor();
//	Unicode::strncpy(HOME_BANNER_TEXT8, HMI_HOME_BANNER_TEXT, 50);
//	Unicode::snprintf(HOME_BANNERBuffer, HOME_BANNER_SIZE, "%s",HOME_BANNER_TEXT8);
//	HOME_BANNER.invalidate();

	if (FLAG_E_STOP_ACTIVE)
	{
		HOME_BANNER_BG.setColor(touchgfx::Color::getColorFromRGB(230, 5, 5)); // red
		HOME_BANNER_BG.invalidate();
		Unicode::strncpy(HOME_BANNER_TEXT8, HMI_HOME_BANNER_TEXT, 50);
		Unicode::snprintf(HOME_BANNERBuffer, HOME_BANNER_SIZE, "%s",HOME_BANNER_TEXT8);
		HOME_BANNER.invalidate();

	}

	else
	{
		HOME_BANNER_BG.setColor(touchgfx::Color::getColorFromRGB(0, 21, 255)); // blue
		HOME_BANNER_BG.invalidate();
		Unicode::strncpy(HOME_BANNER_TEXT8, SCREEN_BANNER_TEXT3, strlen(SCREEN_BANNER_TEXT3)+1);
		Unicode::snprintf(HOME_BANNERBuffer, HOME_BANNER_SIZE, "%s",HOME_BANNER_TEXT8);
		HOME_BANNER.invalidate();

	}
}
