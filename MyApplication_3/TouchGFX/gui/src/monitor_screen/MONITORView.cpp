#include <gui/monitor_screen/MONITORView.hpp>
#include <touchgfx/Color.hpp>
#include <cstring>
char SCREEN_BANNER_TEXT2[] = "MONITOR";
extern uint8_t FLAG_E_STOP_ACTIVE;
extern char HMI_HOME_BANNER_TEXT[50];
Unicode::UnicodeChar HOME_BANNER_TEXT7[50];
extern uint8_t HMI_BANNER_COLOR;
extern uint8_t  Home_Machine;
extern uint8_t FLAG_REQUIRE_HOME;
extern double HMI_TEST_PRESS_DOWN_POSITION;


MONITORView::MONITORView()
{

}

void MONITORView::setupScreen()
{
    MONITORViewBase::setupScreen();
}

void MONITORView::tearDownScreen()
{
    MONITORViewBase::tearDownScreen();
}

extern uint8_t HMI_MONITOR_LEFT_START;
extern uint8_t HMI_MONITOR_RIGHT_START;
extern uint8_t HMI_MONITOR_ESTOP_CLEAR;
extern uint8_t HMI_MONITOR_OVERTRAVEL;

//void MONITORView::updateHomeBannerColor()
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

extern uint8_t Brass_Mode_Enabled;
void MONITORView::afterTransition()
{
	BRASS_MODE_TOGGLE.forceState(Brass_Mode_Enabled);
	BRASS_MODE_TOGGLE.invalidate();
}

void MONITORView::Brass_Mode_Pressed()
{
	Brass_Mode_Enabled ^=1;
}

void MONITORView::handleTickEvent()
{
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

    // Left Start conditions
    bool leftStartRedVisible = !HMI_MONITOR_LEFT_START;
    bool leftStartGreenVisible = HMI_MONITOR_LEFT_START;

    if (LEFT_START_RED.isVisible() != leftStartRedVisible)
    {
        LEFT_START_RED.setVisible(leftStartRedVisible);
        LEFT_START_RED.invalidate();
    }

    if (LEFT_START_GREEN.isVisible() != leftStartGreenVisible)
    {
        LEFT_START_GREEN.setVisible(leftStartGreenVisible);
        LEFT_START_GREEN.invalidate();
    }

    // Right Start conditions
    bool rightStartRedVisible = !HMI_MONITOR_RIGHT_START;
    bool rightStartGreenVisible = HMI_MONITOR_RIGHT_START;

    if (RIGHT_START_RED.isVisible() != rightStartRedVisible)
    {
        RIGHT_START_RED.setVisible(rightStartRedVisible);
        RIGHT_START_RED.invalidate();
    }

    if (RIGHT_START_GREEN.isVisible() != rightStartGreenVisible)
    {
        RIGHT_START_GREEN.setVisible(rightStartGreenVisible);
        RIGHT_START_GREEN.invalidate();
    }

    // E-Stop Clear conditions
    bool eStopClearRedVisible = !HMI_MONITOR_ESTOP_CLEAR;
    bool eStopClearGreenVisible = HMI_MONITOR_ESTOP_CLEAR;

    if (E_STOP_CLEAR_RED.isVisible() != eStopClearRedVisible)
    {
        E_STOP_CLEAR_RED.setVisible(eStopClearRedVisible);
        E_STOP_CLEAR_RED.invalidate();
    }

    if (E_STOP_CLEAR_GREEN.isVisible() != eStopClearGreenVisible)
    {
        E_STOP_CLEAR_GREEN.setVisible(eStopClearGreenVisible);
        E_STOP_CLEAR_GREEN.invalidate();
    }

    // Over Travel conditions
    bool overTravelRedVisible = !HMI_MONITOR_OVERTRAVEL;
    bool overTravelGreenVisible = HMI_MONITOR_OVERTRAVEL;

    if (OVER_TRAVEL_RED.isVisible() != overTravelRedVisible)
    {
        OVER_TRAVEL_RED.setVisible(overTravelRedVisible);
        OVER_TRAVEL_RED.invalidate();
    }

    if (OVER_TRAVEL_GREEN.isVisible() != overTravelGreenVisible)
    {
        OVER_TRAVEL_GREEN.setVisible(overTravelGreenVisible);
        OVER_TRAVEL_GREEN.invalidate();
    }

//	updateHomeBannerColor();
//	Unicode::strncpy(HOME_BANNER_TEXT7, HMI_HOME_BANNER_TEXT, 50);
//	Unicode::snprintf(HOME_BANNERBuffer, HOME_BANNER_SIZE, "%s",HOME_BANNER_TEXT7);
//	HOME_BANNER.invalidate();

	if (FLAG_E_STOP_ACTIVE)
	{
		HOME_BANNER_BG.setColor(touchgfx::Color::getColorFromRGB(230, 5, 5)); // red
		HOME_BANNER_BG.invalidate();
		Unicode::strncpy(HOME_BANNER_TEXT7, HMI_HOME_BANNER_TEXT, 50);
		Unicode::snprintf(HOME_BANNERBuffer, HOME_BANNER_SIZE, "%s",HOME_BANNER_TEXT7);
		HOME_BANNER.invalidate();

	}

	else
	{
		HOME_BANNER_BG.setColor(touchgfx::Color::getColorFromRGB(0, 21, 255)); // blue
		HOME_BANNER_BG.invalidate();
		Unicode::strncpy(HOME_BANNER_TEXT7, SCREEN_BANNER_TEXT2, strlen(SCREEN_BANNER_TEXT2)+1);
		Unicode::snprintf(HOME_BANNERBuffer, HOME_BANNER_SIZE, "%s",HOME_BANNER_TEXT7);
		HOME_BANNER.invalidate();

	}
}


