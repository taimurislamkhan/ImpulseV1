#include <gui/manual_screen/MANUALView.hpp>
#include <touchgfx/Color.hpp>
char SCREEN_BANNER_TEXT[] = "MANUAL CONTROLS";
extern char HMI_HOME_BANNER_TEXT[50];
Unicode::UnicodeChar HOME_BANNER_TEXT6[50];
extern uint8_t HMI_BANNER_COLOR;
extern uint8_t FLAG_E_STOP_ACTIVE;

extern uint8_t  Home_Machine;
extern uint8_t FLAG_REQUIRE_HOME;
extern double HMI_TEST_PRESS_DOWN_POSITION;

MANUALView::MANUALView()
{

}

void MANUALView::setupScreen()
{
    MANUALViewBase::setupScreen();
}

void MANUALView::tearDownScreen()
{
    MANUALViewBase::tearDownScreen();
}

extern uint8_t HMI_MANUAL_PLATEN_UP;
extern uint8_t HMI_MANUAL_PLATEN_DOWN;
extern uint8_t HMI_MANUAL_COOLING;
extern uint8_t HMI_MANUAL_TIP_1;
extern uint8_t HMI_MANUAL_TIP_2;
extern uint8_t HMI_MANUAL_TIP_3;
extern uint8_t HMI_MANUAL_TIP_4;
extern uint8_t HMI_MANUAL_TIP_5;
extern uint8_t HMI_MANUAL_TIP_6;
extern uint8_t HMI_MANUAL_TIP_7;
extern uint8_t HMI_MANUAL_TIP_8;
extern double HMI_MANUAL_PLATEN_DISTANCE;

void MANUALView::afterTransition()
{

}

//void MANUALView::updateHomeBannerColor()
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
void MANUALView::handleTickEvent()
{
	HMI_MANUAL_PLATEN_UP	= MANUAL_PLATEN_UP.getPressedState();
	HMI_MANUAL_PLATEN_DOWN 	= MANUAL_PLATEN_DOWN.getPressedState();
	HMI_MANUAL_COOLING		= MANUAL_COOLING.getPressedState();
	HMI_MANUAL_TIP_1       	= MANUAL_TIP_1.getPressedState();
	HMI_MANUAL_TIP_2       	= MANUAL_TIP_2.getPressedState();
	HMI_MANUAL_TIP_3       	= MANUAL_TIP_3.getPressedState();
	HMI_MANUAL_TIP_4       	= MANUAL_TIP_4.getPressedState();
	HMI_MANUAL_TIP_5       	= MANUAL_TIP_5.getPressedState();
	HMI_MANUAL_TIP_6       	= MANUAL_TIP_6.getPressedState();
	HMI_MANUAL_TIP_7       	= MANUAL_TIP_7.getPressedState();
	HMI_MANUAL_TIP_8       	= MANUAL_TIP_8.getPressedState();

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

	// Print Platen Distance
	Unicode::snprintfFloat(MANUAL_PLATEN_DISTANCEBuffer, MANUAL_PLATEN_DISTANCE_SIZE, "%.2f", HMI_MANUAL_PLATEN_DISTANCE/100);
	MANUAL_PLATEN_DISTANCE.invalidate();

//	updateHomeBannerColor();

	if (FLAG_E_STOP_ACTIVE)
	{
		HOME_BANNER_BG.setColor(touchgfx::Color::getColorFromRGB(230, 5, 5)); // red
		HOME_BANNER_BG.invalidate();
		Unicode::strncpy(HOME_BANNER_TEXT6, HMI_HOME_BANNER_TEXT, 50);
		Unicode::snprintf(HOME_BANNERBuffer, HOME_BANNER_SIZE, "%s",HOME_BANNER_TEXT6);
		HOME_BANNER.invalidate();

	}

	else
	{
		HOME_BANNER_BG.setColor(touchgfx::Color::getColorFromRGB(0, 21, 255)); // blue
		HOME_BANNER_BG.invalidate();
		Unicode::strncpy(HOME_BANNER_TEXT6, SCREEN_BANNER_TEXT, strlen(SCREEN_BANNER_TEXT)+1);
		Unicode::snprintf(HOME_BANNERBuffer, HOME_BANNER_SIZE, "%s",HOME_BANNER_TEXT6);
		HOME_BANNER.invalidate();

	}

}
