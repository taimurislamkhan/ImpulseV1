#include <gui/settings_global_screen/SETTINGS_GLOBALView.hpp>
#include <touchgfx/Color.hpp>
#include <cstring>

char SCREEN_BANNER_TEXT4[] = "SETTINGS";
extern uint8_t FLAG_E_STOP_ACTIVE;

extern char HMI_HOME_BANNER_TEXT[50];
Unicode::UnicodeChar HOME_BANNER_TEXT5[50];
extern uint8_t HMI_BANNER_COLOR;
extern uint8_t  Home_Machine;
extern uint8_t FLAG_REQUIRE_HOME;
extern double HMI_TEST_PRESS_DOWN_POSITION;

SETTINGS_GLOBALView::SETTINGS_GLOBALView()
{

}

void SETTINGS_GLOBALView::setupScreen()
{
    SETTINGS_GLOBALViewBase::setupScreen();
}

void SETTINGS_GLOBALView::tearDownScreen()
{
    SETTINGS_GLOBALViewBase::tearDownScreen();
}

extern uint8_t HMI_ENCODER_BYPASS;
extern double HMI_BOSS_HEIGHT_TOLERANCE_VAL;
extern uint8_t HMI_PREHEAT_ENERGY_VAL;
extern uint8_t HMI_PLATEN_SPEED_CURVE_VAL;
extern uint8_t HMI_PREHEAT_ENABLE;
extern uint8_t HMI_PREHEAT_DISTANCE_VAL;

void SETTINGS_GLOBALView::encoder_bypass_clicked()
{
	HMI_ENCODER_BYPASS = ENCODER_BYPASS_TOGGLE.getState();
}

void SETTINGS_GLOBALView::updateHomeBannerColor()
{
        if (HMI_BANNER_COLOR==1)
        {
            HOME_BANNER_BG.setColor(touchgfx::Color::getColorFromRGB(230, 5, 5)); // red
        }
        else if (HMI_BANNER_COLOR==2)
        {
        	HOME_BANNER_BG.setColor(touchgfx::Color::getColorFromRGB(255, 217, 0)); // yellow
        }
        else if (HMI_BANNER_COLOR==0)
        {
        	HOME_BANNER_BG.setColor(touchgfx::Color::getColorFromRGB(0, 235, 5)); // green
        }
        else if (HMI_BANNER_COLOR==3)
        {
        	HOME_BANNER_BG.setColor(touchgfx::Color::getColorFromRGB(87, 82, 82)); // green
        }

        HOME_BANNER_BG.invalidate();
}

void SETTINGS_GLOBALView::PREHEAT_ENABLE_PRESSED()
{
	HMI_PREHEAT_ENABLE = PREHEAT_TOGGLE.getState();
}
//
//void SETTINGS_GLOBALView::BOSS_HEIGHT_SLIDER_CHANGED()
//{
//	HMI_BOSS_HEIGHT_TOLERANCE_VAL = BOSS_HEIGHT_SLIDER.getValue();
//}
//
//void SETTINGS_GLOBALView::PLATEN_SPEED_CURVE_CHANGED()
//{
//	HMI_PLATEN_SPEED_CURVE_VAL = PLATEN_SPEED_CURVE_SLIDER.getValue();
//}
//
//void SETTINGS_GLOBALView::PREHEAT_SLIDER_CHANGED()
//{
//	HMI_PREHEAT_VAL = PREHEAT_SLIDER.getValue();
//}

void SETTINGS_GLOBALView::afterTransition()
{
	ENCODER_BYPASS_TOGGLE.forceState(HMI_ENCODER_BYPASS);
	ENCODER_BYPASS_TOGGLE.invalidate();

	BOSS_HEIGHT_SLIDER.setValue((int)HMI_BOSS_HEIGHT_TOLERANCE_VAL);
	BOSS_HEIGHT_SLIDER.invalidate();

	PLATEN_SPEED_CURVE_SLIDER.setValue((int)HMI_PLATEN_SPEED_CURVE_VAL);
	PLATEN_SPEED_CURVE_SLIDER.invalidate();

	PREHEAT_TOGGLE.forceState(HMI_PREHEAT_ENABLE);
	PREHEAT_TOGGLE.invalidate();

	PREHEAT_ENERGY_SLIDER.setValue(HMI_PREHEAT_ENERGY_VAL);
	PREHEAT_ENERGY_SLIDER.invalidate();

	PREHEAT_DISTANCE_SLIDER.setValue(HMI_PREHEAT_DISTANCE_VAL);
	PREHEAT_DISTANCE_SLIDER.invalidate();

}

void SETTINGS_GLOBALView::handleTickEvent()
{

	HMI_BOSS_HEIGHT_TOLERANCE_VAL = (double)BOSS_HEIGHT_SLIDER.getValue();
	Unicode::snprintfFloat(BOSS_HEIGHT_SLIDER_VALBuffer, BOSS_HEIGHT_SLIDER_VAL_SIZE, "%.2f",(double)(HMI_BOSS_HEIGHT_TOLERANCE_VAL/100));
	BOSS_HEIGHT_SLIDER_VAL.invalidate();

	HMI_PLATEN_SPEED_CURVE_VAL = PLATEN_SPEED_CURVE_SLIDER.getValue();
	Unicode::snprintf(SPEED_CURVE_SLIDER_VALBuffer, SPEED_CURVE_SLIDER_VAL_SIZE, "%d", HMI_PLATEN_SPEED_CURVE_VAL);
	SPEED_CURVE_SLIDER_VAL.invalidate();

	HMI_PREHEAT_ENERGY_VAL = PREHEAT_ENERGY_SLIDER.getValue();
	Unicode::snprintf(PREHEAT_ENERGY_SLIDER_VALBuffer, PREHEAT_ENERGY_SLIDER_VAL_SIZE, "%d", HMI_PREHEAT_ENERGY_VAL);
	PREHEAT_ENERGY_SLIDER_VAL.invalidate();

	HMI_PREHEAT_DISTANCE_VAL = PREHEAT_DISTANCE_SLIDER.getValue();
	Unicode::snprintf(PREHEAT_DISTANCE_SLIDER_VALBuffer, PREHEAT_DISTANCE_SLIDER_VAL_SIZE, "%d", HMI_PREHEAT_DISTANCE_VAL);
	PREHEAT_DISTANCE_SLIDER_VAL.invalidate();

//	updateHomeBannerColor();
//	Unicode::strncpy(HOME_BANNER_TEXT5, HMI_HOME_BANNER_TEXT, 50);
//	Unicode::snprintf(HOME_BANNERBuffer, HOME_BANNER_SIZE, "%s",HOME_BANNER_TEXT5);
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
		Unicode::strncpy(HOME_BANNER_TEXT5, HMI_HOME_BANNER_TEXT, 50);
		Unicode::snprintf(HOME_BANNERBuffer, HOME_BANNER_SIZE, "%s",HOME_BANNER_TEXT5);
		HOME_BANNER.invalidate();

	}

	else
	{
		HOME_BANNER_BG.setColor(touchgfx::Color::getColorFromRGB(0, 21, 255)); // blue
		HOME_BANNER_BG.invalidate();
		Unicode::strncpy(HOME_BANNER_TEXT5, SCREEN_BANNER_TEXT4, strlen(SCREEN_BANNER_TEXT4)+1);
		Unicode::snprintf(HOME_BANNERBuffer, HOME_BANNER_SIZE, "%s",HOME_BANNER_TEXT5);
		HOME_BANNER.invalidate();

	}


}
