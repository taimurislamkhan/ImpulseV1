#include <gui/home_screen/HOMEView.hpp>
#include <touchgfx/Color.hpp>

HOMEView::HOMEView()
{
	previousFlagEStopState =0; // Initialize the variable
}

void HOMEView::setupScreen()
{
    HOMEViewBase::setupScreen();
}

void HOMEView::tearDownScreen()
{
    HOMEViewBase::tearDownScreen();
}

// Common Vars for Banner
extern char HMI_HOME_BANNER_TEXT[50];
Unicode::UnicodeChar HOME_BANNER_TEXT[50];
extern uint8_t HMI_BANNER_COLOR;
extern uint8_t  Home_Machine;
extern uint8_t FLAG_REQUIRE_HOME;
extern double HMI_TEST_PRESS_DOWN_POSITION;


extern uint16_t HOME_Tip_1_Energy;
extern uint16_t HOME_Tip_2_Energy;
extern uint16_t HOME_Tip_3_Energy;
extern uint16_t HOME_Tip_4_Energy;
extern uint16_t HOME_Tip_5_Energy;
extern uint16_t HOME_Tip_6_Energy;
extern uint16_t HOME_Tip_7_Energy;
extern uint16_t HOME_Tip_8_Energy;

uint16_t Last_Tip_1_Energy=255;
uint16_t Last_Tip_2_Energy=255;
uint16_t Last_Tip_3_Energy=255;

double Last_Tip_1_Distance=255;
double Last_Tip_2_Distance=255;
double Last_Tip_3_Distance=255;

uint8_t Last_Fault_Active=255;
uint8_t Last_Home_Banner_Color=255;

extern double HMI_Tip_1_Distance;
extern double HMI_Tip_2_Distance;
extern double HMI_Tip_3_Distance;
extern double HMI_Tip_4_Distance;
extern double Tip_5_Distance;
extern double Tip_6_Distance;
extern double Tip_7_Distance;
extern double Tip_8_Distance;

extern uint8_t FLAG_GOING_HOME;
extern uint8_t FLAG_REACHED_HOME;
extern uint8_t FLAG_E_STOP_ACTIVE;
extern uint8_t ANTI_TIE_DOWN_ACTIVE;
extern uint8_t FLAG_IN_CYCLE;
extern uint8_t FAULT_ACTIVE_FLAG;
extern uint8_t MODAL_OK_BUTTON;

extern uint32_t SCREEN_REFRESH_RATE;
uint32_t counter=0;

extern double HMI_BOSS_HEIGHT;
extern double HMI_MANUAL_PLATEN_DISTANCE;

void HOMEView::updateHomeBannerColor()
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

void HOMEView::handleTickEvent()
{
//	if (counter >= (60/SCREEN_REFRESH_RATE))
//	{
//		counter=0;
		MODAL_OK_BUTTON = MODAL_OK.getPressedState();
				if (FLAG_REQUIRE_HOME)
				{
					Home_Machine = HOME_MACHINE.getPressed();
				}
				if (HMI_TEST_PRESS_DOWN_POSITION < 10 && FLAG_REQUIRE_HOME ==0 )
				{
					SET_WORK_HEIGHT.setTouchable(1);
				}
				else
				{
					SET_WORK_HEIGHT.setTouchable(0);
				}

//				if (Last_Fault_Active != FAULT_ACTIVE_FLAG )
//				{
//					Last_Fault_Active = FAULT_ACTIVE_FLAG;
//
//					if (FAULT_ACTIVE_FLAG)
//					{
//						MODAL_BG.show();
//						MODAL_ERROR_BG.setVisible(true);
//						MODAL_OK.setVisible(true);
//						MODAL_TEXT.setVisible(true);
//						Unicode::strncpy(HOME_BANNER_TEXT, HMI_HOME_BANNER_TEXT, strlen(HMI_HOME_BANNER_TEXT)+1);
//						Unicode::snprintf(MODAL_TEXTBuffer, MODAL_TEXT_SIZE, "%s",HOME_BANNER_TEXT);
//						MODAL_TEXT.invalidate();
//						MODAL_TITLE.setVisible(true);
//					}
//					else
//					{
						MODAL_BG.hide();
						MODAL_ERROR_BG.setVisible(false);
						MODAL_OK.setVisible(false);
						MODAL_TEXT.setVisible(false);
						MODAL_TITLE.setVisible(false);
					//}

				//}

					Last_Tip_1_Energy = HOME_Tip_1_Energy;
					Unicode::snprintf(Tip_1_EnergyBuffer, TIP_1_ENERGY_SIZE, "%d", HOME_Tip_1_Energy);
					Tip_1_Energy.invalidate();

					Last_Tip_2_Energy = HOME_Tip_2_Energy;
					Unicode::snprintf(Tip_2_EnergyBuffer, TIP_2_ENERGY_SIZE, "%d", HOME_Tip_2_Energy);
					Tip_2_Energy.invalidate();

					Unicode::snprintf(Tip_3_EnergyBuffer, TIP_3_ENERGY_SIZE, "%d", HOME_Tip_3_Energy);
					Tip_3_Energy.invalidate();


				Unicode::snprintf(Tip_4_EnergyBuffer, TIP_4_ENERGY_SIZE, "%d", HOME_Tip_4_Energy);
				Tip_4_Energy.invalidate();

				Unicode::snprintf(Tip_5_EnergyBuffer, TIP_5_ENERGY_SIZE, "%d", HOME_Tip_5_Energy);
				Tip_5_Energy.invalidate();

				Unicode::snprintf(Tip_6_EnergyBuffer, TIP_6_ENERGY_SIZE, "%d", HOME_Tip_6_Energy);
				Tip_6_Energy.invalidate();

				Unicode::snprintf(Tip_7_EnergyBuffer, TIP_7_ENERGY_SIZE, "%d", HOME_Tip_7_Energy);
				Tip_7_Energy.invalidate();

				Unicode::snprintf(Tip_8_EnergyBuffer, TIP_8_ENERGY_SIZE, "%d", HOME_Tip_8_Energy);
				Tip_8_Energy.invalidate();

				Unicode::snprintfFloat(HOME_DISTANCE_1Buffer, HOME_DISTANCE_1_SIZE, "%.1f", HMI_Tip_1_Distance);
				HOME_DISTANCE_1.invalidate();

				Unicode::snprintfFloat(HOME_DISTANCE_2Buffer, HOME_DISTANCE_2_SIZE, "%.1f", HMI_Tip_2_Distance);
				HOME_DISTANCE_2.invalidate();

				Unicode::snprintfFloat(HOME_DISTANCE_3Buffer, HOME_DISTANCE_3_SIZE, "%.1f", HMI_Tip_3_Distance);
				HOME_DISTANCE_3.invalidate();

				Unicode::snprintfFloat(HOME_DISTANCE_4Buffer, HOME_DISTANCE_4_SIZE, "%.1f", HMI_Tip_4_Distance);
				HOME_DISTANCE_4.invalidate();

				Unicode::snprintfFloat(HOME_DISTANCE_5Buffer, HOME_DISTANCE_5_SIZE, "%.1f", Tip_5_Distance);
				HOME_DISTANCE_5.invalidate();

				Unicode::snprintfFloat(HOME_DISTANCE_6Buffer, HOME_DISTANCE_6_SIZE, "%.1f", Tip_6_Distance);
				HOME_DISTANCE_6.invalidate();

				Unicode::snprintfFloat(HOME_DISTANCE_7Buffer, HOME_DISTANCE_7_SIZE, "%.1f", Tip_7_Distance);
				HOME_DISTANCE_7.invalidate();

				Unicode::snprintfFloat(HOME_DISTANCE_8Buffer, HOME_DISTANCE_8_SIZE, "%.1f", Tip_8_Distance);
				HOME_DISTANCE_8.invalidate();

					updateHomeBannerColor();

			Unicode::strncpy(HOME_BANNER_TEXT, HMI_HOME_BANNER_TEXT, 50);
			Unicode::snprintf(HOME_BANNERBuffer, HOME_BANNER_SIZE, "%s",HOME_BANNER_TEXT);
			HOME_BANNER.invalidate();

			Unicode::snprintfFloat(BOSS_HEIGHTBuffer, BOSS_HEIGHT_SIZE, "%.2f", HMI_BOSS_HEIGHT/100);
			BOSS_HEIGHT.invalidate();

//
			Unicode::snprintfFloat(CURRENT_HEIGHTBuffer, CURRENT_HEIGHT_SIZE, "%.2f", HMI_MANUAL_PLATEN_DISTANCE/100);
			CURRENT_HEIGHT.invalidate();
//	}
//
//	else
//	{
//		counter++;
//	}

}
