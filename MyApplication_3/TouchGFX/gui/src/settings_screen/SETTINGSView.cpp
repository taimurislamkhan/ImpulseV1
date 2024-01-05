#include <gui/settings_screen/SETTINGSView.hpp>
#include <touchgfx/Color.hpp>
#include <cstring>

char SCREEN_BANNER_TEXT7[] = "SETTINGS";
extern uint8_t FLAG_E_STOP_ACTIVE;

extern char HMI_HOME_BANNER_TEXT[50];
Unicode::UnicodeChar HOME_BANNER_TEXT2[50];
extern uint8_t HMI_BANNER_COLOR;
extern uint8_t  Home_Machine;
extern uint8_t FLAG_REQUIRE_HOME;
extern double HMI_TEST_PRESS_DOWN_POSITION;

extern uint8_t FLAG_SCREEN_LOCKED;
extern uint8_t HMI_MODAL_LOCK_NUM_1;
extern uint8_t HMI_MODAL_LOCK_NUM_2;
extern uint8_t HMI_MODAL_LOCK_NUM_3;
extern uint8_t HMI_MODAL_LOCK_NUM_4;
extern uint8_t HMI_MODAL_LOCK_NUM_5;
extern uint8_t HMI_MODAL_LOCK_NUM_6;
extern uint8_t HMI_MODAL_LOCK_NUM_7;
extern uint8_t HMI_MODAL_LOCK_NUM_8;
extern uint8_t HMI_MODAL_LOCK_NUM_9;
extern uint8_t keypad_index;
extern char keypad_text[4];

char HMI_PASSWORD_TEXT_SETTINGS[5]= "****";
Unicode::UnicodeChar PASSWORD_TEXT_SETTINGS[5];

SETTINGSView::SETTINGSView()
{

}

void SETTINGSView::setupScreen()
{
    SETTINGSViewBase::setupScreen();
}

void SETTINGSView::tearDownScreen()
{
    SETTINGSViewBase::tearDownScreen();
}

extern uint8_t Tip_1_Enabled;
extern uint16_t Tip_1_Energy_Val;
extern uint8_t Tip_2_Enabled;
extern uint16_t Tip_2_Energy_Val;
extern uint8_t Tip_3_Enabled;
extern uint16_t Tip_3_Energy_Val;
extern uint8_t Tip_4_Enabled;
extern uint16_t Tip_4_Energy_Val;


//void SETTINGSView::updateHomeBannerColor()
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


void SETTINGSView::handleTickEvent()
{

	if (FLAG_SCREEN_LOCKED !=0)
			{
				PASSWORD_MODAL.show();
				boxWithBorder1.setVisible(true);
				MODAL_PASSWORD_TEXT.setVisible(true);
				MODAL_NUM_1.setVisible(true);
				MODAL_NUM_2.setVisible(true);
				MODAL_NUM_3.setVisible(true);
				MODAL_NUM_4.setVisible(true);
				MODAL_NUM_5.setVisible(true);
				MODAL_NUM_6.setVisible(true);
				MODAL_NUM_7.setVisible(true);
				MODAL_NUM_8.setVisible(true);
				MODAL_NUM_9.setVisible(true);

				HMI_MODAL_LOCK_NUM_1= MODAL_NUM_1.getPressedState();
				HMI_MODAL_LOCK_NUM_2=MODAL_NUM_2.getPressedState();
				HMI_MODAL_LOCK_NUM_3=MODAL_NUM_3.getPressedState();
				HMI_MODAL_LOCK_NUM_4=MODAL_NUM_4.getPressedState();
				HMI_MODAL_LOCK_NUM_5=MODAL_NUM_5.getPressedState();
				HMI_MODAL_LOCK_NUM_6=MODAL_NUM_6.getPressedState();
				HMI_MODAL_LOCK_NUM_7=MODAL_NUM_7.getPressedState();
				HMI_MODAL_LOCK_NUM_8=MODAL_NUM_8.getPressedState();
				HMI_MODAL_LOCK_NUM_9=MODAL_NUM_9.getPressedState();

				if (keypad_index>=1)
				{
					Unicode::strncpy(PASSWORD_TEXT_SETTINGS, HMI_PASSWORD_TEXT_SETTINGS,keypad_index);
				}
				else
				{
					Unicode::strncpy(PASSWORD_TEXT_SETTINGS,"                  ",15);

				}
				Unicode::snprintf(MODAL_PASSWORD_TEXTBuffer, MODAL_PASSWORD_TEXT_SIZE, "%s",PASSWORD_TEXT_SETTINGS);
				MODAL_PASSWORD_TEXT.invalidate();


			}
			else
			{
				PASSWORD_MODAL.hide();
				boxWithBorder1.setVisible(false);
				boxWithBorder1.invalidate();
				MODAL_PASSWORD_TEXT.setVisible(false);
				MODAL_PASSWORD_TEXT.invalidate();
				MODAL_NUM_1.setVisible(false);
				MODAL_NUM_2.setVisible(false);
				MODAL_NUM_3.setVisible(false);
				MODAL_NUM_4.setVisible(false);
				MODAL_NUM_5.setVisible(false);
				MODAL_NUM_6.setVisible(false);
				MODAL_NUM_7.setVisible(false);
				MODAL_NUM_8.setVisible(false);
				MODAL_NUM_9.setVisible(false);
			}

    if (Tip_1_Enabled)
    {
        Tip_1_Enable_BG.setColor(0xFF0303);
        Tip_1_Enable_BG.invalidate();
    }
    else
    {
        Tip_1_Enable_BG.setColor(0x00FF1E);
        Tip_1_Enable_BG.invalidate();
    }

    if (Tip_2_Enabled)
    {
        Tip_2_Enable_BG.setColor(0xFF0303);
        Tip_2_Enable_BG.invalidate();
    }
    else
    {
        Tip_2_Enable_BG.setColor(0x00FF1E);
        Tip_2_Enable_BG.invalidate();
    }

    if (Tip_3_Enabled)
    {
        Tip_3_Enable_BG.setColor(0xFF0303);
        Tip_3_Enable_BG.invalidate();
    }
    else
    {
        Tip_3_Enable_BG.setColor(0x00FF1E);
        Tip_3_Enable_BG.invalidate();
    }
    if (Tip_4_Enabled)
    {
        Tip_4_Enable_BG.setColor(0xFF0303);
        Tip_4_Enable_BG.invalidate();
    }
    else
    {
        Tip_4_Enable_BG.setColor(0x00FF1E);
        Tip_4_Enable_BG.invalidate();
    }

    Unicode::snprintf(Tip_1_EnergyBuffer, TIP_1_ENERGY_SIZE, "%d", Tip_1_Energy_Val);
    Tip_1_Energy.invalidate();
    Unicode::snprintf(Tip_2_EnergyBuffer, TIP_2_ENERGY_SIZE, "%d", Tip_2_Energy_Val);
    Tip_2_Energy.invalidate();
    Unicode::snprintf(Tip_3_EnergyBuffer, TIP_3_ENERGY_SIZE, "%d", Tip_3_Energy_Val);
    Tip_3_Energy.invalidate();
    Unicode::snprintf(Tip_4_EnergyBuffer, TIP_4_ENERGY_SIZE, "%d", Tip_4_Energy_Val);
    Tip_4_Energy.invalidate();

//	updateHomeBannerColor();
//	Unicode::strncpy(HOME_BANNER_TEXT2, HMI_HOME_BANNER_TEXT, 50);
//	Unicode::snprintf(HOME_BANNERBuffer, HOME_BANNER_SIZE, "%s",HOME_BANNER_TEXT2);
//	HOME_BANNER.invalidate();
//
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
		Unicode::strncpy(HOME_BANNER_TEXT2, HMI_HOME_BANNER_TEXT, 50);
		Unicode::snprintf(HOME_BANNERBuffer, HOME_BANNER_SIZE, "%s",HOME_BANNER_TEXT2);
		HOME_BANNER.invalidate();

	}

	else
	{
		HOME_BANNER_BG.setColor(touchgfx::Color::getColorFromRGB(0, 21, 255)); // blue
		HOME_BANNER_BG.invalidate();
		Unicode::strncpy(HOME_BANNER_TEXT2, SCREEN_BANNER_TEXT7, strlen(SCREEN_BANNER_TEXT7)+1);
		Unicode::snprintf(HOME_BANNERBuffer, HOME_BANNER_SIZE, "%s",HOME_BANNER_TEXT2);
		HOME_BANNER.invalidate();

	}
}
