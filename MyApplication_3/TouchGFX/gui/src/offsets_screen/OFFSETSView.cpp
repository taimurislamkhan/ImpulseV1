#include <gui/offsets_screen/OFFSETSView.hpp>

OFFSETSView::OFFSETSView()
{

}

void OFFSETSView::setupScreen()
{
    OFFSETSViewBase::setupScreen();
}

void OFFSETSView::tearDownScreen()
{
    OFFSETSViewBase::tearDownScreen();
}

extern uint8_t HMI_Tip_1_Offset_Up;
extern uint8_t HMI_Tip_2_Offset_Up;
extern uint8_t HMI_Tip_3_Offset_Up;
extern uint8_t HMI_Tip_4_Offset_Up;

extern uint8_t HMI_Tip_1_Offset_Down;
extern uint8_t HMI_Tip_2_Offset_Down;
extern uint8_t HMI_Tip_3_Offset_Down;
extern uint8_t HMI_Tip_4_Offset_Down;

extern double HMI_Tip_1_Offset;
extern double HMI_Tip_2_Offset;
extern double HMI_Tip_3_Offset;
extern double HMI_Tip_4_Offset;

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

char HMI_PASSWORD_TEXT_OFFSETS[5]= "****";
Unicode::UnicodeChar PASSWORD_TEXT_OFFSETS[5];

void OFFSETSView::handleTickEvent()
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
				Unicode::strncpy(PASSWORD_TEXT_OFFSETS, HMI_PASSWORD_TEXT_OFFSETS,keypad_index);
			}
			else
			{
				Unicode::strncpy(PASSWORD_TEXT_OFFSETS,"                  ",15);

			}
			Unicode::snprintf(MODAL_PASSWORD_TEXTBuffer, MODAL_PASSWORD_TEXT_SIZE, "%s",PASSWORD_TEXT_OFFSETS);
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

	HMI_Tip_1_Offset_Up = Tip_1_Offset_Up.getPressed();
	HMI_Tip_2_Offset_Up = Tip_2_Offset_Up.getPressed();
	HMI_Tip_3_Offset_Up = Tip_3_Offset_Up.getPressed();
	HMI_Tip_4_Offset_Up = Tip_4_Offset_Up.getPressed();

	HMI_Tip_1_Offset_Down = Tip_1_Offset_Down.getPressed();
	HMI_Tip_2_Offset_Down = Tip_2_Offset_Down.getPressed();
	HMI_Tip_3_Offset_Down = Tip_3_Offset_Down.getPressed();
	HMI_Tip_4_Offset_Down = Tip_4_Offset_Down.getPressed();

	Unicode::snprintfFloat(Tip_1_OffsetBuffer, TIP_1_OFFSET_SIZE, "%.2f", HMI_Tip_1_Offset);
	Tip_1_Offset.invalidate();

	Unicode::snprintfFloat(Tip_2_OffsetBuffer, TIP_2_OFFSET_SIZE, "%.2f", HMI_Tip_2_Offset);
	Tip_2_Offset.invalidate();

	Unicode::snprintfFloat(Tip_3_OffsetBuffer, TIP_3_OFFSET_SIZE, "%.2f", HMI_Tip_3_Offset);
	Tip_3_Offset.invalidate();

	Unicode::snprintfFloat(Tip_4_OffsetBuffer, TIP_4_OFFSET_SIZE, "%.2f", HMI_Tip_4_Offset);
	Tip_4_Offset.invalidate();
}
