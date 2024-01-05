#include <gui/test_press_screen/TEST_PRESSView.hpp>

extern uint8_t HMI_TEST_PRESS_PLATEN_UP;
extern uint8_t HMI_TEST_PRESS_PLATEN_DOWN;
extern uint8_t HMI_TEST_PRESS_SAVE_UP;
extern uint8_t HMI_TEST_PRESS_SAVE_DOWN;
extern uint8_t HMI_TEST_PRESS_START_CYCLE;
extern double  HMI_TEST_PRESS_UP_POSITION;
extern double  HMI_TEST_PRESS_DOWN_POSITION;
extern double  HMI_TEST_PRESS_PLATEN_DISTANCE;
extern double  HMI_MANUAL_PLATEN_DISTANCE;

TEST_PRESSView::TEST_PRESSView()
{

}

void TEST_PRESSView::setupScreen()
{
    TEST_PRESSViewBase::setupScreen();
}

void TEST_PRESSView::tearDownScreen()
{
    TEST_PRESSViewBase::tearDownScreen();
}

void TEST_PRESSView::handleTickEvent()
{
	HMI_TEST_PRESS_PLATEN_UP       =  TEST_PRESS_PLATEN_UP.getPressedState();
	HMI_TEST_PRESS_PLATEN_DOWN     =  TEST_PRESS_PLATEN_DOWN.getPressedState();
	HMI_TEST_PRESS_SAVE_UP         =  TEST_PRESS_SAVE_UP.getPressedState();
	HMI_TEST_PRESS_SAVE_DOWN       =  TEST_PRESS_SAVE_DOWN.getPressedState();
	HMI_TEST_PRESS_START_CYCLE     =  TEST_PRESS_START_CYCLE.getPressedState();

	Unicode::snprintfFloat(TEST_PRESS_UP_POSITIONBuffer, TEST_PRESS_UP_POSITION_SIZE, "%.2f", HMI_TEST_PRESS_UP_POSITION/100);
	Unicode::snprintfFloat(TEST_PRESS_DOWN_POSITIONBuffer, TEST_PRESS_DOWN_POSITION_SIZE, "%.2f", HMI_TEST_PRESS_DOWN_POSITION/100);
	Unicode::snprintfFloat(TEST_PRESS_PLATEN_DISTANCEBuffer, TEST_PRESS_PLATEN_DISTANCE_SIZE, "%.2f", HMI_MANUAL_PLATEN_DISTANCE/100);
	TEST_PRESS_UP_POSITION.invalidate();
	TEST_PRESS_DOWN_POSITION.invalidate();
	TEST_PRESS_PLATEN_DISTANCE.invalidate();
}
