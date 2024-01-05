#include <gui/tip_1_screen/TIP_1View.hpp>

TIP_1View::TIP_1View()
{

}

void TIP_1View::setupScreen()
{
    TIP_1ViewBase::setupScreen();
}

void TIP_1View::tearDownScreen()
{
    TIP_1ViewBase::tearDownScreen();
}

extern uint8_t Tip_1_Enabled;
extern uint8_t Tip_1_Energy_Up_Pressed;
extern uint8_t Tip_1_Energy_Down_Pressed;
extern uint16_t Tip_1_Energy_Val;

extern uint8_t Tip_1_Distance_Up_Pressed;
extern uint8_t Tip_1_Distance_Down_Pressed;
extern double Tip_1_Distance_Val;


void TIP_1View::handleTickEvent()
{

	Tip_1_Energy_Up_Pressed = Tip_1_Energy_Up.getPressed();
	Tip_1_Energy_Down_Pressed = Tip_1_Energy_Down.getPressed();
	Unicode::snprintf(Tip_1_EnergyBuffer, TIP_1_ENERGY_SIZE, "%d",Tip_1_Energy_Val);
	Tip_1_Energy.invalidate();

	Tip_1_Distance_Up_Pressed = Tip_1_Distance_Up.getPressed();
	Tip_1_Distance_Down_Pressed = Tip_1_Distance_Down.getPressed();
	Unicode::snprintfFloat(Tip_1_DistanceBuffer, TIP_1_DISTANCE_SIZE, "%.1f",Tip_1_Distance_Val);
	Tip_1_Distance.invalidate();

}


void TIP_1View::afterTransition()
{
	Tip_1_Enable.forceState(Tip_1_Enabled);
	Tip_1_Enable.invalidate();
}

void TIP_1View::Tip_1_Pressed()
{
	Tip_1_Enabled ^=1;
}
