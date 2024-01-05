#include <gui/tip_2_screen/TIP_2View.hpp>

TIP_2View::TIP_2View()
{

}

void TIP_2View::setupScreen()
{
    TIP_2ViewBase::setupScreen();
}

void TIP_2View::tearDownScreen()
{
    TIP_2ViewBase::tearDownScreen();
}

extern uint8_t Tip_2_Enabled;
extern uint8_t Tip_2_Energy_Up_Pressed;
extern uint8_t Tip_2_Energy_Down_Pressed;
extern uint16_t Tip_2_Energy_Val;

extern uint8_t Tip_2_Distance_Up_Pressed;
extern uint8_t Tip_2_Distance_Down_Pressed;
extern double Tip_2_Distance_Val;


void TIP_2View::handleTickEvent()
{
    Tip_2_Energy_Up_Pressed = Tip_2_Energy_Up.getPressed();
    Tip_2_Energy_Down_Pressed = Tip_2_Energy_Down.getPressed();
    Unicode::snprintf(Tip_2_EnergyBuffer, TIP_2_ENERGY_SIZE, "%d", Tip_2_Energy_Val);
    Tip_2_Energy.invalidate();

    Tip_2_Distance_Up_Pressed = Tip_2_Distance_Up.getPressed();
    Tip_2_Distance_Down_Pressed = Tip_2_Distance_Down.getPressed();
    Unicode::snprintfFloat(Tip_2_DistanceBuffer, TIP_2_DISTANCE_SIZE, "%.1f", Tip_2_Distance_Val);
    Tip_2_Distance.invalidate();
}

void TIP_2View::afterTransition()
{
	Tip_2_Enable.forceState(Tip_2_Enabled);
	Tip_2_Enable.invalidate();
}

void TIP_2View::Tip_2_Pressed()
{
	Tip_2_Enabled ^=1;
}
