#include <gui/tip_4_screen/TIP_4View.hpp>

TIP_4View::TIP_4View()
{

}

void TIP_4View::setupScreen()
{
    TIP_4ViewBase::setupScreen();
}

void TIP_4View::tearDownScreen()
{
    TIP_4ViewBase::tearDownScreen();
}

extern uint8_t Tip_4_Enabled;
extern uint8_t Tip_4_Energy_Up_Pressed;
extern uint8_t Tip_4_Energy_Down_Pressed;
extern uint16_t Tip_4_Energy_Val;

extern uint8_t Tip_4_Distance_Up_Pressed;
extern uint8_t Tip_4_Distance_Down_Pressed;
extern double Tip_4_Distance_Val;


void TIP_4View::handleTickEvent()
{
    Tip_4_Energy_Up_Pressed = Tip_4_Energy_Up.getPressed();
    Tip_4_Energy_Down_Pressed = Tip_4_Energy_Down.getPressed();
    Unicode::snprintf(Tip_4_EnergyBuffer, TIP_4_ENERGY_SIZE, "%d", Tip_4_Energy_Val);
    Tip_4_Energy.invalidate();

    Tip_4_Distance_Up_Pressed = Tip_4_Distance_Up.getPressed();
    Tip_4_Distance_Down_Pressed = Tip_4_Distance_Down.getPressed();
    Unicode::snprintfFloat(Tip_4_DistanceBuffer, TIP_4_DISTANCE_SIZE, "%.1f", Tip_4_Distance_Val);
    Tip_4_Distance.invalidate();
}

void TIP_4View::afterTransition()
{
	Tip_4_Enable.forceState(Tip_4_Enabled);
	Tip_4_Enable.invalidate();
}

void TIP_4View::Tip_4_Pressed()
{
	Tip_4_Enabled ^=1;
}
