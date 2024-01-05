#include <gui/tip_3_screen/TIP_3View.hpp>

TIP_3View::TIP_3View()
{

}

void TIP_3View::setupScreen()
{
    TIP_3ViewBase::setupScreen();
}

void TIP_3View::tearDownScreen()
{
    TIP_3ViewBase::tearDownScreen();
}

extern uint8_t Tip_3_Enabled;
extern uint8_t Tip_3_Energy_Up_Pressed;
extern uint8_t Tip_3_Energy_Down_Pressed;
extern uint16_t Tip_3_Energy_Val;

extern uint8_t Tip_3_Distance_Up_Pressed;
extern uint8_t Tip_3_Distance_Down_Pressed;
extern double Tip_3_Distance_Val;


void TIP_3View::handleTickEvent()
{
    Tip_3_Energy_Up_Pressed = Tip_3_Energy_Up.getPressed();
    Tip_3_Energy_Down_Pressed = Tip_3_Energy_Down.getPressed();
    Unicode::snprintf(Tip_3_EnergyBuffer, TIP_3_ENERGY_SIZE, "%d", Tip_3_Energy_Val);
    Tip_3_Energy.invalidate();

    Tip_3_Distance_Up_Pressed = Tip_3_Distance_Up.getPressed();
    Tip_3_Distance_Down_Pressed = Tip_3_Distance_Down.getPressed();
    Unicode::snprintfFloat(Tip_3_DistanceBuffer, TIP_3_DISTANCE_SIZE, "%.1f", Tip_3_Distance_Val);
    Tip_3_Distance.invalidate();
}

void TIP_3View::afterTransition()
{
	Tip_3_Enable.forceState(Tip_3_Enabled);
	Tip_3_Enable.invalidate();
}

void TIP_3View::Tip_3_Pressed()
{
	Tip_3_Enabled ^=1;
}

