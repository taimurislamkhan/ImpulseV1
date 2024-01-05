#ifndef ALARM_HISTORYVIEW_HPP
#define ALARM_HISTORYVIEW_HPP

#include <gui_generated/alarm_history_screen/ALARM_HISTORYViewBase.hpp>
#include <gui/alarm_history_screen/ALARM_HISTORYPresenter.hpp>

class ALARM_HISTORYView : public ALARM_HISTORYViewBase
{
public:
    ALARM_HISTORYView();
    virtual ~ALARM_HISTORYView() {}
    virtual void setupScreen();
    virtual void tearDownScreen();
protected:
};

#endif // ALARM_HISTORYVIEW_HPP
