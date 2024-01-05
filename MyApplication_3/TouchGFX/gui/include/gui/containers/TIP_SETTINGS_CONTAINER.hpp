#ifndef TIP_SETTINGS_CONTAINER_HPP
#define TIP_SETTINGS_CONTAINER_HPP

#include <gui_generated/containers/TIP_SETTINGS_CONTAINERBase.hpp>

class TIP_SETTINGS_CONTAINER : public TIP_SETTINGS_CONTAINERBase
{
public:
    TIP_SETTINGS_CONTAINER();
    virtual ~TIP_SETTINGS_CONTAINER() {}

    virtual void initialize();
protected:
};

#endif // TIP_SETTINGS_CONTAINER_HPP
