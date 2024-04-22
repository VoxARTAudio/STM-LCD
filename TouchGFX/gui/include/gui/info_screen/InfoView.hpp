#ifndef INFOVIEW_HPP
#define INFOVIEW_HPP

#include <gui_generated/info_screen/InfoViewBase.hpp>
#include <gui/info_screen/InfoPresenter.hpp>

class InfoView : public InfoViewBase
{
public:

    InfoView();
    virtual ~InfoView() {}
    virtual void setupScreen();
    virtual void tearDownScreen();

protected:

    virtual void handleKeyEvent(uint8_t key);
};

#endif // INFOVIEW_HPP
