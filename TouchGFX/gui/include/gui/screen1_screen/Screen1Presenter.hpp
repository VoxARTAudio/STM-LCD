#ifndef SCREEN1_PRESENTER_HPP
#define SCREEN1_PRESENTER_HPP

#include <gui/model/ModelListener.hpp>
#include <mvp/Presenter.hpp>

using namespace touchgfx;

class Screen1View;

class Screen1Presenter : public Presenter, public ModelListener
{
public:
    Screen1Presenter(Screen1View& v);

private:
    Screen1View& view;
};


#endif // SCREEN1_PRESENTER_HPP
