#ifndef INFOPRESENTER_HPP
#define INFOPRESENTER_HPP

#include <gui/model/ModelListener.hpp>
#include <mvp/Presenter.hpp>

using namespace touchgfx;

class InfoView;

class InfoPresenter : public touchgfx::Presenter, public ModelListener
{
public:
    InfoPresenter(InfoView& v);

private:

    InfoView& view;
};


#endif // INFOPRESENTER_HPP
