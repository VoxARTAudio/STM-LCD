#ifndef LISTPRESENTER_HPP
#define LISTPRESENTER_HPP

#include <gui/model/ModelListener.hpp>
#include <mvp/Presenter.hpp>

using namespace touchgfx;

class ListView;

class ListPresenter : public touchgfx::Presenter, public ModelListener
{
public:
    ListPresenter(ListView& v);

private:
    ListView& view;
};


#endif // LISTPRESENTER_HPP
