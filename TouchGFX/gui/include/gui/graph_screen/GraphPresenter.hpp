#ifndef GRAPHPRESENTER_HPP
#define GRAPHPRESENTER_HPP

#include <gui/model/ModelListener.hpp>
#include <mvp/Presenter.hpp>

using namespace touchgfx;

class GraphView;

class GraphPresenter : public touchgfx::Presenter, public ModelListener
{
public:
    GraphPresenter(GraphView& v);

private:
    GraphPresenter();
    GraphView& view;
};

#endif // GRAPHPRESENTER_HPP
