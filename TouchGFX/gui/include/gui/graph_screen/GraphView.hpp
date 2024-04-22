#ifndef GRAPHVIEW_HPP
#define GRAPHVIEW_HPP

#include <gui_generated/graph_screen/GraphViewBase.hpp>
#include <gui/graph_screen/GraphPresenter.hpp>

class GraphView : public GraphViewBase
{
public:
    GraphView();

    virtual void handleTickEvent();
    virtual void handleKeyEvent(uint8_t key);

protected:

    int tickCounter;
};

#endif // GRAPHVIEW_HPP
