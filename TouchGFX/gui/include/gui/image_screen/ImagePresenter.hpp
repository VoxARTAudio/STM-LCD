#ifndef IMAGEPRESENTER_HPP
#define IMAGEPRESENTER_HPP

#include <gui/model/ModelListener.hpp>
#include <mvp/Presenter.hpp>

using namespace touchgfx;

class ImageView;

class ImagePresenter : public touchgfx::Presenter, public ModelListener
{
public:
    ImagePresenter(ImageView& v);

private:
    ImagePresenter();
    ImageView& view;
};


#endif // IMAGEPRESENTER_HPP
