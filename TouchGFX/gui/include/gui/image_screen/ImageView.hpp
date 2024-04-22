#ifndef IMAGEVIEW_HPP
#define IMAGEVIEW_HPP

#include <gui_generated/image_screen/ImageViewBase.hpp>
#include <gui/image_screen/ImagePresenter.hpp>

class ImageView : public ImageViewBase
{
public:
    ImageView();
    virtual void setupScreen();

    virtual void handleTickEvent();
    virtual void handleKeyEvent(uint8_t key);
    virtual void handleDragEvent(const DragEvent& Event);
    virtual void handleGestureEvent(const GestureEvent& gestureEvent);

    void setThePositionOfTextBasedOnLanguage();
    void showNavigationMessage();

protected:
    static const int NUMBER_OF_IMAGES = 3;

    uint32_t tickCounter;
    uint8_t selectedImageIndex;

    void updateSelectedImage(uint8_t newImageIndex);
};

#endif // IMAGEVIEW_HPP
