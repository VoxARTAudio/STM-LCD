#include <gui/image_screen/ImageView.hpp>
#include <BitmapDatabase.hpp>
#include <touchgfx/Utils.hpp>
#include <texts/TextKeysAndLanguages.hpp>

ImageView::ImageView() :
    tickCounter(0),
    selectedImageIndex(0)
{

}

void ImageView::setupScreen()
{
    setThePositionOfTextBasedOnLanguage();

    ImageViewBase::setupScreen();

    showNavigationMessage();
}

void ImageView::setThePositionOfTextBasedOnLanguage()
{
    switch (Texts::getLanguage())
    {
    case DK:
        joystickText.setXY(123, 76);
        break;
    case FR:
        joystickText.setXY(123, 76);
        break;
    case KR:
        joystickText.setXY(123, 63);
        break;
    case CN:
        joystickText.setXY(123, 75);
        break;
    case GB:
        joystickText.setXY(123, 58);
        break;
    case JP:
        joystickText.setXY(123, 77);
        break;
    case TN:
        joystickText.setXY(127, 84);
        break;
    default:
        break;
    }
}


void ImageView::showNavigationMessage()
{

    uint16_t delay = 110;
    uint16_t animationDuration = 35;

    joystickOverlay.setFadeAnimationDelay(delay);
    joystickOverlay.startFadeAnimation(0, animationDuration, EasingEquations::cubicEaseIn);

    joystickText.setFadeAnimationDelay(delay);
    joystickText.startFadeAnimation(0, animationDuration, EasingEquations::cubicEaseIn);

}

void ImageView::handleTickEvent()
{
    tickCounter++;
}

void ImageView::handleKeyEvent(uint8_t key)
{
    if (key == '6')
    {
        // Right
        updateSelectedImage((selectedImageIndex == NUMBER_OF_IMAGES - 1) ? 0 : selectedImageIndex + 1);
    }
    else if (key == '4')
    {
        // Left
        updateSelectedImage((selectedImageIndex == 0) ? NUMBER_OF_IMAGES - 1 : selectedImageIndex - 1);
    }
    else if (key == '5' && joystickOverlay.getAlpha() == 0)
    {
        // Select
        goToScreen1();
    }
}

void ImageView::updateSelectedImage(uint8_t newImageIndex)
{
    selectedImageIndex = newImageIndex;

    switch (selectedImageIndex)
    {
    case 0:
        image.setBitmap(Bitmap(BITMAP_IMAGE0_ID));
        break;
    case 1:
        image.setBitmap(Bitmap(BITMAP_IMAGE01_ID));
        break;
    case 2:
        image.setBitmap(Bitmap(BITMAP_IMAGE02_ID));
        break;
    default:
        break;
    }
    image.invalidate();
}

void ImageView::handleGestureEvent(const GestureEvent& gestureEvent)
{
    //For changing Image
    touchgfx::GestureEvent::GestureEventType typeOfGesture = gestureEvent.getType();

    if (joystickOverlay.getAlpha() > 0)
    {

        return;

    }
    else
    {
        if (typeOfGesture == touchgfx::GestureEvent::SWIPE_HORIZONTAL)
        {

            updateSelectedImage((selectedImageIndex == NUMBER_OF_IMAGES - 1) ? 0 : selectedImageIndex + 1);

        }
    }
}

void ImageView::handleDragEvent(const DragEvent& Event)
{
    //For Returning to menu
    ImageViewBase::handleDragEvent(Event);

    if (joystickOverlay.getAlpha() > 0)
    {

        return;

    }

    int deltaY = abs(Event.getDeltaY());
    if (deltaY > 20)
    {
        goToScreen1();
    }
}
