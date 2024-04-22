#include <gui/info_screen/InfoView.hpp>
#include <touchgfx/EasingEquations.hpp>
#include <touchgfx/hal/HAL.hpp>

InfoView::InfoView()
{

}

void InfoView::setupScreen()
{
    InfoViewBase::setupScreen();

    HAL::getInstance()->setFrameRateCompensation(true);

}

void InfoView::tearDownScreen()
{
    InfoViewBase::tearDownScreen();

    HAL::getInstance()->setFrameRateCompensation(false);

}

void InfoView::handleKeyEvent(uint8_t key)
{
    if (key == '5')
    {
        goToScreen1();
    }
}
