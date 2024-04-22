#ifndef SCREEN1_VIEW_HPP
#define SCREEN1_VIEW_HPP

#include <gui_generated/screen1_screen/Screen1ViewBase.hpp>
#include <gui/screen1_screen/Screen1Presenter.hpp>

class Screen1View : public Screen1ViewBase
{
public:
    enum AnimationState
    {
        ANIMATION_READY = 0,
        ANIMATION_STEP_0,
        ANIMATION_STEP_1,
        ANIMATION_STEP_2,
        ANIMATION_STEP_3,
        ANIMATION_STEP_4,
        ANIMATION_STEP_5,
        ANIMATION_DONE
    };

    static const int ANIMATION_STEP_DURATION = 40;

    enum MenuAnimationState
    {
        MENU_ANIMATION_READY = 0,
        MENU_ANIMATION_STEP_0,
        MENU_ANIMATION_STEP_1,
        MENU_ANIMATION_DONE
    };

    static const int MENU_ANIMATION_MOVE_X_SIZE = 40;
    static const int MENU_ANIMATION_STEP_DURATION = 40;
    static const int NUMBER_OF_MENU_ITEMS = 4;

    Screen1View();

    virtual void setupScreen();

    virtual void handleTickEvent();
    virtual void handleKeyEvent(uint8_t key);

    virtual void menuLeftButtonClicked();
    virtual void menuRightButtonClicked();
    virtual void menuHiddenButtonClicked();

protected:

    uint32_t animationTickCounter;
    AnimationState animationState;
    uint32_t menuAnimationTickCounter;
    bool menuAnimationDirectionLeft;
    MenuAnimationState menuAnimationState;
    int16_t subtitleStartPositionX;
    int16_t subtitleStartPositionY;
    int16_t logoXStartPositionX;
    int16_t logoXStartPositionY;
    uint8_t selectedMenuItem;

    void menuSelectLeft();
    void menuSelectRight();

    void selectProperMenu();

private:
    void setSelectedMenuItem();
};

#endif // SCREEN1_VIEW_HPP
