#include <gui/screen1_screen/Screen1View.hpp>
#include <BitmapDatabase.hpp>
#include <touchgfx/EasingEquations.hpp>
#include <texts/TextKeysAndLanguages.hpp>

Screen1View::Screen1View() :

    animationTickCounter(0),
    animationState(ANIMATION_STEP_0),
    menuAnimationTickCounter(0),
    menuAnimationDirectionLeft(false),
    menuAnimationState(MENU_ANIMATION_READY),
    subtitleStartPositionX(0),
    subtitleStartPositionY(0),
    logoXStartPositionX(0),
    logoXStartPositionY(0),
    selectedMenuItem(0)
{

}

void Screen1View::setupScreen()
{
    animationTickCounter = 0;
    animationState = ANIMATION_STEP_0;
    menuAnimationTickCounter = 0;
    menuAnimationDirectionLeft = true ;
    menuAnimationState = MENU_ANIMATION_READY;
    subtitleStartPositionX = 0;
    subtitleStartPositionY = 0;
    logoXStartPositionX = 0;
    logoXStartPositionY = 0;
    selectedMenuItem = 0;
    menuHiddenButton.setTouchable(false);
}

void Screen1View::handleTickEvent()
{
    if (animationState > ANIMATION_READY)
    {
        animationTickCounter++;

        if (animationState == ANIMATION_STEP_0)
        {
            if (animationTickCounter < ANIMATION_STEP_DURATION)
            {
                int16_t deltaXforLogoX = EasingEquations::cubicEaseInOut(animationTickCounter, 0, ((280 - logoX.getWidth()) / 2), ANIMATION_STEP_DURATION);
                logoX.moveTo(((320 - logoX.getWidth()) / 2) + deltaXforLogoX, logoX.getY());

                int16_t deltaXforLogoText = EasingEquations::cubicEaseInOut(animationTickCounter, 0, (280 / 2), ANIMATION_STEP_DURATION);

                logoTextContainer.moveTo(320 / 2 - deltaXforLogoText, logoTextContainer.getY());

                int16_t newWidth = logoX.getX() - logoTextContainer.getX();
                logoTextContainer.setWidth((newWidth > 0) ? newWidth : 0);
                logoTextContainer.invalidate();
            }
            else
            {
                animationState = ANIMATION_STEP_1;
                animationTickCounter = 0;
            }
        }
        else if (animationState == ANIMATION_STEP_1)
        {
            if (animationTickCounter < ANIMATION_STEP_DURATION)
            {
                uint8_t deltaAlpha = (uint8_t)EasingEquations::cubicEaseInOut(animationTickCounter, 0, 255, ANIMATION_STEP_DURATION);
                subtitle.setAlpha(deltaAlpha);
                subtitle.invalidate();
            }
            else
            {
                animationState = ANIMATION_STEP_2;
                animationTickCounter = 0;
            }
        }
        else if (animationState == ANIMATION_STEP_2)
        {
            if (animationTickCounter >= ANIMATION_STEP_DURATION)
            {
                animationState = ANIMATION_STEP_3;
                animationTickCounter = 0;

                subtitleStartPositionX = -1; // Reset
            }
        }
        else if (animationState == ANIMATION_STEP_3)
        {
            if (subtitleStartPositionX == -1)
            {
                subtitleStartPositionX = subtitle.getX();
                subtitleStartPositionY = subtitle.getY();
                logoXStartPositionX = logoX.getX();
                logoXStartPositionY = logoX.getY();
            }

            if (animationTickCounter < ANIMATION_STEP_DURATION)
            {
                int16_t deltaXforLogoX = EasingEquations::cubicEaseInOut(animationTickCounter, 0, 266 - logoXStartPositionX, ANIMATION_STEP_DURATION);
                int16_t deltaYforLogoY = EasingEquations::cubicEaseInOut(animationTickCounter, 0, 180 - logoXStartPositionY, ANIMATION_STEP_DURATION);
                logoX.moveTo(logoXStartPositionX + deltaXforLogoX, logoXStartPositionY + deltaYforLogoY);

                int16_t deltaXforSubtitleX = EasingEquations::cubicEaseInOut(animationTickCounter, 0, 29 - subtitleStartPositionX, ANIMATION_STEP_DURATION);
                int16_t deltaYforSubtitleY = EasingEquations::cubicEaseInOut(animationTickCounter, 0, 202 - subtitleStartPositionY, ANIMATION_STEP_DURATION);
                subtitle.moveTo(subtitleStartPositionX + deltaXforSubtitleX, subtitleStartPositionY + deltaYforSubtitleY);
            }
            else
            {
                animationState = ANIMATION_STEP_4;
                animationTickCounter = 0;
            }
        }
        else if (animationState == ANIMATION_STEP_4)
        {
            if (animationTickCounter < ANIMATION_STEP_DURATION)
            {
                int16_t deltaX = EasingEquations::cubicEaseInOut(animationTickCounter, 0, seperatorLine.getWidth(), ANIMATION_STEP_DURATION);
                seperatorLine.moveTo(-seperatorLine.getWidth() + deltaX, seperatorLine.getY());

                int16_t deltaYText = EasingEquations::cubicEaseInOut(animationTickCounter, 0, logoTextContainer.getHeight(), ANIMATION_STEP_DURATION);
                logoText.moveTo(logoText.getX(), deltaYText);
            }
            else
            {
                animationState = ANIMATION_STEP_5;
                animationTickCounter = 0;
            }
        }
        else if (animationState == ANIMATION_STEP_5)
        {
            if (animationTickCounter < ANIMATION_STEP_DURATION)
            {
                uint8_t deltaAlpha = (uint8_t)EasingEquations::cubicEaseInOut(animationTickCounter, 0, 255, ANIMATION_STEP_DURATION);

                menuIcon0.setAlpha(deltaAlpha);
                menuLeftArrow.setAlpha(deltaAlpha);
                menuRightArrow.setAlpha(deltaAlpha);
                headline.setAlpha(deltaAlpha);
                headlineSeperatorLine.setAlpha(deltaAlpha);
                menuHiddenButton.setTouchable(true);

                menuIcon0.invalidate();
                menuLeftArrow.invalidate();
                menuRightArrow.invalidate();
                headline.invalidate();
                headlineSeperatorLine.invalidate();
            }
            else
            {
                animationState = ANIMATION_READY;
                animationTickCounter = 0;
            }
        }
    }

    if (menuAnimationState > MENU_ANIMATION_READY)
    {
        menuAnimationTickCounter++;

        int16_t centerPositionOfMenuIcon = (320 - menuIcon0.getWidth()) / 2;
        int16_t centerPositionOfHeadline = (320 - headline.getWidth()) / 2;
        int16_t directionModifier = (menuAnimationDirectionLeft) ? -1 : 1;
        int16_t deltaX = directionModifier * EasingEquations::cubicEaseInOut(menuAnimationTickCounter, 0, MENU_ANIMATION_MOVE_X_SIZE, MENU_ANIMATION_STEP_DURATION - 10);
        int16_t deltaX_Headline = directionModifier * EasingEquations::cubicEaseInOut(menuAnimationTickCounter, 0, MENU_ANIMATION_MOVE_X_SIZE, MENU_ANIMATION_STEP_DURATION);
        uint8_t deltaAlpha = (uint8_t) EasingEquations::cubicEaseInOut(menuAnimationTickCounter, 0, 255, MENU_ANIMATION_STEP_DURATION);

        if (menuAnimationState == MENU_ANIMATION_STEP_0)
        {
            menuLeftArrow.setTouchable(false);
            menuRightArrow.setTouchable(false);

            if (menuAnimationTickCounter < MENU_ANIMATION_STEP_DURATION)
            {
                menuIcon0.setAlpha(255 - deltaAlpha);
                menuIcon0.invalidate();
                menuIcon0.moveTo(centerPositionOfMenuIcon + deltaX, menuIcon0.getY());

                headline.setAlpha(255 - deltaAlpha);
                headline.invalidate();
                headline.moveTo(centerPositionOfHeadline + deltaX_Headline, headline.getY());
            }
            else
            {
                menuAnimationState = MENU_ANIMATION_STEP_1;
                menuAnimationTickCounter = 0;

                setSelectedMenuItem();
            }
        }
        else if (menuAnimationState == MENU_ANIMATION_STEP_1)
        {
            if (menuAnimationTickCounter < MENU_ANIMATION_STEP_DURATION)
            {
                menuIcon0.setAlpha(deltaAlpha);
                menuIcon0.invalidate();
                menuIcon0.moveTo(centerPositionOfMenuIcon - (directionModifier * MENU_ANIMATION_MOVE_X_SIZE) + deltaX, menuIcon0.getY());

                headline.setAlpha(deltaAlpha);
                headline.invalidate();
                headline.moveTo(centerPositionOfHeadline - (directionModifier * MENU_ANIMATION_MOVE_X_SIZE) + deltaX_Headline, headline.getY());
            }
            else
            {
                menuLeftArrow.setTouchable(true);
                menuRightArrow.setTouchable(true);

                menuAnimationState = MENU_ANIMATION_READY;
                menuAnimationTickCounter = 0;
            }
        }
    }
}

void Screen1View::handleKeyEvent(uint8_t key)
{
    if (animationState != ANIMATION_READY || menuAnimationState != MENU_ANIMATION_READY)
    {
        return;
    }

    if (key == '6')
    {
        // Right
        menuSelectRight();
    }
    else if (key == '4')
    {
        // Left
        menuSelectLeft();
    }
    else if (key == '5')
    {
        // Select
        selectProperMenu();
    }


}

void Screen1View::selectProperMenu()
{
    switch (selectedMenuItem)
    {
    case 0:
        goToInfoScreen();
        break;
    case 1:
        goToImageScreen();
        break;
    case 2:
        goToListScreen();
        break;
    case 3:
        goToGraphScreen();
        break;
    default:
        break;
    }
}

void Screen1View::menuSelectLeft()
{
    menuAnimationState = MENU_ANIMATION_STEP_0;
    menuAnimationTickCounter = 0;

    menuAnimationDirectionLeft = false;
}

void Screen1View::menuSelectRight()
{
    menuAnimationState = MENU_ANIMATION_STEP_0;
    menuAnimationTickCounter = 0;

    menuAnimationDirectionLeft = true;
}

void Screen1View::setSelectedMenuItem()
{
    if (menuAnimationDirectionLeft)
    {
        selectedMenuItem = (selectedMenuItem == 0) ? NUMBER_OF_MENU_ITEMS - 1 : selectedMenuItem - 1;
    }
    else
    {
        selectedMenuItem = (selectedMenuItem == NUMBER_OF_MENU_ITEMS - 1) ? 0 : selectedMenuItem + 1;
    }

    switch (selectedMenuItem)
    {
    case 0:
        menuIcon0.setBitmap(Bitmap(BITMAP_SUBDEMOICON_INFO_ID));
        headline.setTypedText(touchgfx::TypedText(T_INFORMATIONMENUITEM));
        break;
    case 1:
        menuIcon0.setBitmap(Bitmap(BITMAP_SUBDEMOICON_IMAGE_ID));
        headline.setTypedText(touchgfx::TypedText(T_PICTURESMENUITEM));
        break;
    case 2:
        menuIcon0.setBitmap(Bitmap(BITMAP_SUBDEMOICON_LIST_ID));
        headline.setTypedText(touchgfx::TypedText(T_LANGUAGESMENUITEM));
        break;
    case 3:
        menuIcon0.setBitmap(Bitmap(BITMAP_SUBDEMOICON_GRAPH_ID));
        headline.setTypedText(touchgfx::TypedText(T_GRAPHMENUITEM));
        break;
    default:
        break;
    }
}

void Screen1View::menuLeftButtonClicked()
{
    menuSelectLeft();
}

void Screen1View::menuRightButtonClicked()
{
    menuSelectRight();
}

void Screen1View::menuHiddenButtonClicked()
{
    selectProperMenu();
}
