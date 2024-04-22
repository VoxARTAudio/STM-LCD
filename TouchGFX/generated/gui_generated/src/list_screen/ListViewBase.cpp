/*********************************************************************************/
/********** THIS FILE IS GENERATED BY TOUCHGFX DESIGNER, DO NOT MODIFY ***********/
/*********************************************************************************/
#include <gui_generated/list_screen/ListViewBase.hpp>
#include <touchgfx/Color.hpp>
#include <images/BitmapDatabase.hpp>
#include <texts/TextKeysAndLanguages.hpp>

ListViewBase::ListViewBase() :
    updateItemCallback(this, &ListViewBase::updateItemCallbackHandler),
    buttonCallback(this, &ListViewBase::buttonCallbackHandler)
{
    __background.setPosition(0, 0, 320, 240);
    __background.setColor(touchgfx::Color::getColorFromRGB(0, 0, 0));
    add(__background);

    image1.setXY(0, 0);
    image1.setBitmap(touchgfx::Bitmap(BITMAP_LISTBACKGROUND_ID));
    add(image1);

    languageSelectorWheel.setPosition(60, 86, 200, 118);
    languageSelectorWheel.setHorizontal(false);
    languageSelectorWheel.setCircular(true);
    languageSelectorWheel.setEasingEquation(touchgfx::EasingEquations::backEaseOut);
    languageSelectorWheel.setSwipeAcceleration(10);
    languageSelectorWheel.setDragAcceleration(10);
    languageSelectorWheel.setNumberOfItems(7);
    languageSelectorWheel.setSelectedItemOffset(40);
    languageSelectorWheel.setOvershootPercentage(0);
    languageSelectorWheel.setSelectedItemExtraSize(0, 0);
    languageSelectorWheel.setSelectedItemMargin(0, 0);
    languageSelectorWheel.setDrawableSize(40, 0);
    languageSelectorWheel.setDrawables(languageSelectorWheelListItems, updateItemCallback,
    
                          languageSelectorWheelSelectedListItems, updateItemCallback);
    languageSelectorWheel.animateToItem(0, 0);
    add(languageSelectorWheel);

    headline.setPosition(50, 13, 220, 29);
    headline.setColor(touchgfx::Color::getColorFromRGB(16, 141, 171));
    headline.setLinespacing(0);
    headline.setTypedText(touchgfx::TypedText(T_LANGUAGEHEADLINE));
    add(headline);

    returnButton.setXY(0, 0);
    returnButton.setBitmaps(touchgfx::Bitmap(BITMAP_RETURNICON_ID), touchgfx::Bitmap(BITMAP_RETURNICON_ID));
    returnButton.setAction(buttonCallback);
    add(returnButton);
}

ListViewBase::~ListViewBase()
{

}

void ListViewBase::setupScreen()
{
    languageSelectorWheel.initialize();
    for (int i = 0; i < languageSelectorWheelListItems.getNumberOfDrawables(); i++)
    {
        languageSelectorWheelListItems[i].initialize();
    }
    for (int i = 0; i < languageSelectorWheelSelectedListItems.getNumberOfDrawables(); i++)
    {
        languageSelectorWheelSelectedListItems[i].initialize();
    }
}

void ListViewBase::buttonCallbackHandler(const touchgfx::AbstractButton& src)
{
    if (&src == &returnButton)
    {
        //BackToMainScreen
        //When returnButton clicked change screen to Screen1
        //Go to Screen1 with no screen transition
        application().gotoScreen1ScreenNoTransition();
    }
}

void ListViewBase::goToScreen1()
{
    //SwitchScreenToScreen1
    //When goToScreen1 is called change screen to Screen1
    //Go to Screen1 with no screen transition
    application().gotoScreen1ScreenNoTransition();
}

void ListViewBase::updateItemCallbackHandler(touchgfx::DrawableListItemsInterface* items, int16_t containerIndex, int16_t itemIndex)
{
    if (items == &languageSelectorWheelListItems)
    {
        touchgfx::Drawable* d = items->getDrawable(containerIndex);
        languageSelectorElement* cc = (languageSelectorElement*)d;
        languageSelectorWheelUpdateItem(*cc, itemIndex);
    }
    if (items == &languageSelectorWheelSelectedListItems)
    {
        touchgfx::Drawable* d = items->getDrawable(containerIndex);
        languageSelectorElementCenter* cc = (languageSelectorElementCenter*)d;
        languageSelectorWheelUpdateCenterItem(*cc, itemIndex);
    }
}