/*********************************************************************************/
/********** THIS FILE IS GENERATED BY TOUCHGFX DESIGNER, DO NOT MODIFY ***********/
/*********************************************************************************/
#include <gui_generated/containers/languageSelectorElementCenterBase.hpp>
#include <touchgfx/Color.hpp>
#include <images/BitmapDatabase.hpp>
#include <texts/TextKeysAndLanguages.hpp>

languageSelectorElementCenterBase::languageSelectorElementCenterBase()
{
    setWidth(200);
    setHeight(40);
    background.setPosition(0, 0, 200, 40);
    background.setColor(touchgfx::Color::getColorFromRGB(158, 180, 203));
    add(background);

    flagImage.setXY(13, 9);
    flagImage.setBitmap(touchgfx::Bitmap(BITMAP_DENMARK_FLAG_SMALL_ID));
    add(flagImage);

    languageText.setPosition(52, 7, 135, 28);
    languageText.setColor(touchgfx::Color::getColorFromRGB(44, 69, 75));
    languageText.setLinespacing(0);
    languageText.setTypedText(touchgfx::TypedText(T_LANGUAGE0));
    add(languageText);
}

languageSelectorElementCenterBase::~languageSelectorElementCenterBase()
{

}

void languageSelectorElementCenterBase::initialize()
{

}