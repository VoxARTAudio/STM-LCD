#ifndef LANGUAGESELECTORELEMENT_HPP
#define LANGUAGESELECTORELEMENT_HPP

#include <gui_generated/containers/languageSelectorElementBase.hpp>
#include <BitmapDatabase.hpp>
#include <texts/TextKeysAndLanguages.hpp>

class languageSelectorElement : public languageSelectorElementBase
{
public:
    languageSelectorElement();
    void setupElement(BitmapId bmpID, TEXTS text);
};

#endif // LANGUAGESELECTORELEMENT_HPP
