#ifndef LANGUAGESELECTORELEMENTCENTER_HPP
#define LANGUAGESELECTORELEMENTCENTER_HPP

#include <gui_generated/containers/languageSelectorElementCenterBase.hpp>
#include <BitmapDatabase.hpp>
#include <texts/TextKeysAndLanguages.hpp>

class languageSelectorElementCenter : public languageSelectorElementCenterBase
{
public:
    languageSelectorElementCenter();
    void setupElement(BitmapId bmpID, TEXTS text);

};

#endif // LANGUAGESELECTORELEMENTCENTER_HPP
