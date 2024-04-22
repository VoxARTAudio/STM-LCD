/* DO NOT EDIT THIS FILE */
/* This file is autogenerated by the text-database code generator */

#include <fonts/ApplicationFontProvider.hpp>
#include <fonts/GeneratedFont.hpp>
#include <texts/TypedTextDatabase.hpp>

touchgfx::Font* ApplicationFontProvider::getFont(touchgfx::FontId typography)
{
    switch (typography)
    {
    case Typography::SMALL_C:
        // timesbi_13_4bpp
        return const_cast<touchgfx::Font*>(TypedTextDatabase::getFonts()[0]);
    case Typography::SMALL_C_AUTO_GENERATED_FOR_KR:
        // NanumGothic_13_4bpp
        return const_cast<touchgfx::Font*>(TypedTextDatabase::getFonts()[1]);
    case Typography::SMALL_C_AUTO_GENERATED_FOR_CN:
        // chinese_msyh_13_4bpp
        return const_cast<touchgfx::Font*>(TypedTextDatabase::getFonts()[2]);
    case Typography::SMALL_C_AUTO_GENERATED_FOR_JP:
        // chinese_msyh_13_4bpp
        return const_cast<touchgfx::Font*>(TypedTextDatabase::getFonts()[2]);
    case Typography::SMALL_C_AUTO_GENERATED_FOR_TN:
        // Amiri_Regular_13_4bpp
        return const_cast<touchgfx::Font*>(TypedTextDatabase::getFonts()[3]);
    case Typography::TINY:
        // timesbi_13_4bpp
        return const_cast<touchgfx::Font*>(TypedTextDatabase::getFonts()[0]);
    case Typography::SMALL_B:
        // timesbi_20_4bpp
        return const_cast<touchgfx::Font*>(TypedTextDatabase::getFonts()[4]);
    case Typography::SMALL_B_AUTO_GENERATED_FOR_KR:
        // NanumGothic_20_4bpp
        return const_cast<touchgfx::Font*>(TypedTextDatabase::getFonts()[5]);
    case Typography::SMALL_B_AUTO_GENERATED_FOR_CN:
        // chinese_msyh_20_4bpp
        return const_cast<touchgfx::Font*>(TypedTextDatabase::getFonts()[6]);
    case Typography::SMALL_B_AUTO_GENERATED_FOR_JP:
        // chinese_msyh_20_4bpp
        return const_cast<touchgfx::Font*>(TypedTextDatabase::getFonts()[6]);
    case Typography::SMALL_B_AUTO_GENERATED_FOR_TN:
        // Amiri_Regular_18_2bpp
        return const_cast<touchgfx::Font*>(TypedTextDatabase::getFonts()[7]);
    case Typography::KOREANTINY:
        // NanumGothic_13_4bpp
        return const_cast<touchgfx::Font*>(TypedTextDatabase::getFonts()[1]);
    case Typography::JP_CN_TINY:
        // chinese_msyh_13_4bpp
        return const_cast<touchgfx::Font*>(TypedTextDatabase::getFonts()[2]);
    case Typography::ARABICTINY:
        // Amiri_Regular_16_4bpp
        return const_cast<touchgfx::Font*>(TypedTextDatabase::getFonts()[8]);
    default:
        return 0;
    }
}