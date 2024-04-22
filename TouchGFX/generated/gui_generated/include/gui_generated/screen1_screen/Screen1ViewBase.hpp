/*********************************************************************************/
/********** THIS FILE IS GENERATED BY TOUCHGFX DESIGNER, DO NOT MODIFY ***********/
/*********************************************************************************/
#ifndef SCREEN1VIEWBASE_HPP
#define SCREEN1VIEWBASE_HPP

#include <gui/common/FrontendApplication.hpp>
#include <mvp/View.hpp>
#include <gui/screen1_screen/Screen1Presenter.hpp>
#include <touchgfx/widgets/Box.hpp>
#include <touchgfx/containers/Container.hpp>
#include <touchgfx/widgets/Image.hpp>
#include <touchgfx/widgets/Button.hpp>
#include <touchgfx/widgets/TextArea.hpp>

class Screen1ViewBase : public touchgfx::View<Screen1Presenter>
{
public:
    Screen1ViewBase();
    virtual ~Screen1ViewBase();
    virtual void setupScreen();

    /*
     * Custom Actions
     */
    virtual void goToInfoScreen();
    virtual void goToImageScreen();
    virtual void goToListScreen();
    virtual void goToGraphScreen();

    /*
     * Virtual Action Handlers
     */
    virtual void menuRightButtonClicked()
    {
        // Override and implement this function in Screen1
    }
    virtual void menuLeftButtonClicked()
    {
        // Override and implement this function in Screen1
    }
    virtual void menuHiddenButtonClicked()
    {
        // Override and implement this function in Screen1
    }

protected:
    FrontendApplication& application() {
        return *static_cast<FrontendApplication*>(touchgfx::Application::getInstance());
    }

    /*
     * Member Declarations
     */
    touchgfx::Box __background;
    touchgfx::Box background;
    touchgfx::Container logoTextContainer;
    touchgfx::Image logoText;
    touchgfx::Image logoX;
    touchgfx::Box seperatorLine;
    touchgfx::Image menuIcon0;
    touchgfx::Button menuLeftArrow;
    touchgfx::Button menuRightArrow;
    touchgfx::Button menuHiddenButton;
    touchgfx::TextArea headline;
    touchgfx::Box headlineSeperatorLine;
    touchgfx::Image subtitle;

private:

    /*
     * Callback Declarations
     */
    touchgfx::Callback<Screen1ViewBase, const touchgfx::AbstractButton&> buttonCallback;

    /*
     * Callback Handler Declarations
     */
    void buttonCallbackHandler(const touchgfx::AbstractButton& src);

};

#endif // SCREEN1VIEWBASE_HPP