/*******************************************************************************
  MPLAB Harmony Graphics Composer Generated Implementation File

  File Name:
    libaria_init.c

  Summary:
    Build-time generated implementation from the MPLAB Harmony
    Graphics Composer.

  Description:
    Build-time generated implementation from the MPLAB Harmony
    Graphics Composer.

    Created with MPLAB Harmony Version 3.0
*******************************************************************************/
// DOM-IGNORE-BEGIN
/*******************************************************************************
* Copyright (C) 2018 Microchip Technology Inc. and its subsidiaries.
*
* Subject to your compliance with these terms, you may use Microchip software
* and any derivatives exclusively with Microchip products. It is your
* responsibility to comply with third party license terms applicable to your
* use of third party software (including open source software) that may
* accompany Microchip software.
*
* THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
* EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
* WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
* PARTICULAR PURPOSE.
*
* IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
* INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
* WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
* BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
* FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
* ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
* THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
*******************************************************************************/
// DOM-IGNORE-END

#include "gfx/libaria/libaria_init.h"

laScheme defaultScheme;
laScheme titleScheme;
laScheme WhiteText;
laScheme WhiteBackground;
laImageWidget* ImageWidget1;
laButtonWidget* SendMessageButton;
laLabelWidget* ReceivedMessages;
laLabelWidget* TextMessage3;
laLabelWidget* TextMessage2;
laLabelWidget* TextMessage1;
laImagePlusWidget* Connected_Image;
laImagePlusWidget* PairedImage;
laImagePlusWidget* No_pair_no_connection_Image;
laImagePlusWidget* ImagePlusWidget1;
laLabelWidget* Title;


static void ScreenCreate_default(laScreen* screen);


int32_t libaria_initialize(void)
{
    laScreen* screen;

    laScheme_Initialize(&defaultScheme, GFX_COLOR_MODE_RGB_565);
    defaultScheme.base = 0xC67A;
    defaultScheme.highlight = 0xC67A;
    defaultScheme.highlightLight = 0xFFFF;
    defaultScheme.shadow = 0x8410;
    defaultScheme.shadowDark = 0x4208;
    defaultScheme.foreground = 0x0;
    defaultScheme.foregroundInactive = 0xD71C;
    defaultScheme.foregroundDisabled = 0x8410;
    defaultScheme.background = 0x1967;
    defaultScheme.backgroundInactive = 0xD71C;
    defaultScheme.backgroundDisabled = 0xC67A;
    defaultScheme.text = 0x0;
    defaultScheme.textHighlight = 0x1F;
    defaultScheme.textHighlightText = 0xFFFF;
    defaultScheme.textInactive = 0xD71C;
    defaultScheme.textDisabled = 0x8C92;

    laScheme_Initialize(&titleScheme, GFX_COLOR_MODE_RGB_565);
    titleScheme.base = 0xC7FF;
    titleScheme.highlight = 0xC67A;
    titleScheme.highlightLight = 0xFFFF;
    titleScheme.shadow = 0x8410;
    titleScheme.shadowDark = 0x4208;
    titleScheme.foreground = 0x0;
    titleScheme.foregroundInactive = 0xD71C;
    titleScheme.foregroundDisabled = 0x8410;
    titleScheme.background = 0xC7FF;
    titleScheme.backgroundInactive = 0xD71C;
    titleScheme.backgroundDisabled = 0xC67A;
    titleScheme.text = 0x0;
    titleScheme.textHighlight = 0x1F;
    titleScheme.textHighlightText = 0xFFFF;
    titleScheme.textInactive = 0xD71C;
    titleScheme.textDisabled = 0x8C92;

    laScheme_Initialize(&WhiteText, GFX_COLOR_MODE_RGB_565);
    WhiteText.base = 0x4;
    WhiteText.highlight = 0xC67A;
    WhiteText.highlightLight = 0xFFFF;
    WhiteText.shadow = 0x8410;
    WhiteText.shadowDark = 0x4208;
    WhiteText.foreground = 0x0;
    WhiteText.foregroundInactive = 0xD71C;
    WhiteText.foregroundDisabled = 0x8410;
    WhiteText.background = 0xFFFF;
    WhiteText.backgroundInactive = 0xD71C;
    WhiteText.backgroundDisabled = 0xC67A;
    WhiteText.text = 0xFFFF;
    WhiteText.textHighlight = 0x1F;
    WhiteText.textHighlightText = 0xFFFF;
    WhiteText.textInactive = 0xD71C;
    WhiteText.textDisabled = 0x8C92;

    laScheme_Initialize(&WhiteBackground, GFX_COLOR_MODE_RGB_565);
    WhiteBackground.base = 0xFFFF;
    WhiteBackground.highlight = 0xC67A;
    WhiteBackground.highlightLight = 0xFFFF;
    WhiteBackground.shadow = 0x8410;
    WhiteBackground.shadowDark = 0x4208;
    WhiteBackground.foreground = 0x0;
    WhiteBackground.foregroundInactive = 0xD71C;
    WhiteBackground.foregroundDisabled = 0x8410;
    WhiteBackground.background = 0xFFFF;
    WhiteBackground.backgroundInactive = 0xD71C;
    WhiteBackground.backgroundDisabled = 0xC67A;
    WhiteBackground.text = 0x0;
    WhiteBackground.textHighlight = 0x1F;
    WhiteBackground.textHighlightText = 0xFFFF;
    WhiteBackground.textInactive = 0xD71C;
    WhiteBackground.textDisabled = 0x8C92;

    GFX_Set(GFXF_DRAW_PIPELINE_MODE, GFX_PIPELINE_GCU);
    laContext_SetStringTable(&stringTable);

    screen = laScreen_New(LA_FALSE, LA_FALSE, &ScreenCreate_default);
    laContext_AddScreen(screen);

    laContext_SetActiveScreen(0);

	return 0;
}

static void ScreenCreate_default(laScreen* screen)
{
    laLayer* layer0;

    layer0 = laLayer_New();
    laWidget_SetPosition((laWidget*)layer0, 0, 0);
    laWidget_SetSize((laWidget*)layer0, 480, 272);
    laWidget_SetBackgroundType((laWidget*)layer0, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetScheme((laWidget*)layer0, &defaultScheme);
    laLayer_SetBufferCount(layer0, 1);
    laLayer_SetVSync(layer0, LA_FALSE);

    laScreen_SetLayer(screen, 0, layer0);

    ImageWidget1 = laImageWidget_New();
    laWidget_SetPosition((laWidget*)ImageWidget1, 0, 1);
    laWidget_SetSize((laWidget*)ImageWidget1, 480, 272);
    laWidget_SetScheme((laWidget*)ImageWidget1, &defaultScheme);
    laWidget_SetBackgroundType((laWidget*)ImageWidget1, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)ImageWidget1, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(ImageWidget1, &blue_bg480x272);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)ImageWidget1);

    SendMessageButton = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)SendMessageButton, 170, 80);
    laWidget_SetSize((laWidget*)SendMessageButton, 151, 30);
    laWidget_SetVisible((laWidget*)SendMessageButton, LA_FALSE);
    laWidget_SetScheme((laWidget*)SendMessageButton, &defaultScheme);
    laWidget_SetBackgroundType((laWidget*)SendMessageButton, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)SendMessageButton, LA_WIDGET_BORDER_BEVEL);
    laButtonWidget_SetPressedImage(SendMessageButton, &sendMessage150x30);
    laButtonWidget_SetReleasedImage(SendMessageButton, &sendMessage150x30);
    laButtonWidget_SetPressedOffset(SendMessageButton, 2);
    laButtonWidget_SetPressedEventCallback(SendMessageButton, &SendMessageButton_PressedEvent);
    laButtonWidget_SetReleasedEventCallback(SendMessageButton, &SendMessageButton_ReleasedEvent);

    laWidget_AddChild((laWidget*)layer0, (laWidget*)SendMessageButton);

    ReceivedMessages = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)ReceivedMessages, 18, 123);
    laWidget_SetSize((laWidget*)ReceivedMessages, 193, 25);
    laWidget_SetVisible((laWidget*)ReceivedMessages, LA_FALSE);
    laWidget_SetScheme((laWidget*)ReceivedMessages, &WhiteText);
    laWidget_SetBackgroundType((laWidget*)ReceivedMessages, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ReceivedMessages, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(ReceivedMessages, laString_CreateFromID(string_ReceivedMessages));
    laLabelWidget_SetHAlignment(ReceivedMessages, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)ReceivedMessages);

    TextMessage3 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)TextMessage3, 73, 159);
    laWidget_SetSize((laWidget*)TextMessage3, 347, 25);
    laWidget_SetScheme((laWidget*)TextMessage3, &WhiteText);
    laWidget_SetBackgroundType((laWidget*)TextMessage3, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)TextMessage3, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetHAlignment(TextMessage3, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)TextMessage3);

    TextMessage2 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)TextMessage2, 73, 195);
    laWidget_SetSize((laWidget*)TextMessage2, 347, 25);
    laWidget_SetScheme((laWidget*)TextMessage2, &WhiteText);
    laWidget_SetBackgroundType((laWidget*)TextMessage2, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)TextMessage2, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetHAlignment(TextMessage2, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)TextMessage2);

    TextMessage1 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)TextMessage1, 73, 231);
    laWidget_SetSize((laWidget*)TextMessage1, 347, 25);
    laWidget_SetScheme((laWidget*)TextMessage1, &WhiteText);
    laWidget_SetBackgroundType((laWidget*)TextMessage1, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)TextMessage1, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetHAlignment(TextMessage1, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)TextMessage1);

    Connected_Image = laImagePlusWidget_New();
    laWidget_SetPosition((laWidget*)Connected_Image, 423, 0);
    laWidget_SetSize((laWidget*)Connected_Image, 57, 57);
    laWidget_SetVisible((laWidget*)Connected_Image, LA_FALSE);
    laWidget_SetScheme((laWidget*)Connected_Image, &defaultScheme);
    laWidget_SetBackgroundType((laWidget*)Connected_Image, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)Connected_Image, LA_WIDGET_BORDER_NONE);
    laImagePlusWidget_SetImage(Connected_Image, &CONNECTED);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)Connected_Image);

    PairedImage = laImagePlusWidget_New();
    laWidget_SetPosition((laWidget*)PairedImage, 423, 0);
    laWidget_SetSize((laWidget*)PairedImage, 57, 57);
    laWidget_SetVisible((laWidget*)PairedImage, LA_FALSE);
    laWidget_SetScheme((laWidget*)PairedImage, &defaultScheme);
    laWidget_SetBackgroundType((laWidget*)PairedImage, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)PairedImage, LA_WIDGET_BORDER_NONE);
    laImagePlusWidget_SetImage(PairedImage, &PAIRED);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)PairedImage);

    No_pair_no_connection_Image = laImagePlusWidget_New();
    laWidget_SetPosition((laWidget*)No_pair_no_connection_Image, 423, 0);
    laWidget_SetSize((laWidget*)No_pair_no_connection_Image, 57, 57);
    laWidget_SetScheme((laWidget*)No_pair_no_connection_Image, &defaultScheme);
    laWidget_SetBackgroundType((laWidget*)No_pair_no_connection_Image, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)No_pair_no_connection_Image, LA_WIDGET_BORDER_NONE);
    laImagePlusWidget_SetImage(No_pair_no_connection_Image, &NO_PAIR_NO_CONNECTION);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)No_pair_no_connection_Image);

    ImagePlusWidget1 = laImagePlusWidget_New();
    laWidget_SetSize((laWidget*)ImagePlusWidget1, 103, 24);
    laWidget_SetScheme((laWidget*)ImagePlusWidget1, &defaultScheme);
    laWidget_SetBackgroundType((laWidget*)ImagePlusWidget1, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ImagePlusWidget1, LA_WIDGET_BORDER_NONE);
    laImagePlusWidget_SetImage(ImagePlusWidget1, &microchip_logo103x24);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)ImagePlusWidget1);

    Title = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)Title, 97, 36);
    laWidget_SetSize((laWidget*)Title, 295, 25);
    laWidget_SetScheme((laWidget*)Title, &titleScheme);
    laWidget_SetBackgroundType((laWidget*)Title, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)Title, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(Title, laString_CreateFromID(string_BLECommDemo));
    laLabelWidget_SetHAlignment(Title, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)Title);

}



