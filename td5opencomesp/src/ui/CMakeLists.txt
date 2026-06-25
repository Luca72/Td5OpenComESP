SET(SOURCES screens/ui_ScreenStartup.c
    screens/ui_ScreenInstrument.c
    screens/ui_ScreenFuelling.c
    screens/ui_ScreenEcu.c
    screens/ui_ScreenTest.c
    screens/ui_ScreenPreferences.c
    ui.c
    components/ui_comp_hook.c
    ui_helpers.c
    images/ui_img_espp4_icons_alloff_png.c
    images/ui_img_espp4_icon_obd_on_png.c
    images/ui_img_espp4_icon_sd_on_png.c
    images/ui_img_espp4_icon_sd_rec_png.c
    images/ui_img_espp4_icon_wifi_on_png.c
    images/ui_img_panelinstrumentgaugesbkg_png.c
    images/ui_img_panelinstrumentgaugesneedlelarge_png.c
    images/ui_img_panelinstrumentgaugesneedlelargegray_png.c
    images/ui_img_panelinstrumentgaugesneedlemedium_png.c
    images/ui_img_panelinstrumentgaugesneedlemediumyellow_png.c
    fonts/ui_font_FontDigit58Num.c
    fonts/ui_font_Tahoma80Num.c
    fonts/ui_font_Tahoma96Num.c)

add_library(ui ${SOURCES})
