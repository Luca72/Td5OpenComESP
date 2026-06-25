#ifndef UI_POSTINIT_H
#define UI_POSTINIT_H

#include "td5opencomesp.h"
#include "lvgl.h"

#include "src/ui/ui_helpers.h"
#include "src/ui/ui_events.h"

#ifdef __cplusplus
extern "C" {
#endif

// UI POST INIT
void ui_post_init(void);

// ECU SCREEN
extern lv_obj_t * ui_TableEcuFaultCodes;



#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif