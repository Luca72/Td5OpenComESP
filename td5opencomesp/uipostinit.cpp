#include "uipostinit.h"

#include "td5opencomesp.h"
#include "td5comm.h"

lv_obj_t * ui_TableEcuFaultCodes = NULL;

// POST INIT CODE
void ui_post_init(void)
{
  // SET DARK THEME
  lv_theme_t* th = lv_theme_default_init(lv_disp_get_default(), lv_color_hex(0x2e7d32), lv_color_hex(0xff0000), true, LV_FONT_DEFAULT);
  lv_disp_set_theme(lv_disp_get_default(), th);  
  // SET TOP LAYER OBJECTS
  lv_obj_set_parent(ui_PanelBar, lv_layer_top());
  lv_obj_set_parent(ui_PanelButtonBar, lv_layer_top());
  // STARTUP SCREEN
  char text[40];
  sprintf(text, "Td5OpenCOM v%d.%d.%d", TD5_OPEN_COM_VERSION_MAJOR, TD5_OPEN_COM_VERSION_MINOR, TD5_OPEN_COM_VERSION_PATCH);
  lv_label_set_text(ui_LabelPanelTitle, text);
  lv_label_set_text(ui_LabelStartupProject, text);
  sprintf(text, "DRAM memory free: %d Kb", ESP.getFreeHeap()/1024);
  lv_label_set_text(ui_LabelStartupDRAM, text);
  sprintf(text, "PSRAM memory free: %d Kb", ESP.getMaxAllocPsram()/1024);
  lv_label_set_text(ui_LabelStartupPSRAM, text);
  // ECU SCREEN
  ui_TableEcuFaultCodes = lv_table_create(ui_PanelEcuFaultCodes);
  static lv_style_t style;
  lv_style_init(&style);
  lv_style_set_bg_color(&style, lv_color_hex(0xFFFFFF));
  lv_style_set_text_color(&style, lv_color_hex(0x000000));
  lv_obj_add_style(ui_TableEcuFaultCodes, &style, LV_PART_MAIN | LV_PART_ITEMS);  
  lv_obj_set_width(ui_TableEcuFaultCodes, UI_TABLE_ECU_FAULT_CODES_WIDTH);
  lv_obj_set_height(ui_TableEcuFaultCodes, UI_TABLE_ECU_FAULT_CODES_HEIGHT);
  lv_obj_set_x(ui_TableEcuFaultCodes, UI_TABLE_ECU_FAULT_CODES_POS_X);
  lv_obj_set_y(ui_TableEcuFaultCodes, UI_TABLE_ECU_FAULT_CODES_POS_Y);  
  lv_obj_set_style_pad_ver(ui_TableEcuFaultCodes, 5, LV_PART_ITEMS);
  lv_table_set_col_width(ui_TableEcuFaultCodes, 0, UI_TABLE_ECU_FAULT_CODES_WIDTH);
  lv_table_set_row_cnt(ui_TableEcuFaultCodes, MAX_FAULT_CODES);
  lv_table_set_col_cnt(ui_TableEcuFaultCodes, 1);
  lv_obj_remove_style(ui_TableEcuFaultCodes, NULL, LV_PART_ITEMS | LV_STATE_PRESSED);
  lv_obj_align(ui_PanelEcuFaultCodes, LV_ALIGN_TOP_LEFT, 0, 0);
  lv_obj_set_style_text_font(ui_TableEcuFaultCodes, &UI_TABLE_ECU_FAULT_CODES_FONT, LV_PART_MAIN | LV_STATE_DEFAULT);
}
