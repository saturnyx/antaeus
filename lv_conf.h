#ifndef LV_CONF_H
#define LV_CONF_H

/* Color depth: 32-bit (ARGB8888) matches vexDisplayCopyRect format */
#define LV_COLOR_DEPTH 32

/* Display resolution */
#define LV_HOR_RES_MAX 480
#define LV_VER_RES_MAX 240

/* Memory settings - V5 Brain has ~512KB available user RAM */
#define LV_MEM_CUSTOM 0
#define LV_MEM_SIZE (48U * 1024U)  /* 48KB for LVGL heap */

/* Disable features you don't need to save flash/RAM */
#define LV_USE_GPU 0
#define LV_USE_FILESYSTEM 0
#define LV_USE_LOG 0

/* Enable core widgets */
#define LV_USE_LABEL 1
#define LV_USE_BTN 1
#define LV_USE_BTNMATRIX 1
#define LV_USE_SLIDER 1
#define LV_USE_BAR 1
#define LV_USE_ARC 1
#define LV_USE_LINE 1
#define LV_USE_IMG 1

#endif /* LV_CONF_H */
