/*
  Copyright (c), 2001-2024, Shenshu Tech. Co., Ltd.
 */

#ifndef CV2005_CMOS_H
#define CV2005_CMOS_H

#include "ot_common_isp.h"
#include "ot_sns_ctrl.h"
#include "securec.h"

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif

/****************************************************************************
 * basic info config                                                        *
 ****************************************************************************/
/* sensor ID & Resolution */
#define CV2005_ID             2005
#define CV2005_WIDTH          1920
#define CV2005_HEIGHT         1080

/* sensor Register Address */
#define CV2005_EXPO_H_ADDR          0x304A
#define CV2005_EXPO_M_ADDR          0x3049
#define CV2005_EXPO_L_ADDR          0x3048
#define CV2005_AGAIN_L_ADDR         0x3118
#define CV2005_DGAIN_H_ADDR         0x311D
#define CV2005_DGAIN_L_ADDR         0x311C
#define CV2005_VMAX_H_ADDR          0x301D
#define CV2005_VMAX_L_ADDR          0x301C

/****************************************************************************
 * i2c bus config                                                           *
 ****************************************************************************/
/* sensor I2C bus config */
#define CV2005_I2C_ADDR    0x6A
#define CV2005_ADDR_BYTE   2
#define CV2005_DATA_BYTE   1

/* common I2C bus config */
#define I2C_DEV_FILE_NUM    16
#define I2C_BUF_NUM         8

/****************************************************************************
 * sensor lines configs                                                     *
 ****************************************************************************/
/* increase line */
#define CV2005_INCREASE_LINES 0  /* make real fps less than stand fps because NVR require */
/* linear mode */
#define CV2005_FULL_LINES_MAX_LINEAR 0x49C * 12 /* calc by std & min fps, 0x49C = 1180 = Min:2.5fps */
#define CV2005_VMAX_VAL_LINEAR 0x49C         /* linear init sequence */
#define CV2005_VMAX_LINEAR     (CV2005_VMAX_VAL_LINEAR + CV2005_INCREASE_LINES)
#define CV2005_FPS_MAX_LINEAR 30
#define CV2005_FPS_MIN_LINEAR 2.5
#define CV2005_WIDTH_LINEAR 1920
#define CV2005_HEIGHT_LINEAR 1080
#define CV2005_MODE_LINEAR 0

/****************************************************************************
 * sensor ae configs                                                        *
 ****************************************************************************/
#define CV2005_AGAIN_MIN    1024
#define CV2005_AGAIN_MAX    262144

#define CV2005_DGAIN_MIN    1024
#define CV2005_DGAIN_MAX    8192

#define ISP_DGAIN_SHIFT           8
#define ISP_DGAIN_TARGET_MIN      1
#define ISP_DGAIN_TARGET_MAX      32
#define ISP_DGAIN_TARGET_WDR_MIN  1
#define ISP_DGAIN_TARGET_WDR_MAX  4
#define INT_TIME_ACCURACY      2
#define AGAIN_ACCURACY         1
#define DGAIN_ACCURACY         1

#define FL_OFFSET_LINEAR       5
/****************************************************************************
 * sensor awb calibrate configs                                             *
 ****************************************************************************/
/* awb static param for Fuji Lens New IR_Cut */
#define CALIBRATE_STATIC_TEMP       5000
#define CALIBRATE_STATIC_WB_R_GAIN  409
#define CALIBRATE_STATIC_WB_GR_GAIN 256
#define CALIBRATE_STATIC_WB_GB_GAIN 256
#define CALIBRATE_STATIC_WB_B_GAIN  452

/* Calibration results for Auto WB Planck */
#define CALIBRATE_AWB_P1 (36)
#define CALIBRATE_AWB_P2 220
#define CALIBRATE_AWB_Q1 0
#define CALIBRATE_AWB_A1 218409
#define CALIBRATE_AWB_B1 128
#define CALIBRATE_AWB_C1 (-167686)

/* Rgain and Bgain of the golden sample */
#define GOLDEN_RGAIN                                  0
#define GOLDEN_BGAIN                                  0

/****************************************************************************
 * sensor other configs                                                     *
 ****************************************************************************/
#define STANDARD_FPS              30
#define FLICKER_FREQ            (50 * 256)  /* light flicker freq: 50Hz, accuracy: 256 */
#define INIT_EXP_DEFAULT_LINEAR   148859
#define MAX_INT_TIME_TARGET       65535
#define AE_COMENSATION_DEFAULT    0x40
/* Black level */
#define BLACK_LEVEL_DEFAULT            0x410
/* DNG */
#define DNG_RAW_FORMAT_BIT_LINEAR         10 /* raw 10 bit */
#define DNG_RAW_FORMAT_WHITE_LEVEL_LINEAR 1023 /* 2^10 - 1 */

/****************************************************************************
 * assist function macros                                                   *
 ****************************************************************************/
#define high_8bits(x)   (((x) & 0xff00) >> 8)
#define low_8bits(x)    ((x) & 0x00ff)
#define lower_4bits(x)  (((x) & 0x000F) << 4)
#define higher_8bits(x) (((x) & 0x0FF0) >> 4)
#define higher_4bits(x) (((x) & 0xFF0000) >> 16)
#ifndef clip3
#define clip3(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))
#endif
#ifndef max
#define max(a, b) (((a) < (b)) ?  (b) : (a))
#endif
#ifndef min
#define min(a, b) (((a) > (b)) ?  (b) : (a))
#endif

/****************************************************************************
 * sensor data type defines                                                 *
 ****************************************************************************/
/* define your sensor modes */
typedef enum {
    CV2005_2M_30FPS_10BIT_LINEAR_MODE = 0,
    CV2005_MODE_MAX
} cv2005_res_mode;

typedef enum {
    EXPO_L_IDX = 0,
    EXPO_M_IDX,
    EXPO_H_IDX,
    AGAIN_L_IDX,
    DGAIN_L_IDX,
    DGAIN_H_IDX,
    VMAX_L_IDX,
    VMAX_H_IDX,
    REG_MAX_IDX
} cv2005_linear_reg_index;

ot_isp_sns_obj *cv2005_get_obj(td_void);

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* End of #ifdef __cplusplus */
#endif /* CV2005_CMOS_H */
