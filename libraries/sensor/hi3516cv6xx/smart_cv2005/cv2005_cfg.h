/*
  Copyright (c), 2001-2024, Shenshu Tech. Co., Ltd.
 */

#ifndef CV2005_CFG_H
#define CV2005_CFG_H

#include "sensor_common.h"

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif

static cis_reg_cfg cv2005_linear_1080P30_10bit[] = {
	{0x301C, 0x9C},   // [19:0]  FRAME_LENGTH        = 0049C(dec:1180)
	{0x301D, 0x04},
	{0x301E, 0x00},
	{0x3020, 0x4E},   // [15:0]  LINE_LENGTH         = 044E(dec:1102)
	{0x3021, 0x04},
	{0x3808, 0x41},   // [7:0]   PLL_MUL_NUM_POST    = 41(dec:65)
	{0x3031, 0x00},   //OB OFF Def
	{0x3204, 0x40},   //BLK LEVEL Def
	{0x359d, 0x01},   //streaking-new
	{0x35b0, 0x50},
	{0x35b1, 0x66},
	{0x3158, 0xFF},   //高温CLP对策
	{0x389D, 0x0A},   //改善高增益横纹，分层
	{0x389C, 0x6A},
	{0x38A0, 0x2B},
	{0x3878, 0x01},
	{0x3879, 0x15},
	{0x356f, 0x02},  //优化高增益竖线
	{0x36d8, 0x0c},
	{0x36d9, 0x0c},
	{0x3274, 0x00},   //OTP-off
	{0x3275, 0x01},
	{0x3109, 0x01},  //separate-gain
	//mipi global timing.
	{0x3420, 0x2f},  //THSPREPARE: 61 ns ; min limit: 45 ; max limt: 92.6923076923077
	{0x3422, 0x87},  //THSZERO   : 174 ns ; min limit: 158 ; max limt: 1000
	{0x3424, 0x3f},  //THSTRAIL  : 82 ns ; min limit: 69 ; max limt: 1000
	{0x3426, 0x57},  //THSEXIT   : 112 ns ; min limit: 100 ; max limt: 1000
	{0x3428, 0x2f},  //TLPX      : 61 ns ; min limit: 50 ; max limt: 1000
	{0x3134, 0x00},
	{0x326E, 0x20},//高温逻辑添加
	{0x3000, 0x00}, //Streaming
};

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif
#endif
