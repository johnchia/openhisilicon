/*
  Copyright (c), 2001-2024, Shenshu Tech. Co., Ltd.
 */

#include <stdio.h>
#include "sensor_common.h"
#include "ot_mpi_isp.h"
#include "ot_mpi_ae.h"
#include "ot_mpi_awb.h"
#include "cv2005_cmos.h"
#include "cv2005_cmos_param.h"
#include "cv2005_sensor_ctrl.h"

/****************************************************************************
 * global variables                                                            *
 ****************************************************************************/
static cis_info g_cv2005_info[OT_ISP_MAX_PIPE_NUM] = {
    [0 ...(OT_ISP_MAX_PIPE_NUM - 1)] {
        .mutex = PTHREAD_MUTEX_INITIALIZER,
        .sns_id = CV2005_ID,
        .fswdr_mode = OT_ISP_FSWDR_NORMAL_MODE,
        .quick_start_en = TD_FALSE,
        .bus_info = { .i2c_dev = -1 },
        .sns_state = TD_NULL,
        .blc_clamp_info = TD_TRUE,
        .mode_tbl = {
            {
                CV2005_VMAX_LINEAR,
                CV2005_FULL_LINES_MAX_LINEAR,
                CV2005_FPS_MAX_LINEAR,
                CV2005_FPS_MIN_LINEAR,
                CV2005_WIDTH_LINEAR,
                CV2005_HEIGHT_LINEAR,
                CV2005_MODE_LINEAR,
                OT_WDR_MODE_NONE,
                "CV2005_2M_30FPS_10BIT_LINEAR_MODE"
             },
        },
        .i2c.fd = -1,
    }
};

/****************************************************************************
  Again & Dgain table for TABLE Mode                                        *
 ****************************************************************************/
struct again_lut {
        unsigned int index;
        unsigned int sensor_again_value;
        unsigned int sensor_dgain_value_low;
        unsigned int sensor_dgain_value_high;
        unsigned int gain; //isp gain arguments to sensor
};

struct again_lut cv2005_again_lut[] = {
    //for ��˼,���ƣ����ܵ�ƽ̨��ISP gain����ֵ1024����sensor X1��
    //index, sensor_again_value, sensor_dgain_value_low, sensor_dgain_value_high, isp_gain_arguments //comment is sensor totaol gain
    //sensor Again Start....
    { 0, 0x0, 0x40, 0x0, 1024 },//1.0
    { 1, 0x1, 0x40, 0x0, 1028 },//1.004
    { 2, 0x2, 0x40, 0x0, 1032 },//1.008
    { 3, 0x3, 0x40, 0x0, 1036 },//1.012
    { 4, 0x4, 0x40, 0x0, 1040 },//1.016
    { 5, 0x5, 0x40, 0x0, 1044 },//1.02
    { 6, 0x6, 0x40, 0x0, 1048 },//1.024
    { 7, 0x7, 0x40, 0x0, 1052 },//1.028
    { 8, 0x8, 0x40, 0x0, 1057 },//1.032
    { 9, 0x9, 0x40, 0x0, 1061 },//1.036
    { 10, 0xa, 0x40, 0x0, 1065 },//1.041
    { 11, 0xb, 0x40, 0x0, 1069 },//1.045
    { 12, 0xc, 0x40, 0x0, 1074 },//1.049
    { 13, 0xd, 0x40, 0x0, 1078 },//1.053
    { 14, 0xe, 0x40, 0x0, 1083 },//1.058
    { 15, 0xf, 0x40, 0x0, 1087 },//1.062
    { 16, 0x10, 0x40, 0x0, 1092 },//1.067
    { 17, 0x11, 0x40, 0x0, 1096 },//1.071
    { 18, 0x12, 0x40, 0x0, 1101 },//1.076
    { 19, 0x13, 0x40, 0x0, 1106 },//1.08
    { 20, 0x14, 0x40, 0x0, 1110 },//1.085
    { 21, 0x15, 0x40, 0x0, 1115 },//1.089
    { 22, 0x16, 0x40, 0x0, 1120 },//1.094
    { 23, 0x17, 0x40, 0x0, 1125 },//1.099
    { 24, 0x18, 0x40, 0x0, 1129 },//1.103
    { 25, 0x19, 0x40, 0x0, 1134 },//1.108
    { 26, 0x1a, 0x40, 0x0, 1139 },//1.113
    { 27, 0x1b, 0x40, 0x0, 1144 },//1.118
    { 28, 0x1c, 0x40, 0x0, 1149 },//1.123
    { 29, 0x1d, 0x40, 0x0, 1154 },//1.128
    { 30, 0x1e, 0x40, 0x0, 1159 },//1.133
    { 31, 0x1f, 0x40, 0x0, 1165 },//1.138
    { 32, 0x20, 0x40, 0x0, 1170 },//1.143
    { 33, 0x21, 0x40, 0x0, 1175 },//1.148
    { 34, 0x22, 0x40, 0x0, 1180 },//1.153
    { 35, 0x23, 0x40, 0x0, 1186 },//1.158
    { 36, 0x24, 0x40, 0x0, 1191 },//1.164
    { 37, 0x25, 0x40, 0x0, 1197 },//1.169
    { 38, 0x26, 0x40, 0x0, 1202 },//1.174
    { 39, 0x27, 0x40, 0x0, 1208 },//1.18
    { 40, 0x28, 0x40, 0x0, 1213 },//1.185
    { 41, 0x29, 0x40, 0x0, 1219 },//1.191
    { 42, 0x2a, 0x40, 0x0, 1224 },//1.196
    { 43, 0x2b, 0x40, 0x0, 1230 },//1.202
    { 44, 0x2c, 0x40, 0x0, 1236 },//1.208
    { 45, 0x2d, 0x40, 0x0, 1242 },//1.213
    { 46, 0x2e, 0x40, 0x0, 1248 },//1.219
    { 47, 0x2f, 0x40, 0x0, 1254 },//1.225
    { 48, 0x30, 0x40, 0x0, 1260 },//1.231
    { 49, 0x31, 0x40, 0x0, 1266 },//1.237
    { 50, 0x32, 0x40, 0x0, 1272 },//1.243
    { 51, 0x33, 0x40, 0x0, 1278 },//1.249
    { 52, 0x34, 0x40, 0x0, 1285 },//1.255
    { 53, 0x35, 0x40, 0x0, 1291 },//1.261
    { 54, 0x36, 0x40, 0x0, 1297 },//1.267
    { 55, 0x37, 0x40, 0x0, 1304 },//1.274
    { 56, 0x38, 0x40, 0x0, 1310 },//1.28
    { 57, 0x39, 0x40, 0x0, 1317 },//1.286
    { 58, 0x3a, 0x40, 0x0, 1323 },//1.293
    { 59, 0x3b, 0x40, 0x0, 1330 },//1.299
    { 60, 0x3c, 0x40, 0x0, 1337 },//1.306
    { 61, 0x3d, 0x40, 0x0, 1344 },//1.313
    { 62, 0x3e, 0x40, 0x0, 1351 },//1.32
    { 63, 0x3f, 0x40, 0x0, 1358 },//1.326
    { 64, 0x40, 0x40, 0x0, 1365 },//1.333
    { 65, 0x41, 0x40, 0x0, 1372 },//1.34
    { 66, 0x42, 0x40, 0x0, 1379 },//1.347
    { 67, 0x43, 0x40, 0x0, 1387 },//1.354
    { 68, 0x44, 0x40, 0x0, 1394 },//1.362
    { 69, 0x45, 0x40, 0x0, 1401 },//1.369
    { 70, 0x46, 0x40, 0x0, 1409 },//1.376
    { 71, 0x47, 0x40, 0x0, 1416 },//1.384
    { 72, 0x48, 0x40, 0x0, 1424 },//1.391
    { 73, 0x49, 0x40, 0x0, 1432 },//1.399
    { 74, 0x4a, 0x40, 0x0, 1440 },//1.407
    { 75, 0x4b, 0x40, 0x0, 1448 },//1.414
    { 76, 0x4c, 0x40, 0x0, 1456 },//1.422
    { 77, 0x4d, 0x40, 0x0, 1464 },//1.43
    { 78, 0x4e, 0x40, 0x0, 1472 },//1.438
    { 79, 0x4f, 0x40, 0x0, 1481 },//1.446
    { 80, 0x50, 0x40, 0x0, 1489 },//1.455
    { 81, 0x51, 0x40, 0x0, 1497 },//1.463
    { 82, 0x52, 0x40, 0x0, 1506 },//1.471
    { 83, 0x53, 0x40, 0x0, 1515 },//1.48
    { 84, 0x54, 0x40, 0x0, 1524 },//1.488
    { 85, 0x55, 0x40, 0x0, 1533 },//1.497
    { 86, 0x56, 0x40, 0x0, 1542 },//1.506
    { 87, 0x57, 0x40, 0x0, 1551 },//1.515
    { 88, 0x58, 0x40, 0x0, 1560 },//1.524
    { 89, 0x59, 0x40, 0x0, 1569 },//1.533
    { 90, 0x5a, 0x40, 0x0, 1579 },//1.542
    { 91, 0x5b, 0x40, 0x0, 1588 },//1.552
    { 92, 0x5c, 0x40, 0x0, 1598 },//1.561
    { 93, 0x5d, 0x40, 0x0, 1608 },//1.571
    { 94, 0x5e, 0x40, 0x0, 1618 },//1.58
    { 95, 0x5f, 0x40, 0x0, 1628 },//1.59
    { 96, 0x60, 0x40, 0x0, 1638 },//1.6
    { 97, 0x61, 0x40, 0x0, 1648 },//1.61
    { 98, 0x62, 0x40, 0x0, 1659 },//1.62
    { 99, 0x63, 0x40, 0x0, 1669 },//1.631
    { 100, 0x64, 0x40, 0x0, 1680 },//1.641
    { 101, 0x65, 0x40, 0x0, 1691 },//1.652
    { 102, 0x66, 0x40, 0x0, 1702 },//1.662
    { 103, 0x67, 0x40, 0x0, 1713 },//1.673
    { 104, 0x68, 0x40, 0x0, 1724 },//1.684
    { 105, 0x69, 0x40, 0x0, 1736 },//1.695
    { 106, 0x6a, 0x40, 0x0, 1747 },//1.707
    { 107, 0x6b, 0x40, 0x0, 1759 },//1.718
    { 108, 0x6c, 0x40, 0x0, 1771 },//1.73
    { 109, 0x6d, 0x40, 0x0, 1783 },//1.741
    { 110, 0x6e, 0x40, 0x0, 1795 },//1.753
    { 111, 0x6f, 0x40, 0x0, 1807 },//1.766
    { 112, 0x70, 0x40, 0x0, 1820 },//1.778
    { 113, 0x71, 0x40, 0x0, 1833 },//1.79
    { 114, 0x72, 0x40, 0x0, 1846 },//1.803
    { 115, 0x73, 0x40, 0x0, 1859 },//1.816
    { 116, 0x74, 0x40, 0x0, 1872 },//1.829
    { 117, 0x75, 0x40, 0x0, 1885 },//1.842
    { 118, 0x76, 0x40, 0x0, 1899 },//1.855
    { 119, 0x77, 0x40, 0x0, 1913 },//1.869
    { 120, 0x78, 0x40, 0x0, 1927 },//1.882
    { 121, 0x79, 0x40, 0x0, 1941 },//1.896
    { 122, 0x7a, 0x40, 0x0, 1956 },//1.91
    { 123, 0x7b, 0x40, 0x0, 1971 },//1.925
    { 124, 0x7c, 0x40, 0x0, 1985 },//1.939
    { 125, 0x7d, 0x40, 0x0, 2001 },//1.954
    { 126, 0x7e, 0x40, 0x0, 2016 },//1.969
    { 127, 0x7f, 0x40, 0x0, 2032 },//1.984
    { 128, 0x80, 0x40, 0x0, 2048 },//2.0
    { 129, 0x81, 0x40, 0x0, 2064 },//2.016
    { 130, 0x82, 0x40, 0x0, 2080 },//2.032
    { 131, 0x83, 0x40, 0x0, 2097 },//2.048
    { 132, 0x84, 0x40, 0x0, 2114 },//2.065
    { 133, 0x85, 0x40, 0x0, 2131 },//2.081
    { 134, 0x86, 0x40, 0x0, 2148 },//2.098
    { 135, 0x87, 0x40, 0x0, 2166 },//2.116
    { 136, 0x88, 0x40, 0x0, 2184 },//2.133
    { 137, 0x89, 0x40, 0x0, 2202 },//2.151
    { 138, 0x8a, 0x40, 0x0, 2221 },//2.169
    { 139, 0x8b, 0x40, 0x0, 2240 },//2.188
    { 140, 0x8c, 0x40, 0x0, 2259 },//2.207
    { 141, 0x8d, 0x40, 0x0, 2279 },//2.226
    { 142, 0x8e, 0x40, 0x0, 2299 },//2.246
    { 143, 0x8f, 0x40, 0x0, 2319 },//2.265
    { 144, 0x90, 0x40, 0x0, 2340 },//2.286
    { 145, 0x91, 0x40, 0x0, 2361 },//2.306
    { 146, 0x92, 0x40, 0x0, 2383 },//2.327
    { 147, 0x93, 0x40, 0x0, 2404 },//2.349
    { 148, 0x94, 0x40, 0x0, 2427 },//2.37
    { 149, 0x95, 0x40, 0x0, 2449 },//2.393
    { 150, 0x96, 0x40, 0x0, 2473 },//2.415
    { 151, 0x97, 0x40, 0x0, 2496 },//2.438
    { 152, 0x98, 0x40, 0x0, 2520 },//2.462
    { 153, 0x99, 0x40, 0x0, 2545 },//2.485
    { 154, 0x9a, 0x40, 0x0, 2570 },//2.51
    { 155, 0x9b, 0x40, 0x0, 2595 },//2.535
    { 156, 0x9c, 0x40, 0x0, 2621 },//2.56
    { 157, 0x9d, 0x40, 0x0, 2647 },//2.586
    { 158, 0x9e, 0x40, 0x0, 2674 },//2.612
    { 159, 0x9f, 0x40, 0x0, 2702 },//2.639
    { 160, 0xa0, 0x40, 0x0, 2730 },//2.667
    { 161, 0xa1, 0x40, 0x0, 2759 },//2.695
    { 162, 0xa2, 0x40, 0x0, 2788 },//2.723
    { 163, 0xa3, 0x40, 0x0, 2818 },//2.753
    { 164, 0xa4, 0x40, 0x0, 2849 },//2.783
    { 165, 0xa5, 0x40, 0x0, 2880 },//2.813
    { 166, 0xa6, 0x40, 0x0, 2912 },//2.844
    { 167, 0xa7, 0x40, 0x0, 2945 },//2.876
    { 168, 0xa8, 0x40, 0x0, 2978 },//2.909
    { 169, 0xa9, 0x40, 0x0, 3013 },//2.943
    { 170, 0xaa, 0x40, 0x0, 3048 },//2.977
    { 171, 0xab, 0x40, 0x0, 3084 },//3.012
    { 172, 0xac, 0x40, 0x0, 3120 },//3.048
    { 173, 0xad, 0x40, 0x0, 3158 },//3.084
    { 174, 0xae, 0x40, 0x0, 3196 },//3.122
    { 175, 0xaf, 0x40, 0x0, 3236 },//3.16
    { 176, 0xb0, 0x40, 0x0, 3276 },//3.2
    { 177, 0xb1, 0x40, 0x0, 3318 },//3.241
    { 178, 0xb2, 0x40, 0x0, 3360 },//3.282
    { 179, 0xb3, 0x40, 0x0, 3404 },//3.325
    { 180, 0xb4, 0x40, 0x0, 3449 },//3.368
    { 181, 0xb5, 0x40, 0x0, 3495 },//3.413
    { 182, 0xb6, 0x40, 0x0, 3542 },//3.459
    { 183, 0xb7, 0x40, 0x0, 3591 },//3.507
    { 184, 0xb8, 0x40, 0x0, 3640 },//3.556
    { 185, 0xb9, 0x40, 0x0, 3692 },//3.606
    { 186, 0xba, 0x40, 0x0, 3744 },//3.657
    { 187, 0xbb, 0x40, 0x0, 3799 },//3.71
    { 188, 0xbc, 0x40, 0x0, 3855 },//3.765
    { 189, 0xbd, 0x40, 0x0, 3912 },//3.821
    { 190, 0xbe, 0x40, 0x0, 3971 },//3.879
    { 191, 0xbf, 0x40, 0x0, 4032 },//3.938
    { 192, 0xc0, 0x40, 0x0, 4096 },//4.0
    { 193, 0xc1, 0x40, 0x0, 4161 },//4.063
    { 194, 0xc2, 0x40, 0x0, 4228 },//4.129
    { 195, 0xc3, 0x40, 0x0, 4297 },//4.197
    { 196, 0xc4, 0x40, 0x0, 4369 },//4.267
    { 197, 0xc5, 0x40, 0x0, 4443 },//4.339
    { 198, 0xc6, 0x40, 0x0, 4519 },//4.414
    { 199, 0xc7, 0x40, 0x0, 4599 },//4.491
    { 200, 0xc8, 0x40, 0x0, 4681 },//4.571
    { 201, 0xc9, 0x40, 0x0, 4766 },//4.655
    { 202, 0xca, 0x40, 0x0, 4854 },//4.741
    { 203, 0xcb, 0x40, 0x0, 4946 },//4.83
    { 204, 0xcc, 0x40, 0x0, 5041 },//4.923
    { 205, 0xcd, 0x40, 0x0, 5140 },//5.02
    { 206, 0xce, 0x40, 0x0, 5242 },//5.12
    { 207, 0xcf, 0x40, 0x0, 5349 },//5.224
    { 208, 0xd0, 0x40, 0x0, 5461 },//5.333
    { 209, 0xd1, 0x40, 0x0, 5577 },//5.447
    { 210, 0xd2, 0x40, 0x0, 5698 },//5.565
    { 211, 0xd3, 0x40, 0x0, 5825 },//5.689
    { 212, 0xd4, 0x40, 0x0, 5957 },//5.818
    { 213, 0xd5, 0x40, 0x0, 6096 },//5.953
    { 214, 0xd6, 0x40, 0x0, 6241 },//6.095
    { 215, 0xd7, 0x40, 0x0, 6393 },//6.244
    { 216, 0xd8, 0x40, 0x0, 6553 },//6.4
    { 217, 0xd9, 0x40, 0x0, 6721 },//6.564
    { 218, 0xda, 0x40, 0x0, 6898 },//6.737
    { 219, 0xdb, 0x40, 0x0, 7084 },//6.919
    { 220, 0xdc, 0x40, 0x0, 7281 },//7.111
    { 221, 0xdd, 0x40, 0x0, 7489 },//7.314
    { 222, 0xde, 0x40, 0x0, 7710 },//7.529
    { 223, 0xdf, 0x40, 0x0, 7943 },//7.758
    { 224, 0xe0, 0x40, 0x0, 8192 },//8.0
    { 225, 0xe1, 0x40, 0x0, 8456 },//8.258
    { 226, 0xe2, 0x40, 0x0, 8738 },//8.533
    { 227, 0xe3, 0x40, 0x0, 9039 },//8.828
    { 228, 0xe4, 0x40, 0x0, 9362 },//9.143
    { 229, 0xe5, 0x40, 0x0, 9709 },//9.481
    { 230, 0xe6, 0x40, 0x0, 10082 },//9.846
    { 231, 0xe7, 0x40, 0x0, 10485 },//10.24
    { 232, 0xe8, 0x40, 0x0, 10922 },//10.667
    { 233, 0xe9, 0x40, 0x0, 11397 },//11.13
    { 234, 0xea, 0x40, 0x0, 11915 },//11.636
    { 235, 0xeb, 0x40, 0x0, 12483 },//12.19
    { 236, 0xec, 0x40, 0x0, 13107 },//12.8
    { 237, 0xed, 0x40, 0x0, 13797 },//13.474
    { 238, 0xee, 0x40, 0x0, 14563 },//14.222
    { 239, 0xef, 0x40, 0x0, 15420 },//15.059
    { 240, 0xf0, 0x40, 0x0, 16384 },//16.0
    { 241, 0xf1, 0x40, 0x0, 17476 },//17.067
    { 242, 0xf2, 0x40, 0x0, 18724 },//18.286
    { 243, 0xf3, 0x40, 0x0, 20164 },//19.692
    { 244, 0xf4, 0x40, 0x0, 21845 },//21.333
    { 245, 0xf5, 0x40, 0x0, 23831 },//23.273
    { 246, 0xf6, 0x40, 0x0, 26214 },//25.6
    { 247, 0xf7, 0x40, 0x0, 29127 },//28.444
    { 248, 0xf8, 0x40, 0x0, 32768 },//32.0
    //Sensor Dgain Start... bellowing is Again* Dgain
    { 249, 0xf8, 0x42, 0x0, 33792 },//33.0
    { 250, 0xf8, 0x44, 0x0, 34816 },//34.0
    { 251, 0xf8, 0x46, 0x0, 35840 },//35.0
    { 252, 0xf8, 0x48, 0x0, 36864 },//36.0
    { 253, 0xf8, 0x4a, 0x0, 37888 },//37.0
    { 254, 0xf8, 0x4c, 0x0, 38912 },//38.0
    { 255, 0xf8, 0x4e, 0x0, 39936 },//39.0
    { 256, 0xf8, 0x50, 0x0, 40960 },//40.0
    { 257, 0xf8, 0x52, 0x0, 41984 },//41.0
    { 258, 0xf8, 0x54, 0x0, 43008 },//42.0
    { 259, 0xf8, 0x56, 0x0, 44032 },//43.0
    { 260, 0xf8, 0x58, 0x0, 45056 },//44.0
    { 261, 0xf8, 0x5a, 0x0, 46080 },//45.0
    { 262, 0xf8, 0x5c, 0x0, 47104 },//46.0
    { 263, 0xf8, 0x5e, 0x0, 48128 },//47.0
    { 264, 0xf8, 0x60, 0x0, 49152 },//48.0
    { 265, 0xf8, 0x62, 0x0, 50176 },//49.0
    { 266, 0xf8, 0x64, 0x0, 51200 },//50.0
    { 267, 0xf8, 0x66, 0x0, 52224 },//51.0
    { 268, 0xf8, 0x68, 0x0, 53248 },//52.0
    { 269, 0xf8, 0x6a, 0x0, 54272 },//53.0
    { 270, 0xf8, 0x6c, 0x0, 55296 },//54.0
    { 271, 0xf8, 0x6e, 0x0, 56320 },//55.0
    { 272, 0xf8, 0x70, 0x0, 57344 },//56.0
    { 273, 0xf8, 0x72, 0x0, 58368 },//57.0
    { 274, 0xf8, 0x74, 0x0, 59392 },//58.0
    { 275, 0xf8, 0x76, 0x0, 60416 },//59.0
    { 276, 0xf8, 0x78, 0x0, 61440 },//60.0
    { 277, 0xf8, 0x7a, 0x0, 62464 },//61.0
    { 278, 0xf8, 0x7c, 0x0, 63488 },//62.0
    { 279, 0xf8, 0x7e, 0x0, 64512 },//63.0
    { 280, 0xf8, 0x80, 0x0, 65536 },//64.0
    { 281, 0xf8, 0x82, 0x0, 66560 },//65.0
    { 282, 0xf8, 0x84, 0x0, 67584 },//66.0
    { 283, 0xf8, 0x86, 0x0, 68608 },//67.0
    { 284, 0xf8, 0x88, 0x0, 69632 },//68.0
    { 285, 0xf8, 0x8a, 0x0, 70656 },//69.0
    { 286, 0xf8, 0x8c, 0x0, 71680 },//70.0
    { 287, 0xf8, 0x8e, 0x0, 72704 },//71.0
    { 288, 0xf8, 0x90, 0x0, 73728 },//72.0
    { 289, 0xf8, 0x92, 0x0, 74752 },//73.0
    { 290, 0xf8, 0x94, 0x0, 75776 },//74.0
    { 291, 0xf8, 0x96, 0x0, 76800 },//75.0
    { 292, 0xf8, 0x98, 0x0, 77824 },//76.0
    { 293, 0xf8, 0x9a, 0x0, 78848 },//77.0
    { 294, 0xf8, 0x9c, 0x0, 79872 },//78.0
    { 295, 0xf8, 0x9e, 0x0, 80896 },//79.0
    { 296, 0xf8, 0xa0, 0x0, 81920 },//80.0
    { 297, 0xf8, 0xa2, 0x0, 82944 },//81.0
    { 298, 0xf8, 0xa4, 0x0, 83968 },//82.0
    { 299, 0xf8, 0xa6, 0x0, 84992 },//83.0
    { 300, 0xf8, 0xa8, 0x0, 86016 },//84.0
    { 301, 0xf8, 0xaa, 0x0, 87040 },//85.0
    { 302, 0xf8, 0xac, 0x0, 88064 },//86.0
    { 303, 0xf8, 0xae, 0x0, 89088 },//87.0
    { 304, 0xf8, 0xb0, 0x0, 90112 },//88.0
    { 305, 0xf8, 0xb2, 0x0, 91136 },//89.0
    { 306, 0xf8, 0xb4, 0x0, 92160 },//90.0
    { 307, 0xf8, 0xb6, 0x0, 93184 },//91.0
    { 308, 0xf8, 0xb8, 0x0, 94208 },//92.0
    { 309, 0xf8, 0xba, 0x0, 95232 },//93.0
    { 310, 0xf8, 0xbc, 0x0, 96256 },//94.0
    { 311, 0xf8, 0xbe, 0x0, 97280 },//95.0
    { 312, 0xf8, 0xc0, 0x0, 98304 },//96.0
    { 313, 0xf8, 0xc2, 0x0, 99328 },//97.0
    { 314, 0xf8, 0xc4, 0x0, 100352 },//98.0
    { 315, 0xf8, 0xc6, 0x0, 101376 },//99.0
    { 316, 0xf8, 0xc8, 0x0, 102400 },//100.0
    { 317, 0xf8, 0xca, 0x0, 103424 },//101.0
    { 318, 0xf8, 0xcc, 0x0, 104448 },//102.0
    { 319, 0xf8, 0xce, 0x0, 105472 },//103.0
    { 320, 0xf8, 0xd0, 0x0, 106496 },//104.0
    { 321, 0xf8, 0xd2, 0x0, 107520 },//105.0
    { 322, 0xf8, 0xd4, 0x0, 108544 },//106.0
    { 323, 0xf8, 0xd6, 0x0, 109568 },//107.0
    { 324, 0xf8, 0xd8, 0x0, 110592 },//108.0
    { 325, 0xf8, 0xda, 0x0, 111616 },//109.0
    { 326, 0xf8, 0xdc, 0x0, 112640 },//110.0
    { 327, 0xf8, 0xde, 0x0, 113664 },//111.0
    { 328, 0xf8, 0xe0, 0x0, 114688 },//112.0
    { 329, 0xf8, 0xe2, 0x0, 115712 },//113.0
    { 330, 0xf8, 0xe4, 0x0, 116736 },//114.0
    { 331, 0xf8, 0xe6, 0x0, 117760 },//115.0
    { 332, 0xf8, 0xe8, 0x0, 118784 },//116.0
    { 333, 0xf8, 0xea, 0x0, 119808 },//117.0
    { 334, 0xf8, 0xec, 0x0, 120832 },//118.0
    { 335, 0xf8, 0xee, 0x0, 121856 },//119.0
    { 336, 0xf8, 0xf0, 0x0, 122880 },//120.0
    { 337, 0xf8, 0xf2, 0x0, 123904 },//121.0
    { 338, 0xf8, 0xf4, 0x0, 124928 },//122.0
    { 339, 0xf8, 0xf6, 0x0, 125952 },//123.0
    { 340, 0xf8, 0xf8, 0x0, 126976 },//124.0
    { 341, 0xf8, 0xfa, 0x0, 128000 },//125.0
    { 342, 0xf8, 0xfc, 0x0, 129024 },//126.0
    { 343, 0xf8, 0xfe, 0x0, 130048 },//127.0
    { 344, 0xf8, 0x0, 0x1, 131072 },//128.0
    { 345, 0xf8, 0x2, 0x1, 132096 },//129.0
    { 346, 0xf8, 0x4, 0x1, 133120 },//130.0
    { 347, 0xf8, 0x6, 0x1, 134144 },//131.0
    { 348, 0xf8, 0x8, 0x1, 135168 },//132.0
    { 349, 0xf8, 0xa, 0x1, 136192 },//133.0
    { 350, 0xf8, 0xc, 0x1, 137216 },//134.0
    { 351, 0xf8, 0xe, 0x1, 138240 },//135.0
    { 352, 0xf8, 0x10, 0x1, 139264 },//136.0
    { 353, 0xf8, 0x12, 0x1, 140288 },//137.0
    { 354, 0xf8, 0x14, 0x1, 141312 },//138.0
    { 355, 0xf8, 0x16, 0x1, 142336 },//139.0
    { 356, 0xf8, 0x18, 0x1, 143360 },//140.0
    { 357, 0xf8, 0x1a, 0x1, 144384 },//141.0
    { 358, 0xf8, 0x1c, 0x1, 145408 },//142.0
    { 359, 0xf8, 0x1e, 0x1, 146432 },//143.0
    { 360, 0xf8, 0x20, 0x1, 147456 },//144.0
    { 361, 0xf8, 0x22, 0x1, 148480 },//145.0
    { 362, 0xf8, 0x24, 0x1, 149504 },//146.0
    { 363, 0xf8, 0x26, 0x1, 150528 },//147.0
    { 364, 0xf8, 0x28, 0x1, 151552 },//148.0
    { 365, 0xf8, 0x2a, 0x1, 152576 },//149.0
    { 366, 0xf8, 0x2c, 0x1, 153600 },//150.0
    { 367, 0xf8, 0x2e, 0x1, 154624 },//151.0
    { 368, 0xf8, 0x30, 0x1, 155648 },//152.0
    { 369, 0xf8, 0x32, 0x1, 156672 },//153.0
    { 370, 0xf8, 0x34, 0x1, 157696 },//154.0
    { 371, 0xf8, 0x36, 0x1, 158720 },//155.0
    { 372, 0xf8, 0x38, 0x1, 159744 },//156.0
    { 373, 0xf8, 0x3a, 0x1, 160768 },//157.0
    { 374, 0xf8, 0x3c, 0x1, 161792 },//158.0
    { 375, 0xf8, 0x3e, 0x1, 162816 },//159.0
    { 376, 0xf8, 0x40, 0x1, 163840 },//160.0
    { 377, 0xf8, 0x42, 0x1, 164864 },//161.0
    { 378, 0xf8, 0x44, 0x1, 165888 },//162.0
    { 379, 0xf8, 0x46, 0x1, 166912 },//163.0
    { 380, 0xf8, 0x48, 0x1, 167936 },//164.0
    { 381, 0xf8, 0x4a, 0x1, 168960 },//165.0
    { 382, 0xf8, 0x4c, 0x1, 169984 },//166.0
    { 383, 0xf8, 0x4e, 0x1, 171008 },//167.0
    { 384, 0xf8, 0x50, 0x1, 172032 },//168.0
    { 385, 0xf8, 0x52, 0x1, 173056 },//169.0
    { 386, 0xf8, 0x54, 0x1, 174080 },//170.0
    { 387, 0xf8, 0x56, 0x1, 175104 },//171.0
    { 388, 0xf8, 0x58, 0x1, 176128 },//172.0
    { 389, 0xf8, 0x5a, 0x1, 177152 },//173.0
    { 390, 0xf8, 0x5c, 0x1, 178176 },//174.0
    { 391, 0xf8, 0x5e, 0x1, 179200 },//175.0
    { 392, 0xf8, 0x60, 0x1, 180224 },//176.0
    { 393, 0xf8, 0x62, 0x1, 181248 },//177.0
    { 394, 0xf8, 0x64, 0x1, 182272 },//178.0
    { 395, 0xf8, 0x66, 0x1, 183296 },//179.0
    { 396, 0xf8, 0x68, 0x1, 184320 },//180.0
    { 397, 0xf8, 0x6a, 0x1, 185344 },//181.0
    { 398, 0xf8, 0x6c, 0x1, 186368 },//182.0
    { 399, 0xf8, 0x6e, 0x1, 187392 },//183.0
    { 400, 0xf8, 0x70, 0x1, 188416 },//184.0
    { 401, 0xf8, 0x72, 0x1, 189440 },//185.0
    { 402, 0xf8, 0x74, 0x1, 190464 },//186.0
    { 403, 0xf8, 0x76, 0x1, 191488 },//187.0
    { 404, 0xf8, 0x78, 0x1, 192512 },//188.0
    { 405, 0xf8, 0x7a, 0x1, 193536 },//189.0
    { 406, 0xf8, 0x7c, 0x1, 194560 },//190.0
    { 407, 0xf8, 0x7e, 0x1, 195584 },//191.0
    { 408, 0xf8, 0x80, 0x1, 196608 },//192.0
    { 409, 0xf8, 0x82, 0x1, 197632 },//193.0
    { 410, 0xf8, 0x84, 0x1, 198656 },//194.0
    { 411, 0xf8, 0x86, 0x1, 199680 },//195.0
    { 412, 0xf8, 0x88, 0x1, 200704 },//196.0
    { 413, 0xf8, 0x8a, 0x1, 201728 },//197.0
    { 414, 0xf8, 0x8c, 0x1, 202752 },//198.0
    { 415, 0xf8, 0x8e, 0x1, 203776 },//199.0
    { 416, 0xf8, 0x90, 0x1, 204800 },//200.0
    { 417, 0xf8, 0x92, 0x1, 205824 },//201.0
    { 418, 0xf8, 0x94, 0x1, 206848 },//202.0
    { 419, 0xf8, 0x96, 0x1, 207872 },//203.0
    { 420, 0xf8, 0x98, 0x1, 208896 },//204.0
    { 421, 0xf8, 0x9a, 0x1, 209920 },//205.0
    { 422, 0xf8, 0x9c, 0x1, 210944 },//206.0
    { 423, 0xf8, 0x9e, 0x1, 211968 },//207.0
    { 424, 0xf8, 0xa0, 0x1, 212992 },//208.0
    { 425, 0xf8, 0xa2, 0x1, 214016 },//209.0
    { 426, 0xf8, 0xa4, 0x1, 215040 },//210.0
    { 427, 0xf8, 0xa6, 0x1, 216064 },//211.0
    { 428, 0xf8, 0xa8, 0x1, 217088 },//212.0
    { 429, 0xf8, 0xaa, 0x1, 218112 },//213.0
    { 430, 0xf8, 0xac, 0x1, 219136 },//214.0
    { 431, 0xf8, 0xae, 0x1, 220160 },//215.0
    { 432, 0xf8, 0xb0, 0x1, 221184 },//216.0
    { 433, 0xf8, 0xb2, 0x1, 222208 },//217.0
    { 434, 0xf8, 0xb4, 0x1, 223232 },//218.0
    { 435, 0xf8, 0xb6, 0x1, 224256 },//219.0
    { 436, 0xf8, 0xb8, 0x1, 225280 },//220.0
    { 437, 0xf8, 0xba, 0x1, 226304 },//221.0
    { 438, 0xf8, 0xbc, 0x1, 227328 },//222.0
    { 439, 0xf8, 0xbe, 0x1, 228352 },//223.0
    { 440, 0xf8, 0xc0, 0x1, 229376 },//224.0
    { 441, 0xf8, 0xc2, 0x1, 230400 },//225.0
    { 442, 0xf8, 0xc4, 0x1, 231424 },//226.0
    { 443, 0xf8, 0xc6, 0x1, 232448 },//227.0
    { 444, 0xf8, 0xc8, 0x1, 233472 },//228.0
    { 445, 0xf8, 0xca, 0x1, 234496 },//229.0
    { 446, 0xf8, 0xcc, 0x1, 235520 },//230.0
    { 447, 0xf8, 0xce, 0x1, 236544 },//231.0
    { 448, 0xf8, 0xd0, 0x1, 237568 },//232.0
    { 449, 0xf8, 0xd2, 0x1, 238592 },//233.0
    { 450, 0xf8, 0xd4, 0x1, 239616 },//234.0
    { 451, 0xf8, 0xd6, 0x1, 240640 },//235.0
    { 452, 0xf8, 0xd8, 0x1, 241664 },//236.0
    { 453, 0xf8, 0xda, 0x1, 242688 },//237.0
    { 454, 0xf8, 0xdc, 0x1, 243712 },//238.0
    { 455, 0xf8, 0xde, 0x1, 244736 },//239.0
    { 456, 0xf8, 0xe0, 0x1, 245760 },//240.0
    { 457, 0xf8, 0xe2, 0x1, 246784 },//241.0
    { 458, 0xf8, 0xe4, 0x1, 247808 },//242.0
    { 459, 0xf8, 0xe6, 0x1, 248832 },//243.0
    { 460, 0xf8, 0xe8, 0x1, 249856 },//244.0
    { 461, 0xf8, 0xea, 0x1, 250880 },//245.0
    { 462, 0xf8, 0xec, 0x1, 251904 },//246.0
    { 463, 0xf8, 0xee, 0x1, 252928 },//247.0
    { 464, 0xf8, 0xf0, 0x1, 253952 },//248.0
    { 465, 0xf8, 0xf2, 0x1, 254976 },//249.0
    { 466, 0xf8, 0xf4, 0x1, 256000 },//250.0
    { 467, 0xf8, 0xf6, 0x1, 257024 },//251.0
    { 468, 0xf8, 0xf8, 0x1, 258048 },//252.0
    { 469, 0xf8, 0xfa, 0x1, 259072 },//253.0
    { 470, 0xf8, 0xfc, 0x1, 260096 },//254.0
    { 471, 0xf8, 0xfe, 0x1, 261120 },//255.0
    { 472, 0xf8, 0x0, 0x2, 262144 },//256.0
    //Total length of the array is: 473
};

/****************************************************************************
 * common functions                                                         *
 ****************************************************************************/
static cis_info *cmos_get_info(ot_vi_pipe vi_pipe)
{
    if (vi_pipe < 0 || vi_pipe >= OT_ISP_MAX_PIPE_NUM) {
        return TD_NULL;
    }
    return &g_cv2005_info[vi_pipe];
}

static ot_isp_sns_state *cmos_get_state(ot_vi_pipe vi_pipe)
{
    if (vi_pipe < 0 || vi_pipe >= OT_ISP_MAX_PIPE_NUM) {
        return TD_NULL;
    }
    return cmos_get_info(vi_pipe)->sns_state;
}

static void cmos_err_mode_print(const ot_isp_cmos_sns_image_mode *sns_image_mode, const ot_isp_sns_state *sns_state)
{
    isp_err_trace("Not support! Width:%u, Height:%u, Fps:%f, WDRMode:%d\n",
        (sns_image_mode)->width, (sns_image_mode)->height, (sns_image_mode)->fps, (sns_state)->wdr_mode);
}

static td_void cmos_get_ae_comm_default(cis_info *cis, ot_isp_ae_sensor_default *ae_sns_dft,
    const ot_isp_sns_state *sns_state)
{
    td_float max_fps = STANDARD_FPS; /* maxfps 30 */

    max_fps = cis->mode_tbl[sns_state->img_mode].max_fps;
    if (sns_state->img_mode == CV2005_2M_30FPS_10BIT_LINEAR_MODE) {
        ae_sns_dft->int_time_accu.accu_type = OT_ISP_AE_ACCURACY_LINEAR;
        ae_sns_dft->int_time_accu.accuracy = INT_TIME_ACCURACY;
        ae_sns_dft->int_time_accu.offset = 0; /* 0.386 line */
    } else {
    }

    ae_sns_dft->full_lines_std = sns_state->fl_std;
    ae_sns_dft->flicker_freq = FLICKER_FREQ; /* light flicker freq: 50Hz, accuracy: 256 */
    ae_sns_dft->full_lines_max = CV2005_FULL_LINES_MAX_LINEAR;
    ae_sns_dft->hmax_times = (1000000000) / (CV2005_VMAX_LINEAR * max_fps); /* 1000000000ns, 30fps */

    ae_sns_dft->again_accu.accu_type = OT_ISP_AE_ACCURACY_TABLE;
    ae_sns_dft->again_accu.accuracy  = AGAIN_ACCURACY;

    ae_sns_dft->dgain_accu.accu_type = OT_ISP_AE_ACCURACY_TABLE;
    ae_sns_dft->dgain_accu.accuracy = DGAIN_ACCURACY;

    ae_sns_dft->isp_dgain_shift = ISP_DGAIN_SHIFT;
    ae_sns_dft->min_isp_dgain_target = ISP_DGAIN_TARGET_MIN << ae_sns_dft->isp_dgain_shift; /* min 1 */
    ae_sns_dft->max_isp_dgain_target = ISP_DGAIN_TARGET_MAX << ae_sns_dft->isp_dgain_shift; /* max 32 */
    (td_void)memcpy_s(&ae_sns_dft->piris_attr, sizeof(ot_isp_piris_attr), &g_piris, sizeof(ot_isp_piris_attr));
    if (cis->lines_per500ms == 0) {
        ae_sns_dft->lines_per500ms = CV2005_VMAX_LINEAR * max_fps / 2; /* 30fps, div 2 */
    } else {
        ae_sns_dft->lines_per500ms = cis->lines_per500ms;
    }

    ae_sns_dft->max_iris_fno = OT_ISP_IRIS_F_NO_1_4;
    ae_sns_dft->min_iris_fno = OT_ISP_IRIS_F_NO_5_6;

    ae_sns_dft->ae_route_ex_valid = TD_FALSE;
    ae_sns_dft->ae_route_attr.total_num = 0;
    ae_sns_dft->ae_route_attr_ex.total_num = 0;
    ae_sns_dft->quick_start.quick_start_enable = cis->quick_start_en;
    ae_sns_dft->quick_start.black_frame_num = 0;
    ae_sns_dft->ae_stat_pos = cis->ae_stat_pos; /* 1 use be stat to AE */
    return;
}

static td_void cmos_get_ae_linear_default(cis_info *cis, ot_isp_ae_sensor_default *ae_sns_dft,
    const ot_isp_sns_state *sns_state)
{
    ae_sns_dft->max_again = CV2005_AGAIN_MAX; /* max 32768 */
    ae_sns_dft->min_again = CV2005_AGAIN_MIN;  /* min 1024 */
    ae_sns_dft->max_again_target = ae_sns_dft->max_again;
    ae_sns_dft->min_again_target = ae_sns_dft->min_again;

    ae_sns_dft->max_dgain = CV2005_DGAIN_MAX; /* max 8*1024 = 8192 */
    ae_sns_dft->min_dgain = CV2005_DGAIN_MIN;  /* min 1024 */
    ae_sns_dft->max_dgain_target = ae_sns_dft->max_dgain;
    ae_sns_dft->min_dgain_target = ae_sns_dft->min_dgain;

    ae_sns_dft->ae_compensation = AE_COMENSATION_DEFAULT;
    ae_sns_dft->ae_exp_mode = OT_ISP_AE_EXP_HIGHLIGHT_PRIOR;
    ae_sns_dft->init_exposure = cis->init_exposure ? cis->init_exposure : INIT_EXP_DEFAULT_LINEAR; /* init 148859 */

    ae_sns_dft->max_int_time = sns_state->fl_std - FL_OFFSET_LINEAR;
    ae_sns_dft->min_int_time = 2; /* min int 2 */
    ae_sns_dft->max_int_time_target = MAX_INT_TIME_TARGET; /* max int 65535 */
    ae_sns_dft->min_int_time_target = ae_sns_dft->min_int_time;
    ae_sns_dft->ae_route_ex_valid = cis->ae_route_ex_valid;
    (td_void)memcpy_s(&ae_sns_dft->ae_route_attr, sizeof(ot_isp_ae_route),
                      &cis->init_ae_route,  sizeof(ot_isp_ae_route));
    (td_void)memcpy_s(&ae_sns_dft->ae_route_attr_ex, sizeof(ot_isp_ae_route_ex),
                      &cis->init_ae_route_ex, sizeof(ot_isp_ae_route_ex));
    return;
}

static td_s32 cmos_get_ae_default(ot_vi_pipe vi_pipe, ot_isp_ae_sensor_default *ae_sns_dft)
{
    cis_info *cis = TD_NULL;
    ot_isp_sns_state *sns_state = TD_NULL;

    sns_check_pipe_return(vi_pipe);
    sns_check_pointer_return(ae_sns_dft);

    cis = cmos_get_info(vi_pipe);
    sns_check_pointer_return(cis);

    sns_state = cis->sns_state;

    sns_check_pointer_return(sns_state);

    (td_void)memset_s(&ae_sns_dft->ae_route_attr, sizeof(ot_isp_ae_route), 0, sizeof(ot_isp_ae_route));

    cmos_get_ae_comm_default(cis, ae_sns_dft, sns_state);

    switch (sns_state->wdr_mode) {
        case OT_WDR_MODE_NONE:   /* linear mode */
            cmos_get_ae_linear_default(cis, ae_sns_dft, sns_state);
            break;
        default:
            cmos_get_ae_linear_default(cis, ae_sns_dft, sns_state);
            break;
    }

    return TD_SUCCESS;
}

static td_void cmos_config_vmax(ot_isp_sns_state *sns_state, td_u32 vmax)
{
////    vmax = vmax * 2;

    if (sns_state->wdr_mode == OT_WDR_MODE_NONE) {
        sns_state->regs_info[0].i2c_data[VMAX_L_IDX].data = low_8bits(vmax);
        sns_state->regs_info[0].i2c_data[VMAX_H_IDX].data = high_8bits(vmax);
    }
    return;
}

/* the function of sensor set fps */
static td_void cmos_fps_set(ot_vi_pipe vi_pipe, td_float fps, ot_isp_ae_sensor_default *ae_sns_dft)
{
    td_u32 lines;
    td_u32 lines_max;
    td_u32 vmax;
    td_float max_fps;
    td_float min_fps;
    cis_info *cis = TD_NULL;
    ot_isp_sns_state *sns_state = TD_NULL;

    sns_check_pipe_void_return(vi_pipe);
    sns_check_pointer_void_return(ae_sns_dft);

    cis = cmos_get_info(vi_pipe);
    sns_check_pointer_void_return(cis);

    sns_state = cis->sns_state;
    sns_check_pointer_void_return(sns_state);

    lines = cis->mode_tbl[sns_state->img_mode].ver_lines;
    lines_max = cis->mode_tbl[sns_state->img_mode].max_ver_lines;
    max_fps = cis->mode_tbl[sns_state->img_mode].max_fps;
    min_fps = cis->mode_tbl[sns_state->img_mode].min_fps;

    printf("shaokc lines=%d\n", lines);

    if ((fps > max_fps) || (fps < min_fps)) {
        isp_err_trace("ISP sensor cv2005 Not support Fps: %f\n", fps);
        return;
    }

    vmax = (td_u32)(lines * max_fps / div_0_to_1_float(fps));
    vmax = (vmax > lines_max) ? lines_max : vmax;

    cmos_config_vmax(sns_state, vmax);
    sns_state->fl_std = vmax;
    ae_sns_dft->lines_per500ms = (td_u32)(lines * max_fps / 2); /* 30/2 */
    cis->lines_per500ms = ae_sns_dft->lines_per500ms;

    ae_sns_dft->fps = fps;
    ae_sns_dft->full_lines_std = sns_state->fl_std;
    ae_sns_dft->max_int_time = sns_state->fl_std - FL_OFFSET_LINEAR;

    sns_state->fl[0] = sns_state->fl_std;
    ae_sns_dft->full_lines = sns_state->fl[0];
    ae_sns_dft->hmax_times =
        (td_u32)((1000000000) / (sns_state->fl_std * div_0_to_1_float(fps))); /* 1000000000ns */

    printf("shaokc vmax=%d\n", vmax);

    return;
}

static td_void cmos_slow_framerate_set(ot_vi_pipe vi_pipe, td_u32 full_lines, ot_isp_ae_sensor_default *ae_sns_dft)
{   
    td_u32 vmax;
    cis_info *cis = TD_NULL;
    ot_isp_sns_state *sns_state = TD_NULL;

    sns_check_pipe_void_return(vi_pipe);
    sns_check_pointer_void_return(ae_sns_dft);

    cis = cmos_get_info(vi_pipe);
    sns_check_pointer_void_return(cis);

    sns_state = cis->sns_state;
    sns_check_pointer_void_return(sns_state);

    //vmax = CV2005_VMAX_LINEAR;
    //vmax = CV2005_VMAX_VAL_LINEAR * CV2005_FPS_MAX_LINEAR/CV2005_SLOW_FPS;
    vmax = CV2005_VMAX_VAL_LINEAR;

    vmax = (vmax > CV2005_FULL_LINES_MAX_LINEAR) ? CV2005_FULL_LINES_MAX_LINEAR : vmax;
    sns_state->fl[0] = vmax;

    cmos_config_vmax(sns_state, vmax);

    // sns_state->regs_info[0].i2c_data[VMAX_L_IDX].data = low_8bits(vmax);
    // sns_state->regs_info[0].i2c_data[VMAX_H_IDX].data = high_8bits(vmax);

    ae_sns_dft->max_int_time = sns_state->fl[0] - FL_OFFSET_LINEAR;
    ae_sns_dft->full_lines = full_lines;
    return;
}

static td_void cmos_inttime_update_linear(cis_info *cis, ot_isp_sns_state *sns_state, td_u32 int_time)
{
    td_u32 shutter0;
    shutter0 = sns_state->fl[0] - int_time;
    //shutter0 = shutter0/2 * 2;
    shutter0 = (shutter0 > (sns_state->fl[0] - 1)) ? (sns_state->fl[0] - 1) : (shutter0 < 5) ? 5 : shutter0;		

    sns_state->regs_info[0].i2c_data[EXPO_L_IDX].data = low_8bits(shutter0);
    sns_state->regs_info[0].i2c_data[EXPO_M_IDX].data = high_8bits(shutter0);
    sns_state->regs_info[0].i2c_data[EXPO_H_IDX].data = higher_4bits(shutter0);

    return;
}

/* while isp notify ae to update sensor regs, ae call these funcs. */
static td_void cmos_inttime_update(ot_vi_pipe vi_pipe, td_u32 int_time)
{
    cis_info *cis = TD_NULL;
    ot_isp_sns_state *sns_state = TD_NULL;

    sns_check_pipe_void_return(vi_pipe);

    cis = cmos_get_info(vi_pipe);
    sns_check_pointer_void_return(cis);

    sns_state = cis->sns_state;
    sns_check_pointer_void_return(sns_state);

    cmos_inttime_update_linear(cis, sns_state, int_time);

    return;
}

static td_void cmos_again_calc_table(ot_vi_pipe vi_pipe, td_u32 *again_lin, td_u32 *again_db)
{
    td_u32 i;
    sns_check_pointer_void_return(again_lin);
    sns_check_pointer_void_return(again_db);
    ot_unused(vi_pipe);

    // printf("isp again=%d\n", *again_lin);

    struct again_lut *lut = cv2005_again_lut;

    td_u32 element_count;
    element_count = sizeof(cv2005_again_lut) / sizeof(cv2005_again_lut[0]);

    if (*again_lin >= lut[element_count-1].gain)
    {
        *again_lin =lut[element_count-1].gain;
        *again_db = lut[element_count-1].index;
        return;
    };

    for (i = 1; i <= element_count; i++) //shaokc. intial value of i need to be 0.
    {
        if (*again_lin < lut[i].gain)
        {
            *again_lin = lut[i-1].gain;
            *again_db = i-1;
            break;
        }
    }
    // printf("isp again db=%d\n", *again_db);

    return;
}

static td_void cmos_dgain_calc_table(ot_vi_pipe vi_pipe, td_u32 *dgain_lin, td_u32 *dgain_db)
{
    return;
}

static td_void cmos_gains_update(ot_vi_pipe vi_pipe, td_u32 again, td_u32 dgain)
{
    cis_info *cis = TD_NULL;
    ot_isp_sns_state *sns_state = TD_NULL;
    cis_i2c *i2c = TD_NULL;
    td_u8 reg_0x3118;
    td_u8 reg_0x311C;
    td_u8 reg_0x311D;

    cis = cmos_get_info(vi_pipe);
    sns_check_pointer_void_return(cis);

    i2c = &cis->i2c;
    sns_state = cis->sns_state;
    sns_check_pointer_void_return(sns_state);

    struct again_lut *lut = cv2005_again_lut; 
    td_s32 index = again & 0xffff;

    reg_0x3118 = low_8bits(lut[index].sensor_again_value);
    reg_0x311C = low_8bits(lut[index].sensor_dgain_value_low);
    reg_0x311D = low_8bits(lut[index].sensor_dgain_value_high);

    if (cis->quick_start_en == TD_TRUE && cis->i2c.fd >= 0) {
        cis_write_reg(i2c, CV2005_DGAIN_H_ADDR, reg_0x311D);
        cis_write_reg(i2c, CV2005_DGAIN_L_ADDR, reg_0x311C);
        cis_write_reg(i2c, CV2005_AGAIN_L_ADDR, reg_0x3118);
    } else {
        sns_state->regs_info[0].i2c_data[DGAIN_H_IDX].data = reg_0x311D;
        sns_state->regs_info[0].i2c_data[DGAIN_L_IDX].data = reg_0x311C;
        sns_state->regs_info[0].i2c_data[AGAIN_L_IDX].data = reg_0x3118;
    }

    return;
}

static td_void cmos_get_inttime_max(ot_vi_pipe vi_pipe, td_u16 man_ratio_enable, td_u32 *ratio,
    ot_isp_ae_int_time_range *int_time, td_u32 *lf_max_int_time)
{
    ot_isp_sns_state *sns_state = TD_NULL;
    ot_unused(man_ratio_enable);
    sns_check_pipe_void_return(vi_pipe);
    sns_check_pointer_void_return(ratio);
    sns_check_pointer_void_return(int_time);
    sns_check_pointer_void_return(lf_max_int_time);
    sns_state = cmos_get_state(vi_pipe);
    sns_check_pointer_void_return(sns_state);

    return;
}

/* Only used in LINE_WDR mode */
static td_void cmos_ae_fswdr_attr_set(ot_vi_pipe vi_pipe, ot_isp_ae_fswdr_attr *ae_fswdr_attr)
{
    cis_info *cis = TD_NULL;

    sns_check_pipe_void_return(vi_pipe);
    sns_check_pointer_void_return(ae_fswdr_attr);

    cis = cmos_get_info(vi_pipe);
    sns_check_pointer_void_return(cis);

    cis->fswdr_mode = ae_fswdr_attr->fswdr_mode;
    cis->max_time_get_cnt = 0;

    return;
}

static td_void cmos_ae_quick_start_status_set(ot_vi_pipe vi_pipe, td_bool quick_start_en)
{
    cis_info *cis = TD_NULL;
    ot_isp_sns_state *sns_state = TD_NULL;

    cis = cmos_get_info(vi_pipe);
    sns_check_pointer_void_return(cis);

    sns_state = cis->sns_state;
    sns_check_pointer_void_return(sns_state);

    cis->quick_start_en = quick_start_en;
    sns_state->sync_init = TD_FALSE;
}

static td_void cmos_init_ae_exp_function(ot_isp_ae_sensor_exp_func *exp_func)
{
    (td_void)memset_s(exp_func, sizeof(ot_isp_ae_sensor_exp_func), 0, sizeof(ot_isp_ae_sensor_exp_func));
    exp_func->pfn_cmos_get_ae_default    = cmos_get_ae_default;
    exp_func->pfn_cmos_fps_set           = cmos_fps_set;
    exp_func->pfn_cmos_slow_framerate_set = cmos_slow_framerate_set;
    exp_func->pfn_cmos_inttime_update    = cmos_inttime_update;
    exp_func->pfn_cmos_gains_update      = cmos_gains_update;
    exp_func->pfn_cmos_again_calc_table  = cmos_again_calc_table;
    exp_func->pfn_cmos_dgain_calc_table   = cmos_dgain_calc_table;
    exp_func->pfn_cmos_get_inttime_max   = cmos_get_inttime_max;
    exp_func->pfn_cmos_ae_fswdr_attr_set = cmos_ae_fswdr_attr_set;
    exp_func->pfn_cmos_ae_quick_start_status_set = cmos_ae_quick_start_status_set;

    return;
}

static td_s32 cmos_awb_get_default(ot_vi_pipe vi_pipe, ot_isp_awb_sensor_default *awb_sns_dft)
{
    cis_info *cis = TD_NULL;
    ot_isp_sns_state *sns_state = TD_NULL;

    sns_check_pipe_return(vi_pipe);
    sns_check_pointer_return(awb_sns_dft);

    cis = cmos_get_info(vi_pipe);
    sns_check_pointer_return(cis);

    sns_state = cis->sns_state;
    sns_check_pointer_return(sns_state);

    (td_void)memset_s(awb_sns_dft, sizeof(ot_isp_awb_sensor_default), 0, sizeof(ot_isp_awb_sensor_default));
    awb_sns_dft->wb_ref_temp = CALIBRATE_STATIC_TEMP; /* wb_ref_temp 4950 */

    awb_sns_dft->gain_offset[0] = CALIBRATE_STATIC_WB_R_GAIN;
    awb_sns_dft->gain_offset[1] = CALIBRATE_STATIC_WB_GR_GAIN;
    awb_sns_dft->gain_offset[2] = CALIBRATE_STATIC_WB_GB_GAIN; /* index 2 */
    awb_sns_dft->gain_offset[3] = CALIBRATE_STATIC_WB_B_GAIN; /* index 3 */

    awb_sns_dft->wb_para[0] = CALIBRATE_AWB_P1;
    awb_sns_dft->wb_para[1] = CALIBRATE_AWB_P2;
    awb_sns_dft->wb_para[2] = CALIBRATE_AWB_Q1; /* index 2 */
    awb_sns_dft->wb_para[3] = CALIBRATE_AWB_A1; /* index 3 */
    awb_sns_dft->wb_para[4] = CALIBRATE_AWB_B1; /* index 4 */
    awb_sns_dft->wb_para[5] = CALIBRATE_AWB_C1; /* index 5 */

    awb_sns_dft->golden_rgain = GOLDEN_RGAIN;
    awb_sns_dft->golden_bgain = GOLDEN_BGAIN;

    switch (sns_state->wdr_mode) {
        case OT_WDR_MODE_NONE:
            (td_void)memcpy_s(&awb_sns_dft->ccm, sizeof(ot_isp_awb_ccm), &g_awb_ccm, sizeof(ot_isp_awb_ccm));
            (td_void)memcpy_s(&awb_sns_dft->agc_tbl, sizeof(ot_isp_awb_agc_table),
                              &g_awb_agc_table, sizeof(ot_isp_awb_agc_table));
            break;

        default:
            (td_void)memcpy_s(&awb_sns_dft->ccm, sizeof(ot_isp_awb_ccm), &g_awb_ccm, sizeof(ot_isp_awb_ccm));
            (td_void)memcpy_s(&awb_sns_dft->agc_tbl, sizeof(ot_isp_awb_agc_table),
                              &g_awb_agc_table, sizeof(ot_isp_awb_agc_table));
            break;
    }

    awb_sns_dft->init_rgain = cis->init_wb_r_gain;
    awb_sns_dft->init_ggain = cis->init_wb_g_gain;
    awb_sns_dft->init_bgain = cis->init_wb_b_gain;
    awb_sns_dft->sample_rgain = cis->sample_r_gain;
    awb_sns_dft->sample_bgain = cis->sample_b_gain;

    return TD_SUCCESS;
}

static td_void cmos_init_awb_exp_function(ot_isp_awb_sensor_exp_func *exp_func)
{
    (td_void)memset_s(exp_func, sizeof(ot_isp_awb_sensor_exp_func), 0, sizeof(ot_isp_awb_sensor_exp_func));

    exp_func->pfn_cmos_get_awb_default = cmos_awb_get_default;

    return;
}

static td_void cmos_isp_get_dng_default(const ot_isp_sns_state *sns_state, ot_isp_cmos_default *isp_def)
{
    ot_isp_cmos_dng_color_param dng_color_param = {{ 286, 256, 608 }, { 415, 256, 429 },
        { 2810, { 0x01AC, 0x8093, 0x8019, 0x8070, 0x01EA, 0x807A, 0x802A, 0x80F3, 0x021D }},
        { 4940, { 0x01D7, 0x8084, 0x8053, 0x8053, 0x01D9, 0x8086, 0x8010, 0x80B3, 0x01C3 }}};

    (td_void)memcpy_s(&isp_def->dng_color_param, sizeof(ot_isp_cmos_dng_color_param), &dng_color_param,
                      sizeof(ot_isp_cmos_dng_color_param));

    switch (sns_state->img_mode) {
        case CV2005_2M_30FPS_10BIT_LINEAR_MODE:
            isp_def->sns_mode.dng_raw_format.bits_per_sample = DNG_RAW_FORMAT_BIT_LINEAR; /* 10bit */
            isp_def->sns_mode.dng_raw_format.white_level = DNG_RAW_FORMAT_WHITE_LEVEL_LINEAR; /* max 1023 */
            break;
        default:
            break;
    }
    isp_def->sns_mode.dng_raw_format.default_scale.default_scale_hor.denominator = 1;
    isp_def->sns_mode.dng_raw_format.default_scale.default_scale_hor.numerator = 1;
    isp_def->sns_mode.dng_raw_format.default_scale.default_scale_ver.denominator = 1;
    isp_def->sns_mode.dng_raw_format.default_scale.default_scale_ver.numerator = 1;
    isp_def->sns_mode.dng_raw_format.cfa_repeat_pattern_dim.repeat_pattern_dim_row = 2; /* pattern 2 */
    isp_def->sns_mode.dng_raw_format.cfa_repeat_pattern_dim.repeat_pattern_dim_col = 2; /* pattern 2 */
    isp_def->sns_mode.dng_raw_format.black_level_repeat_dim.repeat_row = 2; /* pattern 2 */
    isp_def->sns_mode.dng_raw_format.black_level_repeat_dim.repeat_col = 2; /* pattern 2 */
    isp_def->sns_mode.dng_raw_format.cfa_layout = OT_ISP_CFALAYOUT_TYPE_RECTANGULAR;
    isp_def->sns_mode.dng_raw_format.cfa_plane_color[0] = 0;
    isp_def->sns_mode.dng_raw_format.cfa_plane_color[1] = 1;
    isp_def->sns_mode.dng_raw_format.cfa_plane_color[2] = 2; /* index 2, cfa_plane_color 2 */
    isp_def->sns_mode.dng_raw_format.cfa_pattern[0] = 0;
    isp_def->sns_mode.dng_raw_format.cfa_pattern[1] = 1;
    isp_def->sns_mode.dng_raw_format.cfa_pattern[2] = 1; /* index 2, cfa_pattern 1 */
    isp_def->sns_mode.dng_raw_format.cfa_pattern[3] = 2; /* index 3, cfa_pattern 2 */
    isp_def->sns_mode.valid_dng_raw_format = TD_TRUE;

    return;
}

static void cmos_isp_get_linear_default(ot_isp_cmos_default *isp_def)
{
    isp_def->key.bit1_demosaic         = 1;
    isp_def->demosaic                  = &g_cmos_demosaic;
    isp_def->key.bit1_sharpen          = 1;
    isp_def->sharpen                   = &g_cmos_yuv_sharpen;
    isp_def->key.bit1_drc              = 1;
    isp_def->drc                       = &g_cmos_drc;
    isp_def->key.bit1_bayer_nr         = 1;
    isp_def->bayer_nr                  = &g_cmos_bayer_nr;
    isp_def->key.bit1_anti_false_color = 1;
    isp_def->anti_false_color          = &g_cmos_anti_false_color;
    isp_def->key.bit1_cac              = 1;
    isp_def->cac                       = &g_cmos_cac;
    isp_def->key.bit1_ldci             = 1;
    isp_def->ldci                      = &g_cmos_ldci;
    isp_def->key.bit1_gamma            = 1;
    isp_def->gamma                     = &g_cmos_gamma;
    isp_def->key.bit1_clut             = 1;
    isp_def->clut                      = &g_cmos_clut;
#ifdef CONFIG_OT_ISP_CR_SUPPORT
    isp_def->key.bit1_ge               = 1;
    isp_def->ge                        = &g_cmos_ge;
#endif
    isp_def->key.bit1_dehaze = 1;
    isp_def->dehaze = &g_cmos_dehaze;
    isp_def->key.bit1_ca = 1;
    isp_def->ca = &g_cmos_ca;
    (td_void)memcpy_s(&isp_def->noise_calibration, sizeof(ot_isp_noise_calibration),
                      &g_cmos_noise_calibration, sizeof(ot_isp_noise_calibration));
    return;
}

static td_s32 cmos_isp_get_default(ot_vi_pipe vi_pipe, ot_isp_cmos_default *isp_def)
{
    ot_isp_sns_state *sns_state = TD_NULL;

    sns_check_pipe_return(vi_pipe);
    sns_check_pointer_return(isp_def);
    sns_state = cmos_get_state(vi_pipe);
    sns_check_pointer_return(sns_state);

    (td_void)memset_s(isp_def, sizeof(ot_isp_cmos_default), 0, sizeof(ot_isp_cmos_default));
#ifdef CONFIG_OT_ISP_CA_SUPPORT
    isp_def->key.bit1_ca      = 1;
    isp_def->ca               = &g_cmos_ca;
#endif
    isp_def->key.bit1_dpc     = 1;
    isp_def->dpc              = &g_cmos_dpc;

    isp_def->key.bit1_wdr     = 1;
    isp_def->wdr              = &g_cmos_wdr;

    isp_def->key.bit1_lsc      = 1;
    isp_def->lsc               = &g_cmos_lsc;

    isp_def->key.bit1_acs      = 0;
    isp_def->acs               = &g_cmos_acs;

#ifdef CONFIG_OT_ISP_PREGAMMA_SUPPORT
    isp_def->key.bit1_pregamma = 0;
    isp_def->pregamma          = &g_cmos_pregamma;
#endif
    switch (sns_state->wdr_mode) {
        case OT_WDR_MODE_NONE:
            cmos_isp_get_linear_default(isp_def);
            break;
        default:
            cmos_isp_get_linear_default(isp_def);
            break;
    }

    isp_def->wdr_switch_attr.exp_ratio[0] = 0x40;

    isp_def->sns_mode.sns_id = CV2005_ID;
    isp_def->sns_mode.sns_mode = sns_state->img_mode;
    cmos_isp_get_dng_default(sns_state, isp_def);

    return TD_SUCCESS;
}

static td_s32 cmos_isp_get_black_level(ot_vi_pipe vi_pipe, ot_isp_cmos_black_level *black_level)
{
    td_s32  i;
    ot_isp_sns_state *sns_state = TD_NULL;

    sns_check_pipe_return(vi_pipe);
    sns_check_pointer_return(black_level);
    sns_state = cmos_get_state(vi_pipe);
    sns_check_pointer_return(sns_state);

    (td_void)memcpy_s(black_level, sizeof(ot_isp_cmos_black_level), &g_cmos_blc, sizeof(ot_isp_cmos_black_level));

    /* Don't need to update black level when iso change */
    black_level->auto_attr.update = TD_FALSE;

    /* black level of linear mode */
    if (sns_state->wdr_mode == OT_WDR_MODE_NONE) {
        for (i = 0; i < OT_ISP_BAYER_CHN_NUM; i++) {
            black_level->auto_attr.black_level[0][i] = BLACK_LEVEL_DEFAULT;
        }
    } else { /* black level of DOL mode */
        for (i = 0; i < OT_ISP_WDR_MAX_FRAME_NUM; i++) {
            black_level->auto_attr.black_level[i][0] = BLACK_LEVEL_DEFAULT;
            black_level->auto_attr.black_level[i][1] = BLACK_LEVEL_DEFAULT;
            black_level->auto_attr.black_level[i][2] = BLACK_LEVEL_DEFAULT; /* index 2 */
            black_level->auto_attr.black_level[i][3] = BLACK_LEVEL_DEFAULT; /* index 3 */
        }
    }

    return TD_SUCCESS;
}

static td_s32 cmos_isp_get_blc_clamp_info(ot_vi_pipe vi_pipe, td_bool *blc_clamp_en)
{
    cis_info *cis = TD_NULL;

    sns_check_pipe_return(vi_pipe);
    sns_check_pointer_return(blc_clamp_en);

    cis = cmos_get_info(vi_pipe);
    sns_check_pointer_return(cis);

    *blc_clamp_en = cis->blc_clamp_info;

    return TD_SUCCESS;
}

static td_s32 cmos_isp_set_wdr_mode(ot_vi_pipe vi_pipe, td_u8 mode)
{
    ot_isp_sns_state *sns_state = TD_NULL;

    sns_check_pipe_return(vi_pipe);

    sns_state = cmos_get_state(vi_pipe);
    sns_check_pointer_return(sns_state);

    sns_state->sync_init = TD_FALSE;

    switch (mode & 0x3F) {
        case OT_WDR_MODE_NONE:
            sns_state->wdr_mode = OT_WDR_MODE_NONE;
            printf("linear mode\n");
            break;

        default:
            isp_err_trace("Not support this mode!\n");
            return TD_FAILURE;
    }

    (td_void)memset_s(sns_state->wdr_int_time, sizeof(sns_state->wdr_int_time), 0, sizeof(sns_state->wdr_int_time));

    return TD_SUCCESS;
}

static td_void cmos_comm_sns_reg_info_init(cis_info *cis, ot_isp_sns_state *sns_state)
{
    td_u32 i;
    sns_state->regs_info[0].sns_type = OT_ISP_SNS_TYPE_I2C;
    sns_state->regs_info[0].com_bus.i2c_dev = cis->bus_info.i2c_dev;
    sns_state->regs_info[0].cfg2_valid_delay_max = 2; /* delay_max 2 */
    sns_state->regs_info[0].reg_num = REG_MAX_IDX; /* reg num 7 */

    for (i = 0; i < sns_state->regs_info[0].reg_num; i++) {
        sns_state->regs_info[0].i2c_data[i].update = TD_TRUE;
        sns_state->regs_info[0].i2c_data[i].dev_addr = CV2005_I2C_ADDR;
        sns_state->regs_info[0].i2c_data[i].addr_byte_num = CV2005_ADDR_BYTE;
        sns_state->regs_info[0].i2c_data[i].data_byte_num = CV2005_DATA_BYTE;
    }

    /* Linear Mode Regs */
    sns_state->regs_info[0].i2c_data[EXPO_L_IDX].delay_frame_num = 2;
    sns_state->regs_info[0].i2c_data[EXPO_L_IDX].reg_addr = CV2005_EXPO_L_ADDR;
    sns_state->regs_info[0].i2c_data[EXPO_M_IDX].delay_frame_num = 2;
    sns_state->regs_info[0].i2c_data[EXPO_M_IDX].reg_addr = CV2005_EXPO_M_ADDR;
    sns_state->regs_info[0].i2c_data[EXPO_H_IDX].delay_frame_num = 2;
    sns_state->regs_info[0].i2c_data[EXPO_H_IDX].reg_addr = CV2005_EXPO_H_ADDR;

    sns_state->regs_info[0].i2c_data[AGAIN_L_IDX].delay_frame_num = 2;
    sns_state->regs_info[0].i2c_data[AGAIN_L_IDX].reg_addr = CV2005_AGAIN_L_ADDR;

    sns_state->regs_info[0].i2c_data[DGAIN_H_IDX].delay_frame_num = 2;
    sns_state->regs_info[0].i2c_data[DGAIN_H_IDX].reg_addr = CV2005_DGAIN_H_ADDR;
    sns_state->regs_info[0].i2c_data[DGAIN_L_IDX].delay_frame_num = 2;
    sns_state->regs_info[0].i2c_data[DGAIN_L_IDX].reg_addr = CV2005_DGAIN_L_ADDR;

    sns_state->regs_info[0].i2c_data[VMAX_L_IDX].delay_frame_num = 2;
    sns_state->regs_info[0].i2c_data[VMAX_L_IDX].reg_addr = CV2005_VMAX_L_ADDR;
    sns_state->regs_info[0].i2c_data[VMAX_H_IDX].delay_frame_num = 2;
    sns_state->regs_info[0].i2c_data[VMAX_H_IDX].reg_addr = CV2005_VMAX_H_ADDR;
    return;
}

static td_void cmos_sns_config_logic_update(ot_vi_pipe vi_pipe, ot_isp_sns_state *sns_state)
{
    ot_unused(vi_pipe);
}

static td_void cmos_sns_reg_info_update(ot_vi_pipe vi_pipe, ot_isp_sns_state *sns_state)
{
    td_u32 i;
    ot_unused(vi_pipe);

    for (i = 0; i < sns_state->regs_info[0].reg_num; i++) {
        if (sns_state->regs_info[0].i2c_data[i].data ==
            sns_state->regs_info[1].i2c_data[i].data) {
            sns_state->regs_info[0].i2c_data[i].update = TD_FALSE;
        } else {
            sns_state->regs_info[0].i2c_data[i].update = TD_TRUE;
        }
    }

    cmos_sns_config_logic_update(vi_pipe, sns_state);

    return;
}

static td_s32 cmos_isp_get_sns_regs_info(ot_vi_pipe vi_pipe, ot_isp_sns_regs_info *sns_regs_info)
{
    cis_info *cis = TD_NULL;
    ot_isp_sns_state *sns_state = TD_NULL;

    sns_check_pipe_return(vi_pipe);
    sns_check_pointer_return(sns_regs_info);

    cis = cmos_get_info(vi_pipe);
    sns_check_pointer_return(cis);

    sns_state = cis->sns_state;
    sns_check_pointer_return(sns_state);

    if ((sns_state->sync_init == TD_FALSE) || (sns_regs_info->config == TD_FALSE)) {
        cmos_comm_sns_reg_info_init(cis, sns_state);
        sns_state->sync_init = TD_TRUE;
    } else {
        cmos_sns_reg_info_update(vi_pipe, sns_state);
    }

    sns_regs_info->config = TD_FALSE;
    (td_void)memcpy_s(sns_regs_info, sizeof(ot_isp_sns_regs_info),
                      &sns_state->regs_info[0], sizeof(ot_isp_sns_regs_info));
    (td_void)memcpy_s(&sns_state->regs_info[1], sizeof(ot_isp_sns_regs_info),
                      &sns_state->regs_info[0], sizeof(ot_isp_sns_regs_info));
    sns_state->fl[1] = sns_state->fl[0];

    return TD_SUCCESS;
}

static td_void cmos_isp_config_image_mode_param(ot_vi_pipe vi_pipe, td_u8 sns_image_mode,
    ot_isp_sns_state *sns_state)
{
    ot_unused(vi_pipe);
    switch (sns_image_mode) {
        case CV2005_2M_30FPS_10BIT_LINEAR_MODE:
            sns_state->fl_std = CV2005_VMAX_LINEAR;
            break;
        default:
            sns_state->fl_std = CV2005_VMAX_LINEAR;
            break;
    }

    return;
}

static td_s32 cmos_isp_set_image_mode(ot_vi_pipe vi_pipe, const ot_isp_cmos_sns_image_mode *sns_image_mode)
{
    td_u32 i;
    td_u8 image_mode;
    cis_info *cis = TD_NULL;
    ot_isp_sns_state *sns_state = TD_NULL;

    sns_check_pipe_return(vi_pipe);
    sns_check_pointer_return(sns_image_mode);

    cis = cmos_get_info(vi_pipe);
    sns_check_pointer_return(cis);

    sns_state = cis->sns_state;
    sns_check_pointer_return(sns_state);

    image_mode = sns_state->img_mode;

    for (i = 0; i < CV2005_MODE_MAX; i++) {
        if (sns_image_mode->fps <= cis->mode_tbl[i].max_fps &&
            sns_image_mode->width <= cis->mode_tbl[i].width &&
            sns_image_mode->height <= cis->mode_tbl[i].height &&
            sns_state->wdr_mode == cis->mode_tbl[i].wdr_mode) {
            image_mode = (cv2005_res_mode)i;
            break;
        }
    }

    if (i >= CV2005_MODE_MAX) {
        cmos_err_mode_print(sns_image_mode, sns_state);
        return TD_FAILURE;
    }

    cmos_isp_config_image_mode_param(vi_pipe, image_mode, sns_state);

    if ((sns_state->init == TD_TRUE) && (image_mode == sns_state->img_mode)) {
        return OT_ISP_DO_NOT_NEED_SWITCH_IMAGEMODE; /* Don't need to switch image_mode */
    }

    sns_state->sync_init = TD_FALSE;
    sns_state->img_mode = image_mode;
    sns_state->fl[0] = sns_state->fl_std;
    sns_state->fl[1] = sns_state->fl[0];

    return TD_SUCCESS;
}

static td_bool cmos_isp_get_quick_start_flag(ot_vi_pipe vi_pipe)
{
    td_s32 ret;
    ot_isp_ctrl_param isp_ctrl_param = {};

    ret = ot_mpi_isp_get_ctrl_param(vi_pipe, &isp_ctrl_param);
    if (ret != TD_SUCCESS) {
        isp_err_trace("pipe[%d] call ot_mpi_isp_get_ctrl_param failed! ret = 0x%x, quick_start_flag force set to 0!\n",
            vi_pipe, ret);
        return TD_FALSE;
    }

    return isp_ctrl_param.quick_start_en;
}

static void cmos_isp_init(ot_vi_pipe vi_pipe)
{
    td_s32           ret;
    td_bool          quick_start_flag = TD_FALSE;
    cis_info *cis = TD_NULL;
    ot_isp_sns_state *sensor_state = TD_NULL;

    sns_check_pipe_void_return(vi_pipe);

    cis = cmos_get_info(vi_pipe);
    sns_check_pointer_void_return(cis);

    sensor_state = cis->sns_state;
    sns_check_pointer_void_return(sensor_state);

    cis->i2c.addr = CV2005_I2C_ADDR;
    cis->i2c.addr_byte_num = CV2005_ADDR_BYTE;
    cis->i2c.data_byte_num = CV2005_DATA_BYTE;

    ret = cis_i2c_init(cis);
    if (ret != TD_SUCCESS) {
        isp_err_trace("i2c init failed!\n");
        return;
    }

    quick_start_flag = cmos_isp_get_quick_start_flag(vi_pipe);
    if (quick_start_flag == TD_TRUE) {
        sensor_state->init = TD_TRUE;
        return;
    }

    /* When sensor first init, config all registers */
    cv2005_linear_1080P30_10bit_init(cis);

    sensor_state->init = TD_TRUE;

    return;
}

static void cmos_isp_exit(ot_vi_pipe vi_pipe)
{
    td_s32 ret;
    cis_info *cis = TD_NULL;

    sns_check_pipe_void_return(vi_pipe);

    cis = cmos_get_info(vi_pipe);
    sns_check_pointer_void_return(cis);

    ret = cis_i2c_exit(cis);
    if (ret != TD_SUCCESS) {
        isp_err_trace("CV2005 exit failed!\n");
    }

    return;
}

static td_void cmos_isp_global_init(ot_vi_pipe vi_pipe)
{
    ot_isp_sns_state *sns_state = TD_NULL;
    cis_info *cis = TD_NULL;

    sns_check_pipe_void_return(vi_pipe);

    cis = cmos_get_info(vi_pipe);
    sns_check_pointer_void_return(cis);

    sns_state = cis->sns_state;
    sns_check_pointer_void_return(sns_state);

    sns_state->init      = TD_FALSE;
    sns_state->sync_init = TD_FALSE;
    sns_state->img_mode  = CV2005_2M_30FPS_10BIT_LINEAR_MODE;
    sns_state->wdr_mode  = OT_WDR_MODE_NONE;
    sns_state->fl_std    = CV2005_VMAX_LINEAR;
    sns_state->fl[0]     = CV2005_VMAX_LINEAR;
    sns_state->fl[1]     = CV2005_VMAX_LINEAR;

    (td_void)memset_s(&sns_state->regs_info[0], sizeof(ot_isp_sns_regs_info), 0, sizeof(ot_isp_sns_regs_info));
    (td_void)memset_s(&sns_state->regs_info[1], sizeof(ot_isp_sns_regs_info), 0, sizeof(ot_isp_sns_regs_info));

    return;
}

static td_void cmos_isp_set_pixel_detect(ot_vi_pipe vi_pipe, td_bool enable)
{
    sns_check_pipe_void_return(vi_pipe);

    return;
}

static td_void cmos_init_sensor_exp_function(ot_isp_sns_exp_func *sensor_exp_func)
{
    (td_void)memset_s(sensor_exp_func, sizeof(ot_isp_sns_exp_func), 0, sizeof(ot_isp_sns_exp_func));

    sensor_exp_func->pfn_cmos_sns_init              = cmos_isp_init;
    sensor_exp_func->pfn_cmos_sns_exit              = cmos_isp_exit;
    sensor_exp_func->pfn_cmos_sns_global_init       = cmos_isp_global_init;
    sensor_exp_func->pfn_cmos_set_image_mode        = cmos_isp_set_image_mode;
    sensor_exp_func->pfn_cmos_set_wdr_mode          = cmos_isp_set_wdr_mode;
    sensor_exp_func->pfn_cmos_get_isp_default       = cmos_isp_get_default;
    sensor_exp_func->pfn_cmos_get_isp_black_level   = cmos_isp_get_black_level;
    sensor_exp_func->pfn_cmos_get_blc_clamp_info    = cmos_isp_get_blc_clamp_info;
    sensor_exp_func->pfn_cmos_set_pixel_detect      = cmos_isp_set_pixel_detect;
    sensor_exp_func->pfn_cmos_get_sns_reg_info      = cmos_isp_get_sns_regs_info;
}

static td_s32 cmos_register_callback(ot_vi_pipe vi_pipe, ot_isp_3a_alg_lib *ae_lib, ot_isp_3a_alg_lib *awb_lib)
{
    td_s32 ret;
    cis_register reg = {0};
    cis_info *cis = TD_NULL;

    sns_check_pipe_return(vi_pipe);
    sns_check_pointer_return(ae_lib);
    sns_check_pointer_return(awb_lib);

    cis = cmos_get_info(vi_pipe);
    sns_check_pointer_return(cis);

    cis->pipe = vi_pipe;

    reg.ae_lib = ae_lib;
    reg.awb_lib = awb_lib;

    cmos_init_sensor_exp_function(&reg.isp_register.sns_exp);
    cmos_init_ae_exp_function(&reg.ae_register.sns_exp);
    cmos_init_awb_exp_function(&reg.awb_register.sns_exp);

    ret = cis_register_callback(cis, &reg);
    if (ret != TD_SUCCESS) {
        isp_err_trace("cis_register_callback failed!\n");
        return ret;
    }

    return TD_SUCCESS;
}

static td_s32 cmos_unregister_callback(ot_vi_pipe vi_pipe, ot_isp_3a_alg_lib *ae_lib, ot_isp_3a_alg_lib *awb_lib)
{
    td_s32 ret;
    cis_register reg = {0};
    cis_info *cis = TD_NULL;

    sns_check_pipe_return(vi_pipe);
    sns_check_pointer_return(ae_lib);
    sns_check_pointer_return(awb_lib);

    cis = cmos_get_info(vi_pipe);
    sns_check_pointer_return(cis);

    reg.ae_lib = ae_lib;
    reg.awb_lib = awb_lib;
    ret = cis_unregister_callback(cis, &reg);
    if (ret != TD_SUCCESS) {
        isp_err_trace("cis_register_callback failed!\n");
        return ret;
    }

    return TD_SUCCESS;
}

static void cmos_standby(ot_vi_pipe vi_pipe)
{
    ot_unused(vi_pipe);
    return;
}

static void cmos_restart(ot_vi_pipe vi_pipe)
{
    ot_unused(vi_pipe);
    return;
}

static td_s32 cmos_write_register(ot_vi_pipe vi_pipe, td_u32 addr, td_u32 data)
{
    cis_info *cis = TD_NULL;

    sns_check_pipe_return(vi_pipe);

    cis = cmos_get_info(vi_pipe);
    sns_check_pointer_return(cis);

    sns_check_return(cis_write_reg(&cis->i2c, addr, data));

    return TD_SUCCESS;
}

static td_s32 cmos_read_register(ot_vi_pipe vi_pipe, td_u32 addr)
{
    ot_unused(vi_pipe);
    ot_unused(addr);
    return TD_SUCCESS;
}

static td_s32 cmos_set_bus_info(ot_vi_pipe vi_pipe, ot_isp_sns_commbus sns_bus_info)
{
    cis_info *cis = TD_NULL;

    sns_check_pipe_return(vi_pipe);

    cis = cmos_get_info(vi_pipe);
    sns_check_pointer_return(cis);

    cis->bus_info.i2c_dev = sns_bus_info.i2c_dev;

    return TD_SUCCESS;
}

static td_s32 cmos_set_init(ot_vi_pipe vi_pipe, ot_isp_init_attr *init_attr)
{
    cis_info *cis = TD_NULL;

    sns_check_pipe_return(vi_pipe);
    sns_check_pointer_return(init_attr);

    cis = cmos_get_info(vi_pipe);
    sns_check_pointer_return(cis);

    cis_init_attr(cis, init_attr);

    return TD_SUCCESS;
}

static void cmos_mirror_flip (ot_vi_pipe vi_pipe, int type)
{
    printf("vi_pipe:%d, flip_type:%d\n", vi_pipe, type);

    if(type == 0){
        cmos_write_register(vi_pipe,0x3028,0x00);
    }else if(type == 1){
        cmos_write_register(vi_pipe,0x3028,0x01);
    }else if(type == 2){
        cmos_write_register(vi_pipe,0x3028,0x02);
    }else if(type == 3){
        cmos_write_register(vi_pipe,0x3028,0x03);
    }else{
    }

    return;
}

void cv2005_mirror_flip(ot_vi_pipe vi_pipe, ot_isp_sns_mirrorflip_type e_sns_mirror_flip)
{
	cmos_mirror_flip(vi_pipe, e_sns_mirror_flip);
}

ot_isp_sns_obj g_sns_cv2005_obj = {
    .pfn_register_callback     = cmos_register_callback,
    .pfn_un_register_callback  = cmos_unregister_callback,
    .pfn_standby               = cmos_standby,
    .pfn_restart               = cmos_restart,
    .pfn_mirror_flip           = cv2005_mirror_flip,
    .pfn_set_blc_clamp         = TD_NULL,
    .pfn_write_reg             = cmos_write_register,
    .pfn_read_reg              = cmos_read_register,
    .pfn_set_bus_info          = cmos_set_bus_info,
    .pfn_set_init              = cmos_set_init
};

ot_isp_sns_obj *cv2005_get_obj(td_void)
{
    return &g_sns_cv2005_obj;
}
