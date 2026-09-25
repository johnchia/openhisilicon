/*
  Copyright (c), 2001-2024, Shenshu Tech. Co., Ltd.
 */

#include "sensor_common.h"
#include "cv2005_cfg.h"
#include "cv2005_cmos.h"

static void cv2005_default_reg_init(cis_info *cis)
{
    td_u32 i;
    td_s32 ret = TD_SUCCESS;
    ot_isp_sns_state *past_sensor = TD_NULL;

    past_sensor = cis->sns_state;
    for (i = 0; i < past_sensor->regs_info[0].reg_num; i++) {
        ret += cis_write_reg(&cis->i2c,
            past_sensor->regs_info[0].i2c_data[i].reg_addr,
            past_sensor->regs_info[0].i2c_data[i].data);
    }

    if (ret != TD_SUCCESS) {
        isp_err_trace("write register failed!\n");
    }
    return;
}

static td_s32 cv2005_reg_init(cis_info *cis, cis_reg_cfg *cfg, td_u32 len)
{
    td_u32 i;

    sns_check_return(cis_write_reg(&cis->i2c, 0x3004, 0x01));
    sns_check_return(cis_write_reg(&cis->i2c, 0x3004, 0x00));
    cis_delay_ms(1); /* 1ms */

    for (i = 0; i < len; i++) {
        sns_check_return(cis_write_reg(&cis->i2c, cfg->addr, cfg->data));
        cfg++;
    }
    //shaokc..
    // return TD_SUCCESS; //shaokc...

    cv2005_default_reg_init(cis);

    sns_check_return(cis_write_reg(&cis->i2c, 0x3000, 0x00));

    return TD_SUCCESS;
}

td_s32 cv2005_linear_1080P30_10bit_init(cis_info *cis)
{
    td_s32 ret;
    td_u32 len;
    cis_reg_cfg *cfg = cv2005_linear_1080P30_10bit;

    sns_check_pointer_return(cis);

    len = (td_u32)(sizeof(cv2005_linear_1080P30_10bit) / sizeof(cv2005_linear_1080P30_10bit[0]));
    ret = cv2005_reg_init(cis, cfg, len);
    if (ret != TD_SUCCESS) {
        isp_err_trace("cv2005_reg_init failed!\n");
        return ret;
    }

    printf("===================================================================================\n");
    printf("vi_pipe:%d,== CV2005_MIPI_24Minput_2lane_10bit_465Mbps_1920x1080_30fps Init OK! ==\n", cis->pipe);
    printf("===================================================================================\n");

    return TD_SUCCESS;
}
