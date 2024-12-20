/*
 ******************************************************************************
 * @file    IIS2MDC_MAG_driver.c
 * @author  MEMS Software Solution Team
 * @brief   IIS2MDC driver file
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2018 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "IIS2MDC_MAG_driver.h"

/**
  * @addtogroup  iis2mdc
  * @brief  This file provides a set of functions needed to drive the
  *         iis2mdc enanced inertial module.
  * @{
  */

/**
  * @addtogroup  interfaces_functions
  * @brief  This section provide a set of functions used to read and write
  *         a generic register of the device.
  * @{
  */

/**
  * @brief  Read generic device register
  *
  * @param  iis2mdc_ctx_t* ctx: read / write interface definitions
  * @param  uint8_t reg: register to read
  * @param  uint8_t* data: pointer to buffer that store the data read
  * @param  uint16_t len: number of consecutive register to read
  *
  */
int32_t iis2mdc_read_reg(iis2mdc_ctx_t *ctx, uint8_t reg, uint8_t *data,
                         uint16_t len)
{
  return ctx->read_reg(ctx->handle, reg, data, len);
}

/**
  * @brief  Write generic device register
  *
  * @param  iis2mdc_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t reg: register to write
  * @param  uint8_t* data: pointer to data to write in register reg
  * @param  uint16_t len: number of consecutive register to write
  *
*/
int32_t iis2mdc_write_reg(iis2mdc_ctx_t *ctx, uint8_t reg, uint8_t *data,
                          uint16_t len)
{
  return ctx->write_reg(ctx->handle, reg, data, len);
}

/**
  * @}
  */

/**
  * @addtogroup  data_generation
  * @brief   This section group all the functions concerning data generation
  * @{
  */

/**
  * @brief  mag_user_offset: [set]  These registers comprise a 3 group of
  *                                 16-bit number and represent hard-iron
  *                                 offset in order to compensate environmental
  *                                 effects. Data format is the same of
  *                                 output data raw: two’s complement with
  *                                 1LSb = 1.5mG. These values act on the
  *                                 magnetic output data value in order to
  *                                 delete the environmental offset.
  *
  * @param  iis2mdc_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t * : buffer that contains data to write
  *
  */
int32_t iis2mdc_mag_user_offset_set(iis2mdc_ctx_t *ctx, uint8_t *buff)
{
  return iis2mdc_read_reg(ctx, IIS2MDC_OFFSET_X_REG_L, buff, 6);
}

/**
  * @brief  mag_user_offset: [get]  These registers comprise a 3 group of
  *                                 16-bit number and represent hard-iron
  *                                 offset in order to compensate environmental
  *                                 effects. Data format is the same of
  *                                 output data raw: two’s complement with
  *                                 1LSb = 1.5mG. These values act on the
  *                                 magnetic output data value in order to
  *                                 delete the environmental offset.
  *
  * @param  iis2mdc_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t * : buffer that stores data read
  *
  */
int32_t iis2mdc_mag_user_offset_get(iis2mdc_ctx_t *ctx, uint8_t *buff)
{
  return iis2mdc_read_reg(ctx, IIS2MDC_OFFSET_X_REG_L, buff, 6);
}

/**
  * @brief  operating_mode: [set]  Operating mode selection.
  *
  * @param  iis2mdc_ctx_t *ctx: read / write interface definitions
  * @param  iis2mdc_md_t: change the values of md in reg CFG_REG_A
  *
  */
int32_t iis2mdc_operating_mode_set(iis2mdc_ctx_t *ctx, iis2mdc_md_t val)
{
  iis2mdc_reg_t reg;
  int32_t mm_error;

  mm_error = iis2mdc_read_reg(ctx, IIS2MDC_CFG_REG_A, &reg.byte, 1);
  reg.cfg_reg_a.md = val;
  mm_error = iis2mdc_write_reg(ctx, IIS2MDC_CFG_REG_A, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  operating_mode: [get]  Operating mode selection.
  *
  * @param  iis2mdc_ctx_t *ctx: read / write interface definitions
  * @param  iis2mdc_md_t: Get the values of md in reg CFG_REG_A
  *
  */
int32_t iis2mdc_operating_mode_get(iis2mdc_ctx_t *ctx, iis2mdc_md_t *val)
{
  iis2mdc_reg_t reg;
  int32_t mm_error;

  mm_error = iis2mdc_read_reg(ctx, IIS2MDC_CFG_REG_A, &reg.byte, 1);
  *val = (iis2mdc_md_t) reg.cfg_reg_a.md;

  return mm_error;
}

/**
  * @brief  data_rate: [set]  Output data rate selection.
  *
  * @param  iis2mdc_ctx_t *ctx: read / write interface definitions
  * @param  iis2mdc_odr_t: change the values of odr in reg CFG_REG_A
  *
  */
int32_t iis2mdc_data_rate_set(iis2mdc_ctx_t *ctx, iis2mdc_odr_t val)
{
  iis2mdc_reg_t reg;
  int32_t mm_error;

  mm_error = iis2mdc_read_reg(ctx, IIS2MDC_CFG_REG_A, &reg.byte, 1);
  reg.cfg_reg_a.odr = val;
  mm_error = iis2mdc_write_reg(ctx, IIS2MDC_CFG_REG_A, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  data_rate: [get]  Output data rate selection.
  *
  * @param  iis2mdc_ctx_t *ctx: read / write interface definitions
  * @param  iis2mdc_odr_t: Get the values of odr in reg CFG_REG_A
  *
  */
int32_t iis2mdc_data_rate_get(iis2mdc_ctx_t *ctx, iis2mdc_odr_t *val)
{
  iis2mdc_reg_t reg;
  int32_t mm_error;

  mm_error = iis2mdc_read_reg(ctx, IIS2MDC_CFG_REG_A, &reg.byte, 1);
  *val = (iis2mdc_odr_t) reg.cfg_reg_a.odr;

  return mm_error;
}

/**
  * @brief  power_mode: [set]  Enables high-resolution/low-power mode.
  *
  * @param  iis2mdc_ctx_t *ctx: read / write interface definitions
  * @param  iis2mdc_lp_t: change the values of lp in reg CFG_REG_A
  *
  */
int32_t iis2mdc_power_mode_set(iis2mdc_ctx_t *ctx, iis2mdc_lp_t val)
{
  iis2mdc_reg_t reg;
  int32_t mm_error;

  mm_error = iis2mdc_read_reg(ctx, IIS2MDC_CFG_REG_A, &reg.byte, 1);
  reg.cfg_reg_a.lp = val;
  mm_error = iis2mdc_write_reg(ctx, IIS2MDC_CFG_REG_A, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  power_mode: [get]  Enables high-resolution/low-power mode.
  *
  * @param  iis2mdc_ctx_t *ctx: read / write interface definitions
  * @param  iis2mdc_lp_t: Get the values of lp in reg CFG_REG_A
  *
  */
int32_t iis2mdc_power_mode_get(iis2mdc_ctx_t *ctx, iis2mdc_lp_t *val)
{
  iis2mdc_reg_t reg;
  int32_t mm_error;

  mm_error = iis2mdc_read_reg(ctx, IIS2MDC_CFG_REG_A, &reg.byte, 1);
  *val = (iis2mdc_lp_t) reg.cfg_reg_a.lp;

  return mm_error;
}

/**
  * @brief  offset_temp_comp: [set]  Enables the magnetometer temperature
  *                                  compensation.
  *
  * @param  iis2mdc_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of comp_temp_en in reg CFG_REG_A
  *
  */
int32_t iis2mdc_offset_temp_comp_set(iis2mdc_ctx_t *ctx, uint8_t val)
{
  iis2mdc_reg_t reg;
  int32_t mm_error;

  mm_error = iis2mdc_read_reg(ctx, IIS2MDC_CFG_REG_A, &reg.byte, 1);
  reg.cfg_reg_a.comp_temp_en = val;
  mm_error = iis2mdc_write_reg(ctx, IIS2MDC_CFG_REG_A, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  offset_temp_comp: [get]  Enables the magnetometer temperature
  *                                  compensation.
  *
  * @param  iis2mdc_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of comp_temp_en in reg CFG_REG_A
  *
  */
int32_t iis2mdc_offset_temp_comp_get(iis2mdc_ctx_t *ctx, uint8_t *val)
{
  iis2mdc_reg_t reg;
  int32_t mm_error;

  mm_error = iis2mdc_read_reg(ctx, IIS2MDC_CFG_REG_A, &reg.byte, 1);
  *val = reg.cfg_reg_a.comp_temp_en;

  return mm_error;
}

/**
  * @brief  low_pass_bandwidth: [set]  Low-pass bandwidth selection.
  *
  * @param  iis2mdc_ctx_t *ctx: read / write interface definitions
  * @param  iis2mdc_lpf_t: change the values of lpf in reg CFG_REG_B
  *
  */
int32_t iis2mdc_low_pass_bandwidth_set(iis2mdc_ctx_t *ctx,
                                       iis2mdc_lpf_t val)
{
  iis2mdc_reg_t reg;
  int32_t mm_error;

  mm_error = iis2mdc_read_reg(ctx, IIS2MDC_CFG_REG_B, &reg.byte, 1);
  reg.cfg_reg_b.lpf = val;
  mm_error = iis2mdc_write_reg(ctx, IIS2MDC_CFG_REG_B, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  low_pass_bandwidth: [get]  Low-pass bandwidth selection.
  *
  * @param  iis2mdc_ctx_t *ctx: read / write interface definitions
  * @param  iis2mdc_lpf_t: Get the values of lpf in reg CFG_REG_B
  *
  */
int32_t iis2mdc_low_pass_bandwidth_get(iis2mdc_ctx_t *ctx,
                                       iis2mdc_lpf_t *val)
{
  iis2mdc_reg_t reg;
  int32_t mm_error;

  mm_error = iis2mdc_read_reg(ctx, IIS2MDC_CFG_REG_B, &reg.byte, 1);
  *val = (iis2mdc_lpf_t) reg.cfg_reg_b.lpf;

  return mm_error;
}

/**
  * @brief  set_rst_mode: [set]
  *
  * @param  iis2mdc_ctx_t *ctx: read / write interface definitions
  * @param  iis2mdc_set_rst_t: change the values of set_rst in
  *                            reg CFG_REG_B
  *
  */
int32_t iis2mdc_set_rst_mode_set(iis2mdc_ctx_t *ctx, iis2mdc_set_rst_t val)
{
  iis2mdc_reg_t reg;
  int32_t mm_error;

  mm_error = iis2mdc_read_reg(ctx, IIS2MDC_CFG_REG_B, &reg.byte, 1);
  reg.cfg_reg_b.set_rst = val;
  mm_error = iis2mdc_write_reg(ctx, IIS2MDC_CFG_REG_B, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  set_rst_mode: [get]
  *
  * @param  iis2mdc_ctx_t *ctx: read / write interface definitions
  * @param  iis2mdc_set_rst_t: Get the values of set_rst in reg CFG_REG_B
  *
  */
int32_t iis2mdc_set_rst_mode_get(iis2mdc_ctx_t *ctx, iis2mdc_set_rst_t *val)
{
  iis2mdc_reg_t reg;
  int32_t mm_error;

  mm_error = iis2mdc_read_reg(ctx, IIS2MDC_CFG_REG_B, &reg.byte, 1);
  *val = (iis2mdc_set_rst_t) reg.cfg_reg_b.set_rst;

  return mm_error;
}

/**
  * @brief   set_rst_sensor_single: [set] Enables offset cancellation
  *                                       in single measurement mode.
  *                                       The OFF_CANC bit must be set
  *                                       to 1 when enabling offset
  *                                       cancellation in single measurement
  *                                       mode this means a call function:
  *                                       set_rst_mode(SENS_OFF_CANC_EVERY_ODR)
  *                                       is need.
  *
  * @param  iis2mdc_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of off_canc_one_shot in
  *                      reg CFG_REG_B
  *
  */
int32_t iis2mdc_set_rst_sensor_single_set(iis2mdc_ctx_t *ctx, uint8_t val)
{
  iis2mdc_reg_t reg;
  int32_t mm_error;

  mm_error = iis2mdc_read_reg(ctx, IIS2MDC_CFG_REG_B, &reg.byte, 1);
  reg.cfg_reg_b.off_canc_one_shot = val;
  mm_error = iis2mdc_write_reg(ctx, IIS2MDC_CFG_REG_B, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief   set_rst_sensor_single: [get] Enables offset cancellation
  *                                       in single measurement mode.
  *                                       The OFF_CANC bit must be set to
  *                                       1 when enabling offset cancellation
  *                                       in single measurement mode this
  *                                       means a call function:
  *                                       set_rst_mode(SENS_OFF_CANC_EVERY_ODR)
  *                                       is need.
  *
  * @param  iis2mdc_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of off_canc_one_shot in reg CFG_REG_B
  *
  */
int32_t iis2mdc_set_rst_sensor_single_get(iis2mdc_ctx_t *ctx, uint8_t *val)
{
  iis2mdc_reg_t reg;
  int32_t mm_error;

  mm_error = iis2mdc_read_reg(ctx, IIS2MDC_CFG_REG_B, &reg.byte, 1);
  *val = reg.cfg_reg_b.off_canc_one_shot;

  return mm_error;
}

/**
  * @brief  block_data_update: [set] Blockdataupdate.
  *
  * @param  iis2mdc_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of bdu in reg CFG_REG_C
  *
  */
int32_t iis2mdc_block_data_update_set(iis2mdc_ctx_t *ctx, uint8_t val)
{
  iis2mdc_reg_t reg;
  int32_t mm_error;

  mm_error = iis2mdc_read_reg(ctx, IIS2MDC_CFG_REG_C, &reg.byte, 1);
  reg.cfg_reg_c.bdu = val;
  mm_error = iis2mdc_write_reg(ctx, IIS2MDC_CFG_REG_C, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  block_data_update: [get] Blockdataupdate.
  *
  * @param  iis2mdc_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of bdu in reg CFG_REG_C
  *
  */
int32_t iis2mdc_block_data_update_get(iis2mdc_ctx_t *ctx, uint8_t *val)
{
  iis2mdc_reg_t reg;
  int32_t mm_error;

  mm_error = iis2mdc_read_reg(ctx, IIS2MDC_CFG_REG_C, &reg.byte, 1);
  *val = reg.cfg_reg_c.bdu;

  return mm_error;
}

/**
  * @brief  mag_data_ready: [get]  Magnetic set of data available.
  *
  * @param  iis2mdc_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of zyxda in reg STATUS_REG
  *
  */
int32_t iis2mdc_mag_data_ready_get(iis2mdc_ctx_t *ctx, uint8_t *val)
{
  iis2mdc_reg_t reg;
  int32_t mm_error;

  mm_error = iis2mdc_read_reg(ctx, IIS2MDC_STATUS_REG, &reg.byte, 1);
  *val = reg.status_reg.zyxda;

  return mm_error;
}

/**
  * @brief  mag_data_ovr: [get]  Magnetic set of data overrun.
  *
  * @param  iis2mdc_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of zyxor in reg STATUS_REG
  *
  */
int32_t iis2mdc_mag_data_ovr_get(iis2mdc_ctx_t *ctx, uint8_t *val)
{
  iis2mdc_reg_t reg;
  int32_t mm_error;

  mm_error = iis2mdc_read_reg(ctx, IIS2MDC_STATUS_REG, &reg.byte, 1);
  *val = reg.status_reg.zyxor;

  return mm_error;
}

/**
  * @brief  magnetic_raw: [get]  Magnetic output value.
  *
  * @param  iis2mdc_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t * : buffer that stores data read
  *
  */
int32_t iis2mdc_magnetic_raw_get(iis2mdc_ctx_t *ctx, uint8_t *buff)
{
  return iis2mdc_read_reg(ctx, IIS2MDC_OUTX_L_REG, buff, 6);
}

/**
  * @brief  temperature_raw: [get]  Temperature output value.
  *
  * @param  iis2mdc_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t * : buffer that stores data read
  *
  */
int32_t iis2mdc_temperature_raw_get(iis2mdc_ctx_t *ctx, uint8_t *buff)
{
  return iis2mdc_read_reg(ctx, IIS2MDC_TEMP_OUT_L_REG, buff, 2);
}

/**
  * @}
  */

/**
  * @addtogroup  common
  * @brief   This section group common usefull functions
  * @{
  */

/**
  * @brief  device_id: [get] DeviceWhoamI.
  *
  * @param  iis2mdc_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t * : buffer that stores data read
  *
  */
int32_t iis2mdc_device_id_get(iis2mdc_ctx_t *ctx, uint8_t *buff)
{
  return iis2mdc_read_reg(ctx, IIS2MDC_WHO_AM_I, buff, 1);
}

/**
  * @brief  reset: [set]  Software reset. Restore the default values in
  *                       user registers.
  *
  * @param  iis2mdc_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of soft_rst in reg CFG_REG_A
  *
  */
int32_t iis2mdc_reset_set(iis2mdc_ctx_t *ctx, uint8_t val)
{
  iis2mdc_reg_t reg;
  int32_t mm_error;

  mm_error = iis2mdc_read_reg(ctx, IIS2MDC_CFG_REG_A, &reg.byte, 1);
  reg.cfg_reg_a.soft_rst = val;
  mm_error = iis2mdc_write_reg(ctx, IIS2MDC_CFG_REG_A, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  reset: [get]  Software reset. Restore the default values
  *                       in user registers.
  *
  * @param  iis2mdc_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of soft_rst in reg CFG_REG_A
  *
  */
int32_t iis2mdc_reset_get(iis2mdc_ctx_t *ctx, uint8_t *val)
{
  iis2mdc_reg_t reg;
  int32_t mm_error;

  mm_error = iis2mdc_read_reg(ctx, IIS2MDC_CFG_REG_A, &reg.byte, 1);
  *val = reg.cfg_reg_a.soft_rst;

  return mm_error;
}

/**
  * @brief  boot: [set]  Reboot memory content. Reload the calibration
  *                      parameters.
  *
  * @param  iis2mdc_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of reboot in reg CFG_REG_A
  *
  */
int32_t iis2mdc_boot_set(iis2mdc_ctx_t *ctx, uint8_t val)
{
  iis2mdc_reg_t reg;
  int32_t mm_error;

  mm_error = iis2mdc_read_reg(ctx, IIS2MDC_CFG_REG_A, &reg.byte, 1);
  reg.cfg_reg_a.reboot = val;
  mm_error = iis2mdc_write_reg(ctx, IIS2MDC_CFG_REG_A, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  boot: [get]  Reboot memory content. Reload the
  *                      calibration parameters.
  *
  * @param  iis2mdc_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of reboot in reg CFG_REG_A
  *
  */
int32_t iis2mdc_boot_get(iis2mdc_ctx_t *ctx, uint8_t *val)
{
  iis2mdc_reg_t reg;
  int32_t mm_error;

  mm_error = iis2mdc_read_reg(ctx, IIS2MDC_CFG_REG_A, &reg.byte, 1);
  *val = reg.cfg_reg_a.reboot;

  return mm_error;
}

/**
  * @brief  self_test: [set] Selftest.
  *
  * @param  iis2mdc_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of self_test in reg CFG_REG_C
  *
  */
int32_t iis2mdc_self_test_set(iis2mdc_ctx_t *ctx, uint8_t val)
{
  iis2mdc_reg_t reg;
  int32_t mm_error;

  mm_error = iis2mdc_read_reg(ctx, IIS2MDC_CFG_REG_C, &reg.byte, 1);
  reg.cfg_reg_c.self_test = val;
  mm_error = iis2mdc_write_reg(ctx, IIS2MDC_CFG_REG_C, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  self_test: [get] Selftest.
  *
  * @param  iis2mdc_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of self_test in reg CFG_REG_C
  *
  */
int32_t iis2mdc_self_test_get(iis2mdc_ctx_t *ctx, uint8_t *val)
{
  iis2mdc_reg_t reg;
  int32_t mm_error;

  mm_error = iis2mdc_read_reg(ctx, IIS2MDC_CFG_REG_C, &reg.byte, 1);
  *val = reg.cfg_reg_c.self_test;

  return mm_error;
}

/**
  * @brief  data_format: [set]  Big/Little Endian data selection.
  *
  * @param  iis2mdc_ctx_t *ctx: read / write interface definitions
  * @param  iis2mdc_ble_t: change the values of ble in reg CFG_REG_C
  *
  */
int32_t iis2mdc_data_format_set(iis2mdc_ctx_t *ctx, iis2mdc_ble_t val)
{
  iis2mdc_reg_t reg;
  int32_t mm_error;

  mm_error = iis2mdc_read_reg(ctx, IIS2MDC_CFG_REG_C, &reg.byte, 1);
  reg.cfg_reg_c.ble = val;
  mm_error = iis2mdc_write_reg(ctx, IIS2MDC_CFG_REG_C, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  data_format: [get]  Big/Little Endian data selection.
  *
  * @param  iis2mdc_ctx_t *ctx: read / write interface definitions
  * @param  iis2mdc_ble_t: Get the values of ble in reg CFG_REG_C
  *
  */
int32_t iis2mdc_data_format_get(iis2mdc_ctx_t *ctx, iis2mdc_ble_t *val)
{
  iis2mdc_reg_t reg;
  int32_t mm_error;

  mm_error = iis2mdc_read_reg(ctx, IIS2MDC_CFG_REG_C, &reg.byte, 1);
  *val = (iis2mdc_ble_t) reg.cfg_reg_c.ble;

  return mm_error;
}

/**
  * @brief  status: [get]  Info about device status.
  *
  * @param  iis2mdc_ctx_t *ctx: read / write interface definitions
  * @param  iis2mdc_status_reg_t: registers STATUS_REG
  *
  */
int32_t iis2mdc_status_get(iis2mdc_ctx_t *ctx, iis2mdc_status_reg_t *val)
{
  return iis2mdc_read_reg(ctx, IIS2MDC_STATUS_REG, (uint8_t *) val, 1);
}

/**
  * @}
  */

/**
  * @addtogroup  interrupts
  * @brief   This section group all the functions that manage interrupts
  * @{
  */

/**
  * @brief  offset_int_conf: [set]  The interrupt block recognition checks
  *                                 data after/before the hard-iron correction
  *                                 to discover the interrupt.
  *
  * @param  iis2mdc_ctx_t *ctx: read / write interface definitions
  * @param  iis2mdc_int_on_dataoff_t: change the values of int_on_dataoff in
  *                                   reg CFG_REG_B
  *
  */
int32_t iis2mdc_offset_int_conf_set(iis2mdc_ctx_t *ctx,
                                    iis2mdc_int_on_dataoff_t val)
{
  iis2mdc_reg_t reg;
  int32_t mm_error;

  mm_error = iis2mdc_read_reg(ctx, IIS2MDC_CFG_REG_B, &reg.byte, 1);
  reg.cfg_reg_b.int_on_dataoff = val;
  mm_error = iis2mdc_write_reg(ctx, IIS2MDC_CFG_REG_B, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  offset_int_conf: [get]  The interrupt block recognition checks
  *                                 data after/before the hard-iron correction
  *                                 to discover the interrupt.
  *
  * @param  iis2mdc_ctx_t *ctx: read / write interface definitions
  * @param  iis2mdc_int_on_dataoff_t: Get the values of int_on_dataoff in
  *                                   reg CFG_REG_B
  *
  */
int32_t iis2mdc_offset_int_conf_get(iis2mdc_ctx_t *ctx,
                                    iis2mdc_int_on_dataoff_t *val)
{
  iis2mdc_reg_t reg;
  int32_t mm_error;

  mm_error = iis2mdc_read_reg(ctx, IIS2MDC_CFG_REG_B, &reg.byte, 1);
  *val = (iis2mdc_int_on_dataoff_t) reg.cfg_reg_b.int_on_dataoff;

  return mm_error;
}

/**
  * @brief  drdy_on_pin: [set]  Data-ready signal on INT_DRDY pin.
  *
  * @param  iis2mdc_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of drdy_on_pin in reg CFG_REG_C
  *
  */
int32_t iis2mdc_drdy_on_pin_set(iis2mdc_ctx_t *ctx, uint8_t val)
{
  iis2mdc_reg_t reg;
  int32_t mm_error;

  mm_error = iis2mdc_read_reg(ctx, IIS2MDC_CFG_REG_C, &reg.byte, 1);
  reg.cfg_reg_c.drdy_on_pin = val;
  mm_error = iis2mdc_write_reg(ctx, IIS2MDC_CFG_REG_C, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  drdy_on_pin: [get]  Data-ready signal on INT_DRDY pin.
  *
  * @param  iis2mdc_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of drdy_on_pin in reg CFG_REG_C
  *
  */
int32_t iis2mdc_drdy_on_pin_get(iis2mdc_ctx_t *ctx, uint8_t *val)
{
  iis2mdc_reg_t reg;
  int32_t mm_error;

  mm_error = iis2mdc_read_reg(ctx, IIS2MDC_CFG_REG_C, &reg.byte, 1);
  *val = reg.cfg_reg_c.drdy_on_pin;

  return mm_error;
}

/**
  * @brief  int_on_pin: [set]  Interrupt signal on INT_DRDY pin.
  *
  * @param  iis2mdc_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of int_on_pin in reg CFG_REG_C
  *
  */
int32_t iis2mdc_int_on_pin_set(iis2mdc_ctx_t *ctx, uint8_t val)
{
  iis2mdc_reg_t reg;
  int32_t mm_error;

  mm_error = iis2mdc_read_reg(ctx, IIS2MDC_CFG_REG_C, &reg.byte, 1);
  reg.cfg_reg_c.int_on_pin = val;
  mm_error = iis2mdc_write_reg(ctx, IIS2MDC_CFG_REG_C, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  int_on_pin: [get]  Interrupt signal on INT_DRDY pin.
  *
  * @param  iis2mdc_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of int_on_pin in reg CFG_REG_C
  *
  */
int32_t iis2mdc_int_on_pin_get(iis2mdc_ctx_t *ctx, uint8_t *val)
{
  iis2mdc_reg_t reg;
  int32_t mm_error;

  mm_error = iis2mdc_read_reg(ctx, IIS2MDC_CFG_REG_C, &reg.byte, 1);
  *val = reg.cfg_reg_c.int_on_pin;

  return mm_error;
}

/**
  * @brief  int_gen_conf: [set]  Interrupt generator configuration register
  *
  * @param  iis2mdc_ctx_t *ctx: read / write interface definitions
  * @param  iis2mdc_int_crtl_reg_t: registers INT_CRTL_REG
  *
  */
int32_t iis2mdc_int_gen_conf_set(iis2mdc_ctx_t *ctx,
                                 iis2mdc_int_crtl_reg_t *val)
{
  return iis2mdc_read_reg(ctx, IIS2MDC_INT_CRTL_REG, (uint8_t *) val, 1);
}

/**
  * @brief  int_gen_conf: [get]  Interrupt generator configuration register
  *
  * @param  iis2mdc_ctx_t *ctx: read / write interface definitions
  * @param  iis2mdc_int_crtl_reg_t: registers INT_CRTL_REG
  *
  */
int32_t iis2mdc_int_gen_conf_get(iis2mdc_ctx_t *ctx,
                                 iis2mdc_int_crtl_reg_t *val)
{
  return iis2mdc_read_reg(ctx, IIS2MDC_INT_CRTL_REG, (uint8_t *) val, 1);
}

/**
  * @brief  int_gen_source: [get]  Interrupt generator source register
  *
  * @param  iis2mdc_ctx_t *ctx: read / write interface definitions
  * @param  iis2mdc_int_source_reg_t: registers INT_SOURCE_REG
  *
  */
int32_t iis2mdc_int_gen_source_get(iis2mdc_ctx_t *ctx,
                                   iis2mdc_int_source_reg_t *val)
{
  return iis2mdc_read_reg(ctx, IIS2MDC_INT_SOURCE_REG, (uint8_t *) val, 1);
}

/**
  * @brief  int_gen_treshold: [set]  User-defined threshold value for xl
  *                                  interrupt event on generator.
  *                                  Data format is the same of output
  *                                  data raw: two’s complement with
  *                                  1LSb = 1.5mG.
  *
  * @param  iis2mdc_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t * : buffer that contains data to write
  *
  */
int32_t iis2mdc_int_gen_treshold_set(iis2mdc_ctx_t *ctx, uint8_t *buff)
{
  return iis2mdc_read_reg(ctx, IIS2MDC_INT_THS_L_REG, buff, 2);
}

/**
  * @brief  int_gen_treshold: [get]  User-defined threshold value for
  *                                  xl interrupt event on generator.
  *                                  Data format is the same of output
  *                                  data raw: two’s complement with
  *                                  1LSb = 1.5mG.
  *
  * @param  iis2mdc_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t * : buffer that stores data read
  *
  */
int32_t iis2mdc_int_gen_treshold_get(iis2mdc_ctx_t *ctx, uint8_t *buff)
{
  return iis2mdc_read_reg(ctx, IIS2MDC_INT_THS_L_REG, buff, 2);
}

/**
  * @}
  */

/**
  * @addtogroup  serial_interface
  * @brief   This section group all the functions concerning serial
  *          interface management
  * @{
  */

/**
  * @brief  i2c_interface: [set]  Enable/Disable I2C interface.
  *
  * @param  iis2mdc_ctx_t *ctx: read / write interface definitions
  * @param  iis2mdc_i2c_dis_t: change the values of i2c_dis in reg CFG_REG_C
  *
  */
int32_t iis2mdc_i2c_interface_set(iis2mdc_ctx_t *ctx, iis2mdc_i2c_dis_t val)
{
  iis2mdc_reg_t reg;
  int32_t mm_error;

  mm_error = iis2mdc_read_reg(ctx, IIS2MDC_CFG_REG_C, &reg.byte, 1);
  reg.cfg_reg_c.i2c_dis = val;
  mm_error = iis2mdc_write_reg(ctx, IIS2MDC_CFG_REG_C, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  i2c_interface: [get]  Enable/Disable I2C interface.
  *
  * @param  iis2mdc_ctx_t *ctx: read / write interface definitions
  * @param  iis2mdc_i2c_dis_t: Get the values of i2c_dis in reg CFG_REG_C
  *
  */
int32_t iis2mdc_i2c_interface_get(iis2mdc_ctx_t *ctx, iis2mdc_i2c_dis_t *val)
{
  iis2mdc_reg_t reg;
  int32_t mm_error;

  mm_error = iis2mdc_read_reg(ctx, IIS2MDC_CFG_REG_C, &reg.byte, 1);
  *val = (iis2mdc_i2c_dis_t) reg.cfg_reg_c.i2c_dis;

  return mm_error;
}

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
