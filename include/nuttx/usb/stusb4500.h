/****************************************************************************
 * include/nuttx/usb/stusb4500.h
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_USB_STUSB4500_H
#define __INCLUDE_NUTTX_USB_STUSB4500_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/fs/ioctl.h>

/****************************************************************************
 * Pre-Processor Declarations
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* IOCTL Commands ***********************************************************/

#define USBCIOC_READ_DEVID    _USBCIOC(0x0001)  /* Arg: uint8_t* pointer */

/****************************************************************************
 * Public Types
 ****************************************************************************/


#define  STUSB4500_REG_BCD_TYPEC_REV_LOW         0x06
#define  STUSB4500_REG_BCD_TYPEC_REV_HIGH        0x07
#define  STUSB4500_REG_BCD_USBPD_REV_LOW         0x08
#define  STUSB4500_REG_BCD_USBPD_REV_HIGH        0x09
#define  STUSB4500_REG_DEVICE_CAPAB_HIGH         0x0A
#define  STUSB4500_REG_ALERT_STATUS_1            0x0B
#define  STUSB4500_REG_ALERT_STATUS_1_MASK       0x0C
#define  STUSB4500_REG_PORT_STATUS_0             0x0D
#define  STUSB4500_REG_PORT_STATUS_1             0x0E
#define  STUSB4500_REG_TYPEC_MONITORING_STATUS_0 0x0F
#define  STUSB4500_REG_TYPEC_MONITORING_STATUS_1 0x10
#define  STUSB4500_REG_CC_STATUS                 0x11
#define  STUSB4500_REG_CC_HW_FAULT_STATUS_0      0x12
#define  STUSB4500_REG_CC_HW_FAULT_STATUS_1      0x13
#define  STUSB4500_REG_PD_TYPEC_STATUS           0x14
#define  STUSB4500_REG_TYPEC_STATUS              0x15
#define  STUSB4500_REG_PRT_STATUS                0x16
#define  STUSB4500_REG_PD_COMMAND_CTRL           0x1A
#define  STUSB4500_REG_MONITORING_CTRL_0         0x20
#define  STUSB4500_REG_MONITORING_CTRL_2         0x22
#define  STUSB4500_REG_RESET_CTRL                0x23
#define  STUSB4500_REG_VBUS_DISCHARGE_TIME_CTRL  0x25
#define  STUSB4500_REG_VBUS_DISCHARGE_CTRL       0x26
#define  STUSB4500_REG_VBUS_CTRL                 0x27
#define  STUSB4500_REG_PE_FSM                    0x29
#define  STUSB4500_REG_GPIO_SW_GPIO              0x2D
#define  STUSB4500_REG_Device_ID                 0x2F
#define  STUSB4500_REG_RX_HEADER_LOW             0x31
#define  STUSB4500_REG_RX_HEADER_HIGH            0x32
#define  STUSB4500_REG_RX_DATA_OBJ1_0            0x33
#define  STUSB4500_REG_RX_DATA_OBJ1_1            0x34
#define  STUSB4500_REG_RX_DATA_OBJ1_2            0x35
#define  STUSB4500_REG_RX_DATA_OBJ1_3            0x36
#define  STUSB4500_REG_RX_DATA_OBJ2_0            0x37
#define  STUSB4500_REG_RX_DATA_OBJ2_1            0x38
#define  STUSB4500_REG_RX_DATA_OBJ2_2            0x39
#define  STUSB4500_REG_RX_DATA_OBJ2_3            0x3A
#define  STUSB4500_REG_RX_DATA_OBJ3_0            0x3B
#define  STUSB4500_REG_RX_DATA_OBJ3_1            0x3C
#define  STUSB4500_REG_RX_DATA_OBJ3_2            0x3D
#define  STUSB4500_REG_RX_DATA_OBJ3_3            0x3E
#define  STUSB4500_REG_RX_DATA_OBJ4_0            0x3F
#define  STUSB4500_REG_RX_DATA_OBJ4_1            0x40
#define  STUSB4500_REG_RX_DATA_OBJ4_2            0x41
#define  STUSB4500_REG_RX_DATA_OBJ4_3            0x42
#define  STUSB4500_REG_RX_DATA_OBJ5_0            0x43
#define  STUSB4500_REG_RX_DATA_OBJ5_1            0x44
#define  STUSB4500_REG_RX_DATA_OBJ5_2            0x45
#define  STUSB4500_REG_RX_DATA_OBJ5_3            0x46
#define  STUSB4500_REG_RX_DATA_OBJ6_0            0x47
#define  STUSB4500_REG_RX_DATA_OBJ6_1            0x48
#define  STUSB4500_REG_RX_DATA_OBJ6_2            0x49
#define  STUSB4500_REG_RX_DATA_OBJ6_3            0x4A
#define  STUSB4500_REG_RX_DATA_OBJ7_0            0x4B
#define  STUSB4500_REG_RX_DATA_OBJ7_1            0x4C
#define  STUSB4500_REG_RX_DATA_OBJ7_2            0x4D
#define  STUSB4500_REG_RX_DATA_OBJ7_3            0x4E
#define  STUSB4500_REG_TX_HEADER_LOW             0x51
#define  STUSB4500_REG_TX_HEADER_HIGH            0x52
#define  STUSB4500_REG_DPM_PDO_NUMB              0x70
#define  STUSB4500_REG_DPM_SNK_PDO1_0            0x85
#define  STUSB4500_REG_DPM_SNK_PDO1_1            0x86
#define  STUSB4500_REG_DPM_SNK_PDO1_2            0x87
#define  STUSB4500_REG_DPM_SNK_PDO1_3            0x88
#define  STUSB4500_REG_DPM_SNK_PDO2_0            0x89
#define  STUSB4500_REG_DPM_SNK_PDO2_1            0x8A
#define  STUSB4500_REG_DPM_SNK_PDO2_2            0x8B
#define  STUSB4500_REG_DPM_SNK_PDO2_3            0x8C
#define  STUSB4500_REG_DPM_SNK_PDO3_0            0x8D
#define  STUSB4500_REG_DPM_SNK_PDO3_1            0x8E
#define  STUSB4500_REG_DPM_SNK_PDO3_2            0x8F
#define  STUSB4500_REG_DPM_SNK_PDO3_3            0x90
#define  STUSB4500_REG_RDO_REG_STATUS_0          0x91
#define  STUSB4500_REG_RDO_REG_STATUS_1          0x92
#define  STUSB4500_REG_RDO_REG_STATUS_2          0x93
#define  STUSB4500_REG_RDO_REG_STATUS_3          0x94
/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: stusb4500_register
 *
 * Description:
 *   Register the STUSB4500 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/stusb4500_0"
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             STUSB4500
 *   addr    - The I2C address of the STUSB4500.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int stusb4500_register(FAR const char* devpath, FAR struct i2c_master_s* i2c,
                       uint8_t addr);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_USB_FUSB303_H */
