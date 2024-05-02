/****************************************************************************
 * drivers/usbmisc/stusb4500.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <assert.h>
#include <errno.h>
#include <poll.h>
#include <debug.h>

#include <nuttx/compiler.h>
#include <nuttx/fs/fs.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/i2c/i2c_master.h>

#include <nuttx/usb/stusb4500.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_DEBUG_STUSB4500
  #define stusb4500_err(x, ...)        _err(x, ##__VA_ARGS__)
  #define stusb4500_info(x, ...)       _info(x, ##__VA_ARGS__)
#else
  #define stusb4500_err(x, ...)        uerr(x, ##__VA_ARGS__)
  #define stusb4500_info(x, ...)       uinfo(x, ##__VA_ARGS__)
#endif

#ifndef CONFIG_STUSB4500_I2C_FREQUENCY
  #define CONFIG_STUSB4500_I2C_FREQUENCY 400000
#endif

/* Other macros */

#define STUSB4500_I2C_RETRIES  10

/* Debug */

#ifdef CONFIG_DEBUG_STUSB4500

#endif

/****************************************************************************
 * Private Data Types
 ****************************************************************************/

struct stusb4500_dev_s
{
  FAR struct i2c_master_s* i2c;         /* I2C interface */
  uint8_t addr;                         /* I2C address */
  mutex_t devlock;                      /* Manages exclusive access */
};

/****************************************************************************
 * Private Function prototypes
 ****************************************************************************/

#ifdef CONFIG_DEBUG_STUSB4500

#endif
static int stusb4500_open(FAR struct file* filep);
static int stusb4500_close(FAR struct file* filep);
static ssize_t stusb4500_read(FAR struct file*, FAR char*, size_t);
static ssize_t stusb4500_write(FAR struct file* filep, FAR const char* buffer,
                               size_t buflen);
static int stusb4500_ioctl(FAR struct file* filep, int cmd, unsigned long arg);


/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_stusb4500ops =
{
  stusb4500_open,  /* open */
  stusb4500_close, /* close */
  stusb4500_read,  /* read */
  stusb4500_write, /* write */
  NULL,          /* seek */
  stusb4500_ioctl, /* ioctl */
  NULL,          /* mmap */
  NULL,          /* truncate */
  NULL           /* poll */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stusb4500_getreg
 *
 * Description:
 *   Read from an 8-bit STUSB4500 register
 *
 * Input Parameters:
 *   priv   - pointer to STUSB4500 Private Structure
 *   reg    - register to read
 *
 * Returned Value:
 *   Returns positive register value in case of success, otherwise ERROR
 *
 ****************************************************************************/

static int stusb4500_getreg(FAR struct stusb4500_dev_s* priv, uint8_t reg)
{
  int ret = -EIO;
  int retries;
  uint8_t regval;
  struct i2c_msg_s msg[2];

  DEBUGASSERT(priv);

  msg[0].frequency = CONFIG_STUSB4500_I2C_FREQUENCY;
  msg[0].addr      = priv->addr;
  msg[0].flags     = 0;
  msg[0].buffer    = &reg;
  msg[0].length    = 1;

  msg[1].frequency = CONFIG_STUSB4500_I2C_FREQUENCY;
  msg[1].addr      = priv->addr;
  msg[1].flags     = I2C_M_READ;
  msg[1].buffer    = &regval;
  msg[1].length    = 1;

  /* Perform the transfer */

  for (retries = 0; retries < STUSB4500_I2C_RETRIES; retries++)
    {
      ret = I2C_TRANSFER(priv->i2c, msg, 2);
      if (ret >= 0)
        {
          stusb4500_info("reg:%02X, value:%02X\n", reg, regval);
          return regval;
        }
      else
        {
          /* Some error. Try to reset I2C bus and keep trying. */

#ifdef CONFIG_I2C_RESET
          if (retries == STUSB4500_I2C_RETRIES - 1)
            {
              break;
            }

          ret = I2C_RESET(priv->i2c);
          if (ret < 0)
            {
              stusb4500_err("ERROR: I2C_RESET failed: %d\n", ret);
              return ret;
            }
#endif
        }
    }

  stusb4500_info("reg:%02X, error:%d\n", reg, ret);
  return ret;
}

/****************************************************************************
 * Name: stusb4500_putreg
 *
 * Description:
 *   Write a value to an 8-bit STUSB4500 register
 *
 * Input Parameters:
 *   priv    - pointer to STUSB4500 Private Structure
 *   regaddr - register to read
 *   regval  - value to be written
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static int stusb4500_putreg(FAR struct stusb4500_dev_s* priv, uint8_t regaddr,
                            uint8_t regval)
{
  int ret = -EIO;
  int retries;
  struct i2c_msg_s msg;
  uint8_t txbuffer[2];

  /* Setup to the data to be transferred (register address and data). */

  txbuffer[0]   = regaddr;
  txbuffer[1]   = regval;

  /* Setup 8-bit STUSB4500 address write message */

  msg.frequency = CONFIG_STUSB4500_I2C_FREQUENCY;
  msg.addr      = priv->addr;
  msg.flags     = 0;
  msg.buffer    = txbuffer;
  msg.length    = 2;

  /* Perform the transfer */

  for (retries = 0; retries < STUSB4500_I2C_RETRIES; retries++)
    {
      ret = I2C_TRANSFER(priv->i2c, &msg, 1);
      if (ret == OK)
        {
          stusb4500_info("reg:%02X, value:%02X\n", regaddr, regval);

          return OK;
        }
      else
        {
          /* Some error. Try to reset I2C bus and keep trying. */

#ifdef CONFIG_I2C_RESET
          if (retries == STUSB4500_I2C_RETRIES - 1)
            {
              break;
            }

          ret = I2C_RESET(priv->i2c);
          if (ret < 0)
            {
              stusb4500_err("ERROR: I2C_RESET failed: %d\n", ret);
              return ret;
            }
#endif
        }
    }

  stusb4500_err("ERROR: failed reg:%02X, value:%02X, error:%d\n",
                regaddr, regval, ret);
  return ret;
}

/****************************************************************************
 * Name: stusb4500_read_device_id
 *
 * Description:
 *   Read device ID.
 *
 ****************************************************************************/

static int stusb4500_read_device_id(FAR struct stusb4500_dev_s* priv,
                                    FAR uint8_t* dev_id)
{
  int ret;

  ret = stusb4500_getreg(priv, STUSB4500_REG_Device_ID);
  if (ret < 0)
    {
      stusb4500_err("ERROR: Failed to read device ID\n");
      return -EIO;
    }

  if (dev_id != NULL)
    {
      *dev_id = ret;
    }

  return ret;
}

/****************************************************************************
 * Name: stusb4500_open
 *
 * Description:
 *   This function is called whenever the STUSB4500 device is opened.
 *
 ****************************************************************************/

static int stusb4500_open(FAR struct file* filep)
{
  FAR struct inode* inode = filep->f_inode;
  FAR struct stusb4500_dev_s* priv = inode->i_private;
  uint8_t dev_id;
  int ret;

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      return ret;
    }

  /* Probe device */

  ret = stusb4500_read_device_id(priv, &dev_id);
  if (ret < 0)
    {
      stusb4500_err("ERROR: No response at given address 0x%02X\n",
                    priv->addr);
      ret = -EFAULT;
    }
  else
    {
      stusb4500_info("device id: 0x%02X type: 0x%02X\n", dev_id);
    }

  nxmutex_unlock(&priv->devlock);
  return ret;
}

/****************************************************************************
 * Name: stusb4500_close
 *
 * Description:
 *   This routine is called when the STUSB4500 device is closed.
 *
 ****************************************************************************/

static int stusb4500_close(FAR struct file* filep)
{
  FAR struct inode* inode = filep->f_inode;
  FAR struct stusb4500_dev_s* priv = inode->i_private;
  int ret;

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      return ret;
    }

  /* add closing code here */

  nxmutex_unlock(&priv->devlock);
  return OK;
}

/****************************************************************************
 * Name: stusb4500_read
 *
 * Description:
 *   This routine is called when the STUSB4500 device is read.
 *
 ****************************************************************************/

static ssize_t stusb4500_read(FAR struct file* filep, FAR char* buffer,
                              size_t buflen)
{

  return 0;
}

/****************************************************************************
 * Name: stusb4500_write
 *
 * Description:
 *   This routine is called when the STUSB4500 device is written to.
 *
 ****************************************************************************/

static ssize_t stusb4500_write(FAR struct file* filep, FAR const char* buffer,
                               size_t buflen)
{
  ssize_t length = 0;

  return length;
}

/****************************************************************************
 * Name: stusb4500_ioctl
 *
 * Description:
 *   This routine is called when ioctl function call is performed for
 *   the STUSB4500 device.
 *
 ****************************************************************************/

static int stusb4500_ioctl(FAR struct file* filep, int cmd, unsigned long arg)
{
  FAR struct inode* inode = filep->f_inode;
  FAR struct stusb4500_dev_s* priv = inode->i_private;
  int ret;

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      return ret;
    }

  stusb4500_info("cmd: 0x%02X, arg:%lu\n", cmd, arg);

  switch (cmd)
    {
    case USBCIOC_READ_DEVID:
    {
      ret = stusb4500_read_device_id(priv, (uint8_t*)arg);
    }
    break;

    default:
    {
      stusb4500_err("ERROR: Unrecognized cmd: %d\n", cmd);
      ret = -ENOTTY;
    }
    break;
    }

  nxmutex_unlock(&priv->devlock);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int stusb4500_register(FAR const char* devpath, FAR struct i2c_master_s* i2c,
                       uint8_t addr)
{
  FAR struct stusb4500_dev_s* priv;
  int ret;

  DEBUGASSERT(devpath != NULL && i2c != NULL);

  /* Initialize the STUSB4500 device structure */

  priv = (FAR struct stusb4500_dev_s*)
         kmm_zalloc(sizeof(struct stusb4500_dev_s));
  if (!priv)
    {
      stusb4500_err("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  /* Initialize device structure mutex */

  nxmutex_init(&priv->devlock);

  priv->i2c         = i2c;
  priv->addr        = addr;

  /* Register the character driver */

  ret = register_driver(devpath, &g_stusb4500ops, 0666, priv);
  if (ret < 0)
    {
      stusb4500_err("ERROR: Failed to register driver: %d\n", ret);
      goto errout_with_priv;
    }

  return OK;

errout_with_priv:
  nxmutex_destroy(&priv->devlock);
  kmm_free(priv);
  return ret;
}
