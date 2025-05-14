/****************************************************************************
 * drivers/sensors/icm42688.c
 *
 * SPDX-License-Identifier: Apache-2.0
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
 * TODO: Theory of Operation
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <debug.h>
#include <string.h>
#include <limits.h>
#include <nuttx/bits.h>
#include <nuttx/mutex.h>
#include <nuttx/signal.h>

#include <nuttx/compiler.h>
#include <nuttx/kmalloc.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/fs/fs.h>
#include <nuttx/sensors/icm42688.h>
#include <nuttx/sensors/ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Creates a mask of @m bits, i.e. MASK(2) -> 00000011 */

#define MASK(m) (BIT(m) - 1)

/* Masks and shifts @v into bit field @m */

#define TO_BITFIELD(m, v) (((v) & MASK(m##__WIDTH)) << (m##__SHIFT))

/* Un-masks and un-shifts bit field @m from @v */

#define FROM_BITFIELD(m, v) (((v) >> (m##__SHIFT)) & MASK(m##__WIDTH))

#define CONFIG_ICM42688_I2C_FREQ 400000

/****************************************************************************
 * Private Types
 ****************************************************************************/
/* Describes the icm42688 sensor register file. This structure reflects
 * the underlying hardware, so don't change it!
 */

begin_packed_struct struct sensor_data_s
{
  int16_t temp;
  int16_t x_accel;
  int16_t y_accel;
  int16_t z_accel;
  int16_t x_gyro;
  int16_t y_gyro;
  int16_t z_gyro;
} end_packed_struct;

/* Used by the driver to manage the device */

struct icm42688_dev_s
{
  mutex_t lock;                    /* mutex for this structure */
  struct icm42688_config_s config; /* board-specific information */

  struct sensor_data_s buf; /* temporary buffer (for read(), etc.) */
  size_t bufpos;            /* cursor into @buf, in bytes (!) */

  uint8_t smplrt_div;  /* divider to control sample rate */
  uint8_t afs_sel;     /* full scale range of the accelerometer */
  uint8_t dlpf_config; /* digital low pass filter configuration */
  bool fifo_enabled;   /* current enable state of FIFO buffer */
  float sample_rate;   /* current sample rate */
};

/****************************************************************************
 * Private Function Function Prototypes
 ****************************************************************************/

static int icm42688_open(FAR struct file *filep);
static int icm42688_close(FAR struct file *filep);
static ssize_t icm42688_read(FAR struct file *filep, FAR char *buf, size_t len);
static ssize_t icm42688_write(FAR struct file *filep, FAR const char *buf,
                              size_t len);
static off_t icm42688_seek(FAR struct file *filep, off_t offset, int whence);
static int icm42688_ioctl(FAR struct file *filep, int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_icm42688_fops =
    {
        icm42688_open,  /* open */
        icm42688_close, /* close */
        icm42688_read,  /* read */
        icm42688_write, /* write */
        icm42688_seek,  /* seek */
        icm42688_ioctl, /* ioctl */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/
/* NOTE :
 *
 * In all of the following code, functions named with a double leading
 * underscore '__' must be invoked ONLY if the icm42688_dev_s lock is
 * already held. Failure to do this might cause the transaction to get
 * interrupted, which will likely confuse the data you get back.
 *
 * The icm42688_dev_s lock is NOT the same thing as, i.e. the SPI master
 * interface lock: the latter protects the bus interface hardware
 * (which may have other SPI devices attached), the former protects
 * the chip and its associated data.
 */

/* __icm42688_read_reg(), but for i2c-connected devices. */

static int __icm42688_read_reg_i2c(FAR struct icm42688_dev_s *dev,
                                   uint8_t reg_addr,
                                   FAR uint8_t *buf, uint8_t len)
{
  int ret;
  struct i2c_msg_s msg[2];

  msg[0].frequency = CONFIG_ICM42688_I2C_FREQ;
  msg[0].addr = dev->config.addr;
  msg[0].flags = I2C_M_NOSTOP;
  msg[0].buffer = &reg_addr;
  msg[0].length = 1;

  msg[1].frequency = CONFIG_ICM42688_I2C_FREQ;
  msg[1].addr = dev->config.addr;
  msg[1].flags = I2C_M_READ;
  msg[1].buffer = buf;
  msg[1].length = len;

  ret = I2C_TRANSFER(dev->config.i2c, msg, 2);
  if (ret < 0)
  {
    snerr("ERROR: I2C_TRANSFER(read) failed: %d\n", ret);
    return ret;
  }

  return OK;
}

static int __icm42688_write_reg_i2c(FAR struct icm42688_dev_s *dev,
                                    uint8_t reg_addr,
                                    FAR const uint8_t *buf, uint8_t len)
{
  int ret;
  struct i2c_msg_s msg[2];

  msg[0].frequency = CONFIG_ICM42688_I2C_FREQ;
  msg[0].addr = dev->config.addr;
  msg[0].flags = I2C_M_NOSTOP;
  msg[0].buffer = &reg_addr;
  msg[0].length = 1;
  msg[1].frequency = CONFIG_ICM42688_I2C_FREQ;
  msg[1].addr = dev->config.addr;
  msg[1].flags = I2C_M_NOSTART;
  msg[1].buffer = (FAR uint8_t *)buf;
  msg[1].length = len;
  ret = I2C_TRANSFER(dev->config.i2c, msg, 2);
  if (ret < 0)
  {
    snerr("ERROR: I2C_TRANSFER(write) failed: %d\n", ret);
    return ret;
  }

  return OK;
}

/* __icm42688_read_reg()
 *
 * Reads a block of @len byte-wide registers, starting at @reg_addr,
 * from the device connected to @dev. Bytes are returned in @buf,
 * which must have a capacity of at least @len bytes.
 *
 * Note: The caller must hold @dev->lock before calling this function.
 *
 * Returns number of bytes read, or a negative errno.
 */

static inline int __icm42688_read_reg(FAR struct icm42688_dev_s *dev,
                                      enum icm42688_regaddr_e reg_addr,
                                      FAR uint8_t *buf, uint8_t len)
{
  /* If we're wired to I2C, use that function. */

  if (dev->config.i2c != NULL)
  {
    return __icm42688_read_reg_i2c(dev, reg_addr, buf, len);
  }

  /* If we get this far, it's because we can't "find" our device. */

  return -ENODEV;
}

/* __icm42688_write_reg()
 *
 * Writes a block of @len byte-wide registers, starting at @reg_addr,
 * using the values in @buf to the device connected to @dev. Register
 * values are taken in numerical order from @buf, i.e.:
 *
 *   buf[0] -> register[@reg_addr]
 *   buf[1] -> register[@reg_addr + 1]
 *   ...
 *
 * Note: The caller must hold @dev->lock before calling this function.
 *
 * Returns number of bytes written, or a negative errno.
 */

static inline int __icm42688_write_reg(FAR struct icm42688_dev_s *dev,
                                       enum icm42688_regaddr_e reg_addr,
                                       FAR const uint8_t *buf, uint8_t len)
{
  if (dev->config.i2c != NULL)
  {
    return __icm42688_write_reg_i2c(dev, reg_addr, buf, len);
  }

  /* If we get this far, it's because we can't "find" our device. */

  return -ENODEV;
}

/* __icm42688_read_imu()
 *
 * Reads the whole IMU data file from @dev in one uninterrupted pass,
 * placing the sampled values into @buf. This function is the only way
 * to guarantee that the measured values are sampled as closely-spaced
 * in time as the hardware permits, which is almost always what you
 * want.
 */

static inline int __icm42688_read_imu(FAR struct icm42688_dev_s *dev,
                                      FAR struct sensor_data_s *buf)
{
  if (dev->fifo_enabled)
  {
    return __icm42688_read_reg(dev, ICM42688_FIFO_DATA, (FAR uint8_t *)buf, sizeof(*buf));
  }

  return __icm42688_read_reg(dev, ICM42688_TEMP_DATA1, (FAR uint8_t *)buf, sizeof(*buf));
}

/* __icm42688_read_pwr_mgmt_0()
 *
 * Returns the value of the PWR_MGMT_1 register from @dev.
 */

static inline uint8_t __icm42688_read_pwr_mgmt_0(FAR struct icm42688_dev_s *dev)
{
  uint8_t buf = 0xff;
  __icm42688_read_reg(dev, ICM42688_PWR_MGMT0, &buf, sizeof(buf));
  return buf;
}

static inline int __icm42688_write_signal_path_reset(FAR struct icm42688_dev_s *dev,
                                                     uint8_t val)
{
  return __icm42688_write_reg(dev, ICM42688_SIGNAL_PATH_RESET, &val, sizeof(val));
}

// static inline int __icm42688_write_int_pin_cfg(FAR struct icm42688_dev_s *dev,
//                                                uint8_t val)
// {
//   return __icm42688_write_reg(dev, INT_PIN_CFG, &val, sizeof(val));
// }

static inline int __icm42688_write_pwr_mgmt_0(FAR struct icm42688_dev_s *dev,
                                              uint8_t val)
{
  return __icm42688_write_reg(dev, ICM42688_PWR_MGMT0, &val, sizeof(val));
}

static inline int __icm42688_write_reg_bank_sel(FAR struct icm42688_dev_s *dev,
                                                uint8_t val)
{
  return __icm42688_write_reg(dev, ICM42688_REG_BANK_SEL, &val, sizeof(val));
}

/* __icm42688_write_gyro_config() :
 *
 * Sets the @fs_sel bit in GYRO_CONFIG to the value provided. Per the
 * datasheet, the meaning of @fs_sel is as follows:
 *
 * GYRO_CONFIG(0x1b) :   XG_ST YG_ST ZG_ST FS_SEL1 FS_SEL0 x  x  x
 *
 *    XG_ST, YG_ST, ZG_ST  :  self-test (unsupported in this driver)
 *         1 -> activate self-test on X, Y, and/or Z gyros
 *
 *    FS_SEL[10] : full-scale range select
 *         0 -> ±  250 deg/sec
 *         1 -> ±  500 deg/sec
 *         2 -> ± 1000 deg/sec
 *         3 -> ± 2000 deg/sec
 */

// static inline int __icm42688_write_gyro_config(FAR struct icm42688_dev_s *dev,
//                                                uint8_t fs_sel)
// {
//   uint8_t val = TO_BITFIELD(GYRO_CONFIG__FS_SEL, fs_sel);
//   return __icm42688_write_reg(dev, GYRO_CONFIG, &val, sizeof(val));
// }

/* __icm42688_write_accel_config() :
 *
 * Sets the @afs_sel bit in ACCEL_CONFIG to the value provided. Per
 * the datasheet, the meaning of @afs_sel is as follows:
 *
 * ACCEL_CONFIG(0x1c) :   XA_ST YA_ST ZA_ST AFS_SEL1 AFS_SEL0 x  x  x
 *
 *    XA_ST, YA_ST, ZA_ST  :  self-test (unsupported in this driver)
 *         1 -> activate self-test on X, Y, and/or Z accelerometers
 *
 *    AFS_SEL[10] : full-scale range select
 *         0 -> ±  2 g
 *         1 -> ±  4 g
 *         2 -> ±  8 g
 *         3 -> ± 16 g
 */

// static inline int __icm42688_write_accel_config(FAR struct icm42688_dev_s *dev,
//                                                 uint8_t afs_sel)
// {
//   uint8_t val;
//   if (afs_sel > 3)
//   {
//     snerr("ERROR: Invalid AFS_SEL value\n");
//     return -EINVAL;
//   }

//   val = TO_BITFIELD(ACCEL_CONFIG__AFS_SEL, afs_sel);
//   return __icm42688_write_reg(dev, ACCEL_CONFIG, &val, sizeof(val));
// }

/* CONFIG (0x1a) :   x   x   EXT_SYNC_SET[2..0] DLPF_CFG[2..0]
 *
 *    EXT_SYNC_SET  : frame sync bit position
 *    DLPF_CFG      : digital low-pass filter bandwidth
 * (see datasheet, it's ... complicated)
 */

// static inline int __icm42688_write_config(FAR struct icm42688_dev_s *dev,
//                                           uint8_t ext_sync_set, uint8_t dlpf_cfg)
// {
//   uint8_t val = TO_BITFIELD(CONFIG__EXT_SYNC_SET, ext_sync_set) |
//                 TO_BITFIELD(CONFIG__DLPF_CFG, dlpf_cfg);
//   return __icm42688_write_reg(dev, CONFIG, &val, sizeof(val));
// }

/* Sets the SMPLRT_DIV that controls the sample rate. */

// static inline int __icm42688_set_sample_rate_divider(FAR struct icm42688_dev_s *dev,
//                                                      uint8_t val)
// {
//   return __icm42688_write_reg(dev, SMPLRT_DIV, &val, sizeof(val));
// }

// /* Reads current sample rate. Value is updated to icm42688_dev_s->sample_rate. */

// static inline int __icm42688_read_sample_rate(FAR struct icm42688_dev_s *dev)
// {
//   int ret;
//   float gyro_output_rate = 1000.0f;

//   ret = __icm42688_read_reg(dev, SMPLRT_DIV, &dev->smplrt_div,
//                             sizeof(dev->smplrt_div));
//   if (ret < 0)
//   {
//     return ret;
//   }

//   ret = __icm42688_read_reg(dev, CONFIG, &dev->dlpf_config,
//                             sizeof(dev->dlpf_config));
//   if (ret < 0)
//   {
//     return ret;
//   }

//   dev->dlpf_config = TO_BITFIELD(CONFIG__DLPF_CFG, dev->dlpf_config);

//   /* This condition verifies if DLPF is disabled */

//   if ((dev->dlpf_config == 0) || (dev->dlpf_config == 7))
//   {
//     gyro_output_rate = 8000.0f;
//   }

//   dev->sample_rate = gyro_output_rate / (float)(1 + dev->smplrt_div);

//   return OK;
// }

/* Read the number of bytes currently in FIFO buffer. */

// static inline int __icm42688_read_fifo_count(FAR struct icm42688_dev_s *dev,
//                                              uint16_t *buf)
// {
//   int ret;
//   uint8_t fifo_counter[2];
//   ret = __icm42688_read_reg(dev, FIFO_COUNTH, fifo_counter, sizeof(fifo_counter));
//   if (ret < 0)
//   {
//     snerr("ERROR: Failed to read FIFO counter\n");
//     *buf = 0;
//   }
//   else
//   {
//     *buf = (fifo_counter[0] << 8) | fifo_counter[1];
//   }

//   return ret;
// }

/* Enables or disables FIFO loading a specific sensor.
 * It may receive a OR combination of multiple sensors.
 * Example:
 * __icm42688_set_fifo(priv, FIFO_EN__TEMP | FIFO_EN__YG | FIFO_EN__ACCEL);
 */

// static inline int __icm42688_set_fifo(FAR struct icm42688_dev_s *dev,
//                                       uint8_t val)
// {
//   return __icm42688_write_reg(dev, FIFO_EN, &val, sizeof(val));
// }

/* Sets USER CONTROL register. It may receive an OR combination of multiple
 * bitfields.
 * Example:
 * __icm42688_user_control(priv, USER_CTRL__FIFO_EN | USER_CTRL__I2C_MST_RESET);
 */

// static inline int __icm42688_user_control(FAR struct icm42688_dev_s *dev,
//                                           uint8_t val)
// {
//   return __icm42688_write_reg(dev, USER_CTRL, &val, sizeof(val));
// }

/* Resets the icm42688, sets it to a default configuration. */

static int icm42688_reset(FAR struct icm42688_dev_s *dev)
{
  int ret;
  if (dev->config.i2c == NULL)
  {
    return -EINVAL;
  }

  nxmutex_lock(&dev->lock);

  __icm42688_write_reg_bank_sel(dev, 0X00);
  /* Awaken chip, issue hardware reset */
  __icm42688_write_pwr_mgmt_0(dev, 0X0F);
  if (ret < 0)
  {
    nxmutex_unlock(&dev->lock);
    snerr("Could not find icm42688!\n");
    return ret;
  }
  nxsig_usleep(2000);

  /* default No FSYNC, set accel LPF at 184 Hz, gyro LPF at 188 Hz in
   * menuconfig
   */

  // __icm42688_write_config(dev, CONFIG_icm42688_EXT_SYNC_SET,
  //                         CONFIG_icm42688_DLPF_CFG);
  // dev->dlpf_config = CONFIG_icm42688_DLPF_CFG;

  // /* default ± 1000 deg/sec in menuconfig */

  // __icm42688_write_gyro_config(dev, CONFIG_icm42688_GYRO_FS_SEL);

  // /* default ± 8g in menuconfig */

  // __icm42688_write_accel_config(dev, CONFIG_icm42688_ACCEL_AFS_SEL);
  // dev->afs_sel = CONFIG_icm42688_ACCEL_AFS_SEL;

  // /* clear INT on any read (we aren't using that pin right now) */

  // __icm42688_write_int_pin_cfg(dev, INT_PIN_CFG__INT_RD_CLEAR);

  // /* Disable use of FIFO buffer */

  // __icm42688_set_fifo(dev, 0);
  dev->fifo_enabled = false;

  nxmutex_unlock(&dev->lock);
  return 0;
}

/****************************************************************************
 * Name: icm42688_open
 *
 * Note: we don't deal with multiple users trying to access this interface at
 * the same time. Until further notice, don't do that.
 *
 * And no, it's not as simple as just prohibiting concurrent opens or
 * reads with a mutex: there are legit reasons for truy concurrent
 * access, but they must be treated carefully in this interface lest a
 * partial reader end up with a mixture of old and new samples. This
 * will make some users unhappy.
 *
 ****************************************************************************/

static int icm42688_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct icm42688_dev_s *dev = inode->i_private;

  /* Reset the register cache */

  nxmutex_lock(&dev->lock);
  dev->bufpos = 0;
  nxmutex_unlock(&dev->lock);

  return 0;
}

/****************************************************************************
 * Name: icm42688_close
 ****************************************************************************/

static int icm42688_close(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct icm42688_dev_s *dev = inode->i_private;

  /* Reset (clear) the register cache. */

  nxmutex_lock(&dev->lock);
  dev->bufpos = 0;
  nxmutex_unlock(&dev->lock);

  return 0;
}

/****************************************************************************
 * Name: icm42688_read
 *
 * Returns a snapshot of the accelerometer, temperature, and gyro registers.
 *
 * Note: the chip uses traditional, twos-complement notation, i.e. "0"
 * is encoded as 0, and full-scale-negative is 0x8000, and
 * full-scale-positive is 0x7fff. If we read the registers
 * sequentially and directly into memory (as we do), the measurements
 * from each sensor are captured as big endian words.
 *
 * In contrast, ASN.1 maps "0" to 0x8000, full-scale-negative to 0,
 * and full-scale-positive to 0xffff. So if we want to send in a
 * format that an ASN.1 PER-decoder would recognize, must:
 *
 *   1. Treat the register data/measurements as unsigned,
 *   2. Add 0x8000 to each measurement, and then,
 *   3. Send each word in big-endian order.
 *
 * The result of the above will be something you could neatly describe
 * like this (confirmed with asn1scc):
 *
 *    Sint16  ::= INTEGER(-32768..32767)
 *
 *    icm42688Sample ::= SEQUENCE
 *    {
 *      accel-X  Sint16,
 *      accel-Y  Sint16,
 *      accel-Z  Sint16,
 *      temp     Sint16,
 *      gyro-X   Sint16,
 *      gyro-Y   Sint16,
 *      gyro-Z   Sint16
 *    }
 *
 ****************************************************************************/

static ssize_t icm42688_read(FAR struct file *filep, FAR char *buf, size_t len)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct icm42688_dev_s *dev = inode->i_private;
  size_t send_len = 0;

  nxmutex_lock(&dev->lock);

  /* Populate the register cache if it seems empty. */

  if (!dev->bufpos)
  {
    __icm42688_read_imu(dev, &dev->buf);
  }

  /* Send the lesser of: available bytes, or amount requested. */

  send_len = sizeof(dev->buf) - dev->bufpos;
  if (send_len > len)
  {
    send_len = len;
  }

  if (send_len)
  {
    memcpy(buf, ((FAR uint8_t *)&dev->buf) + dev->bufpos, send_len);
  }

  /* Move the cursor, to mark them as sent. */

  dev->bufpos += send_len;

  /* If we've sent the last byte, reset the buffer. */

  if (dev->bufpos >= sizeof(dev->buf))
  {
    dev->bufpos = 0;
  }

  nxmutex_unlock(&dev->lock);
  return send_len;
}

/****************************************************************************
 * Name: icm42688_write
 ****************************************************************************/

static ssize_t icm42688_write(FAR struct file *filep, FAR const char *buf,
                              size_t len)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct icm42688_dev_s *dev = inode->i_private;

  UNUSED(inode);
  UNUSED(dev);
  snerr("ERROR: %p %p %zu\n", inode, dev, len);

  return len;
}

/****************************************************************************
 * Name: icm42688_seek
 ****************************************************************************/

static off_t icm42688_seek(FAR struct file *filep, off_t offset, int whence)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct icm42688_dev_s *dev = inode->i_private;

  UNUSED(inode);
  UNUSED(dev);

  snerr("ERROR: %p %p\n", inode, dev);

  return 0;
}

/****************************************************************************
 * Name: icm42688_ioctl
 ****************************************************************************/

static int icm42688_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct icm42688_dev_s *priv = inode->i_private;
  uint8_t write_data = (uint8_t)arg;
  int ret = OK;

  switch (cmd)
  {
    /* Sets the accelerometer full scale range. Arg: uin8_t value */

  case SNIOC_SET_AFS_SEL:
    // ret = __icm42688_write_accel_config(priv, write_data);
    // if (ret < 0)
    // {
    //   snerr("ERROR: SNIOC_SET_AFS_SEL fails. Returns: %d\n", ret);
    // }
    // else
    // {
    //   priv->afs_sel = write_data;
    //   sninfo("SNIOC_SET_AFS_SEL: %d Returns: %d\n", priv->afs_sel,
    //          ret);
    // }
    break;

    /* Sets the sample rate divider. Arg: uin8_t value */

  case SNIOC_SMPLRT_DIV:
    // ret = __icm42688_set_sample_rate_divider(priv, write_data);
    // priv->smplrt_div = write_data;
    // sninfo("SNIOC_SMPLRT_DIV: %d Returns: %d\n", priv->smplrt_div, ret);
    break;

    /* Read current sample rate. Arg: uin32_t* pointer */

  case SNIOC_READ_SAMPLE_RATE:
  {
    // FAR uint32_t *ptr = (FAR uint32_t *)((uintptr_t)arg);
    // ret = __icm42688_read_sample_rate(priv);
    // sninfo("SNIOC_READ_SAMPLE_RATE: Returns: %d. Read: %f\n",
    //        ret, priv->sample_rate);
    // *ptr = (uint32_t)priv->sample_rate;
    break;
  }

    /* Read current number of bytes in FIFO buffer. Arg: uin16_t* */

  case SNIOC_READ_FIFO_COUNT:
  {
    // FAR uint16_t *ptr = (FAR uint16_t *)((uintptr_t)arg);
    // uint16_t fifo_count = 0;
    // ret = __icm42688_read_fifo_count(priv, &fifo_count);
    // *ptr = fifo_count;
    // sninfo("SNIOC_READ_FIFO_COUNT: Returns: %d. Read: 0x%x\n",
    //        ret, fifo_count);
    break;
  }

    /* Enable or disable the use of FIFO buffer. Arg: bool* */

  case SNIOC_ENABLE_FIFO:
    // if (!write_data)
    // {
    //   ret = __icm42688_set_fifo(priv, 0);
    //   if (ret < 0)
    //   {
    //     sninfo("SNIOC_ENABLE_FIFO failed. Returns: %d\n", ret);
    //   }

    //   ret = __icm42688_user_control(priv, 0);
    //   priv->fifo_enabled = false;
    // }
    // else
    // {
    //   ret = __icm42688_user_control(priv, USER_CTRL__FIFO_EN);
    //   if (ret < 0)
    //   {
    //     sninfo("SNIOC_ENABLE_FIFO failed. Returns: %d\n", ret);
    //   }

    //   /* This configuration enables temperature, accelerometer and
    //    * gyro on all three axis. Each read requires 14 bytes, allowing
    //    * the FIFO to store 1024/14 = 73 samples.
    //    */

    //   ret = __icm42688_set_fifo(priv, FIFO_EN__TEMP | FIFO_EN__XG |
    //                                       FIFO_EN__YG | FIFO_EN__ZG | FIFO_EN__ACCEL);
    //   priv->fifo_enabled = true;
    // }

    // sninfo("SNIOC_ENABLE_FIFO: %d Returns: %d\n", write_data, ret);
    break;

  default:
    sninfo("Unrecognized IOCTL command: 0x%04x\n", cmd);
    ret = -ENOTTY;
    break;
  }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: icm42688_register
 *
 * Description:
 *   Registers the icm42688 interface as 'devpath'
 *
 * Input Parameters:
 *   devpath  - The full path to the interface to register. E.g., "/dev/imu0"
 *   spi      - SPI interface for chip communications
 *   config   - Configuration information
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int icm42688_register(FAR const char *path, FAR struct icm42688_config_s *config)
{
  FAR struct icm42688_dev_s *priv;
  int ret;

  /* Without config info, we can't do anything. */

  if (config == NULL)
  {
    return -EINVAL;
  }

  /* Initialize the device structure. */

  priv = kmm_malloc(sizeof(struct icm42688_dev_s));
  if (priv == NULL)
  {
    snerr("ERROR: Failed to allocate icm42688 device instance\n");
    return -ENOMEM;
  }

  memset(priv, 0, sizeof(*priv));
  nxmutex_init(&priv->lock);

  /* Keep a copy of the config structure, in case the caller discards
   * theirs.
   */

  priv->config = *config;

  /* Reset the chip, to give it an initial configuration. */

  ret = icm42688_reset(priv);
  if (ret < 0)
  {
    snerr("ERROR: Failed to configure icm42688: %d\n", ret);

    nxmutex_destroy(&priv->lock);
    kmm_free(priv);
    return ret;
  }

  /* Register the device node. */

  ret = register_driver(path, &g_icm42688_fops, 0666, priv);
  if (ret < 0)
  {
    snerr("ERROR: Failed to register icm42688 interface: %d\n", ret);

    nxmutex_destroy(&priv->lock);
    kmm_free(priv);
    return ret;
  }

  return OK;
}
