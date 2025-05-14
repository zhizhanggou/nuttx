#include <debug.h>
#include "stm32_gpio.h"
#include "stm32f411-minimum.h"
#include "stm32_i2c.h"
#include "nuttx/sensors/icm42688.h"

int stm32_icm42688_initialize(int devno, int busno)
{
  struct i2c_master_s *i2c;
  char devpath[16];
  int ret;

  sninfo("Initializing ICM42688!\n");

  /* Initialize I2C */

  i2c = stm32_i2cbus_initialize(busno);
  if (!i2c)
  {
    return -ENODEV;
  }

  /* Then register the ambient light sensor */
  struct icm42688_config_s mpuc;
  memset(&mpuc, 0, sizeof(mpuc));
  mpuc.i2c = i2c;
  mpuc.addr = 0x68;

  ret = icm42688_register("/dev/imu0", &mpuc);
  if (ret < 0)
  {
    snerr("ERROR: Error registering ICM42688 in I2C%d\n", busno);
  }
  //   snprintf(devpath, sizeof(devpath), "/dev/imu%d", devno);
  //   ret = bmi270_register(devpath, i2c, BMI270_I2C_ADDR);
  //   if (ret < 0)
  //     {
  //       snerr("ERROR: Error registering BMI270\n");
  //     }

  return ret;
}
