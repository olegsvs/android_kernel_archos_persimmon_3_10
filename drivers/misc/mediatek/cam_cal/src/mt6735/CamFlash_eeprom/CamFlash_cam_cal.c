/*
 * Driver for CAM_Flash
 *
 *
 */

#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include "kd_camera_hw.h"
#include "cam_cal.h"
#include "cam_cal_define.h"

#ifdef CAM_CAL_DEBUG
#define CAM_CALDB printk
#else
#define CAM_CALDB(x,...)
#endif



/*******************************************************************************
 *
 ********************************************************************************/

//Add by Abel 2014.11.28
#if 1 //HTC_CAM_FEATURE_FLASH_RESTRICTION
static struct kobject *led_status_obj; // tmp remove for fc-1
#endif //HTC_CAM_FEATURE_FLASH_RESTRICTION

static uint32_t led_hotspot_status_value;
static uint16_t led_low_temp_limit = 5;
static uint16_t led_low_cap_limit = 14;

static ssize_t led_hotspot_status_get(struct device *dev,
                                      struct device_attribute *attr, char *buf)
{

    int length;
    length = sprintf(buf, "%d\n", led_hotspot_status_value);
    return length;
}

static ssize_t led_hotspot_status_set(struct device *dev,
                                      struct device_attribute *attr, const char *buf, size_t count)
{

    u32 tmp = 0;

    tmp = buf[0] - 0x30; /* only get the first char*/

    led_hotspot_status_value = tmp;
    CAM_CALDB("[CAM_CAL] led_hotspot_status_value = %d\n", led_hotspot_status_value);
    return count;
}

static ssize_t low_temp_limit_get(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t length = 0;
	if(buf)
	    length = sprintf(buf, "%d\n", led_low_temp_limit);
	return length;
}

static ssize_t low_cap_limit_get(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t length = 0;
	if(buf)
	    length = sprintf(buf, "%d\n", led_low_cap_limit);
	return length;
}

static DEVICE_ATTR(led_hotspot_status, 0644,
    led_hotspot_status_get,
    led_hotspot_status_set);

static DEVICE_ATTR(low_temp_limit, 0444,
	low_temp_limit_get,
	NULL);

static DEVICE_ATTR(low_cap_limit, 0444,
	low_cap_limit_get,
	NULL);

//End Add by Abel 2014.11.28

/*******************************************************************************
 *
 ********************************************************************************/
static int __init CamFlash_CAL_init(void)
{

        // Add by Abel 2014.11.28
        int ret = 0;
        CAM_CALDB("CamFlash_Cal_i2C_inti Abel start\n");

        led_status_obj = kobject_create_and_add("camera_led_status", NULL);
        if (led_status_obj == NULL) {

            pr_info("[CAM][FL] msm_camera_led: subsystem_register failed\n");
            ret = -ENOMEM;
            goto error;
        }
        ret = sysfs_create_file(led_status_obj, &dev_attr_led_hotspot_status.attr);
        if (ret){

        pr_err("[CAM][FL] msm_camera_led: sysfs_create_file dev_attr_led_hotspot_status failed\n");
        ret = -EFAULT;
        goto error;
	}
	ret = sysfs_create_file(led_status_obj,
		&dev_attr_low_temp_limit.attr);
	if (ret) {
		pr_err("[CAM][FL] msm_camera_led: sysfs_create_file dev_attr_low_temp_limit failed\n");
		ret = -EFAULT;
		goto error;
	}
	ret = sysfs_create_file(led_status_obj,
		&dev_attr_low_cap_limit.attr);
	if (ret) {
		pr_err("[CAM][FL] msm_camera_led: sysfs_create_file dev_attr_low_cap_limit failed\n");
		ret = -EFAULT;
		goto error;
	}
    //End Add by Abel 2014.11.28
    return 0;

//Add by Abel 2014.11.28
error:
kobject_del(led_status_obj);
return ret;
//End Add by Abel 2014.11.28
}

module_init(CamFlash_CAL_init);

MODULE_DESCRIPTION("CamFlash_CAL driver");
MODULE_AUTHOR("Abel Chuang <Abel_Chuang@htc.com>");
MODULE_LICENSE("GPL");
