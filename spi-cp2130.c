/**
 * Kernel driver for CP2130 USB<->SPI bridge.
 * Copyright (C) 2016 Jochen Henneberg (jh@henneberg-systemdesign.com)
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/slab.h>
#include <linux/module.h>
#include <linux/usb.h>
#include <linux/spi/spi.h>
#include <linux/workqueue.h>
#include <linux/string.h>
#include <asm/byteorder.h>
#include <linux/interrupt.h>
#include <linux/irqdomain.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/version.h>

#define USB_DEVICE_ID_CP2130         0x87a0
#define USB_VENDOR_ID_CYGNAL         0x10c4

#define CP2130_NUM_GPIOS             11
#define CP2130_IRQ_POLL_INTERVAL     1 * 1000 * 1000 /* us */

#define CP2130_CMD_READ              0x00
#define CP2130_CMD_WRITE             0x01
#define CP2130_CMD_WRITEREAD         0x02
#define CP2130_BULK_OFFSET_CMD       2
#define CP2130_BULK_OFFSET_LENGTH    4
#define CP2130_BULK_OFFSET_DATA      8

#define CP2130_BREQ_GET_GPIO_VALUES  0x20
#define CP2130_BREQ_SET_GPIO_MODE    0x23
#define CP2130_BREQ_GET_GPIO_CS      0x24
#define CP2130_BREQ_SET_GPIO_CS      0x25
#define CP2130_BREQ_GET_SPI_WORD     0x30
#define CP2130_BREQ_SET_SPI_WORD     0x31
#define CP2130_BREQ_GET_SPI_DELAY    0x32
#define CP2130_BREQ_SET_SPI_DELAY    0x33
#define CP2130_BREQ_GET_LOCK_BYTE    0x6E
#define CP2130_BREQ_GET_PIN_CONFIG   0x6C
#define CP2130_BREQ_SET_PIN_CONFIG   0x6D

#define CP2130_BREQ_MEMORY_KEY       0xA5F1

/* cp2130 attached chip */
struct cp2130_channel {
        int updated;
        int cs_en; /* chip-select enable mode */
        int irq_pin;
        int clock_phase;
        int polarity;
        int cs_pin_mode;
        int clock_freq; /* spi clock frequency */
        int delay_mask; /* cs enable, pre-deassert,
                           post assert and inter-byte delay enable */
        int inter_byte_delay;
        int pre_deassert_delay;
        int post_assert_delay;
        char *modalias;
	void *pdata;
        struct spi_device *chip;
};

/* cp2130 OTP ROM */
struct cp2130_otprom {
        int lock_byte;
        int pin_config[CP2130_NUM_GPIOS];
        int suspend_level;
        int suspend_mode;
        int wakeup_mask;
        int wakeup_match;
        int divider;
};

struct cp2130_gpio_irq {
	struct irq_domain		*irq_domain;
	struct mutex			irq_lock;
	int				virq[CP2130_NUM_GPIOS];
	u16				irq_mask;
};

/* cp2130 device structure */
struct cp2130_device {
	struct usb_device *udev;
	struct usb_interface *intf;
        struct spi_master *spi_master;

        struct mutex chn_config_lock;
        struct mutex otprom_lock;
        struct mutex usb_bus_lock;

        struct work_struct update_chn_config;
        struct work_struct read_chn_config;
        struct work_struct update_otprom;
        struct work_struct read_otprom;
        struct cp2130_channel chn_configs[CP2130_NUM_GPIOS];
        struct cp2130_otprom otprom_config;

        struct cp2130_gpio_irq irq_chip;
        struct work_struct irq_work;

	struct gpio_chip gpio_chip;
	char *gpio_names[CP2130_NUM_GPIOS];
	u8 gpio_states[2];

        int current_channel;

        int irq_poll_interval;
};

/* USB device functions */
static int cp2130_probe(struct usb_interface *intf,
                        const struct usb_device_id *id);
static void cp2130_disconnect(struct usb_interface *intf);

static const struct usb_device_id cp2130_devices[] = {
	{ USB_DEVICE(USB_VENDOR_ID_CYGNAL, USB_DEVICE_ID_CP2130) },
	{ }
};

static struct usb_driver cp2130_driver = {
	.name			= "cp2130",
	.probe			= cp2130_probe,
	.disconnect		= cp2130_disconnect,
	.suspend		= NULL,
	.resume			= NULL,
	.reset_resume		= NULL,
	.id_table		= cp2130_devices,
	.supports_autosuspend	= 0,
};

static int __init cp2130_init(void)
{
	int ret;
	printk(KERN_DEBUG "cp2130_init\n");
	ret = usb_register_driver(&cp2130_driver, THIS_MODULE, "cp2130");
	if (ret)
		printk(KERN_ERR "can't register cp2130 driver\n");

	return ret;
}

static void __exit cp2130_exit(void)
{
	printk(KERN_DEBUG "cp2130_exit\n");
	usb_deregister(&cp2130_driver);
}

static int cp2130_spi_setup(struct spi_device *spi)
{
        return 0;
}

static void cp2130_spi_cleanup(struct spi_device *spi)
{
}

static char* cp2130_spi_speed_to_string(int reg_val)
{
        switch (reg_val) {
        case 0: return "12 MHz   ";
        case 1: return "6 MHz    ";
        case 2: return "3 MHz    ";
        case 3: return "1.5 MHz  ";
        case 4: return "750 kHz  ";
        case 5: return "375 kHz  ";
        case 6: return "187.5 kHz";
        case 7: return "93.8 kHz ";
        }
        return "";
}

/*
 * This is a workaround for older versions of linux which do not have
 * gpiochip_add(). In this module, the data parameter passed to
 * gpiochip_add_data is always the struct cp2130_device that contains the struct
 * gpio_chip that is passed as the first parameter. Based on this assumption we
 * can simulate the correct behaviour using container_of() to get the struct
 * cp2130_device from the struct gpio_chip.
 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 4, 0)
static inline struct cp2130_device *gpiochip_get_data(struct gpio_chip *chip)
{
	return container_of(chip, struct cp2130_device, gpio_chip);
}

static inline int gpiochip_add_data(struct gpio_chip *chip, void *data)
{
	return gpiochip_add(chip);
}
#endif

/*
 * This is a workaround for older versions of linux which have a __must_check
 * return type of int on the gpiochip_remove() function.
 */
static inline void gpiochip_remove_(struct gpio_chip *chip)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 17, 0)
	if (gpiochip_remove(chip) < 0)
		dev_err(&gpiochip_get_data(chip)->intf->dev,
			"failed to remove GPIO chip");
#else
	gpiochip_remove(chip);
#endif
}

static ssize_t channel_config_show(struct device *dev,
                                   struct device_attribute *attr, char *buf)
{
        struct usb_interface *intf;
	struct usb_device *udev;
        struct cp2130_device *chip;
        struct cp2130_channel *chn;
        int i = 0;
        char out[256];
        ssize_t ret;

        intf = to_usb_interface(dev);
        if (!intf)
                return -EFAULT;

        chip = usb_get_intfdata(intf);
        if (!chip)
                return -EFAULT;

	udev = usb_get_dev(interface_to_usbdev(intf));
        if (!udev)
                return -EFAULT;

        ret = sprintf(out, "channel\tcs_mode\tirq_pin\tclock_phase\tpolarity"
                      "\tcs_pin_mode\tclock_freq\tdelay_mask"
                      "\tinter_byte_delay\tpre_delay\tpost_delay"
                      "\tmod_alias\n");
        strcat(buf, out);
        mutex_lock(&chip->chn_config_lock);
        for (i = 0; i < CP2130_NUM_GPIOS; i++) {
                chn = &chip->chn_configs[i];
                ret += sprintf(out, "%d\t%d\t%d\t%d\t\t%d\t\t%d\t\t%s\t%d"
                               "\t\t%d\t\t\t%d\t\t%d\t\t'%s'\n",
                               i, chn->cs_en, chn->irq_pin, chn->clock_phase,
                               chn->polarity, chn->cs_pin_mode,
                               cp2130_spi_speed_to_string(chn->clock_freq),
                               chn->delay_mask, chn->inter_byte_delay,
                               chn->pre_deassert_delay, chn->post_assert_delay,
                               chn->modalias);
                strcat(buf, out);
        }
        mutex_unlock(&chip->chn_config_lock);

        return ret;
}

static int channel_config_store(struct device *dev,
                                struct device_attribute *attr,
                                const char *buf, size_t count)
{
        struct cp2130_channel chn;
        struct usb_interface *intf;
	struct usb_device *udev;
        struct cp2130_device *chip;
        char *lbuf;
        char *del; /* delimiter ',' */
        char *pos; /* current item position in buffer */
        int i, ret, eos, chn_id;

        intf = to_usb_interface(dev);
        if (!intf)
                return -EFAULT;

        chip = usb_get_intfdata(intf);
        if (!chip)
                return -EFAULT;

	udev = usb_get_dev(interface_to_usbdev(intf));
        if (!udev)
                return -EFAULT;

        dev_dbg(&udev->dev, "received '%s' from 'channel_config'", buf);

        if (!count)
                return -EINVAL;

        lbuf = kstrdup(buf, GFP_KERNEL);
        if (!lbuf)
                return -ENOMEM;

        pos = lbuf;
        eos = i = 0;
        do {
                /* search for next separator or end-of-string */
                del = strchr(pos, ',');
                if (!del)
                        del = strchr(pos, '\0');
                if (!del) {
                        ret = -EINVAL;
                        goto out;
                }

                if (*del == '\0')
                        eos = 1;
                *del = '\0';

                dev_dbg(&udev->dev, "parsing: %s(%d)", pos, i);

                /* parse current item */
                switch (i) {
                case 0: /* channel id */
                        ret = kstrtoint(pos, 10, &chn_id);
                        ret |= (chn_id < 0 || chn_id > 10) ? -EINVAL : 0;
                        if (ret)
                                goto out;
                        break;
                case 1: /* chip-select enable */
                        ret = kstrtoint(pos, 10, &chn.cs_en);
                        ret |= (chn.cs_en < 0 || chn.cs_en > 2) ? -EINVAL : 0;
                        if (ret)
                                goto out;
                        break;
                case 2: /* irq pin */
                        ret = kstrtoint(pos, 10, &chn.irq_pin);
                        ret |= (chn.irq_pin > 10) ? -EINVAL : 0;
                        if (ret)
                                goto out;
                        break;
                case 3: /* clock phase */
                        ret = kstrtoint(pos, 10, &chn.clock_phase);
                        if (ret)
                                goto out;
                        chn.clock_phase = !!chn.clock_phase;
                        break;
                case 4: /* polarity */
                        ret = kstrtoint(pos, 10, &chn.polarity);
                        if (ret)
                                goto out;
                        chn.polarity = !!chn.polarity;
                        break;
                case 5: /* chip-select pin mode */
                        ret = kstrtoint(pos, 10, &chn.cs_pin_mode);
                        if (ret)
                                goto out;
                        chn.cs_pin_mode = !!chn.cs_pin_mode;
                        break;
                case 6: /* spi clock frequency */
                        ret = kstrtoint(pos, 10, &chn.clock_freq);
                        ret |= (chn.clock_freq < 0 || chn.clock_freq > 7) ?
                                -EINVAL : 0;
                        if (ret)
                                goto out;
                        break;
                case 7: /* delay mask */
                        ret = kstrtoint(pos, 10, &chn.delay_mask);
                        ret |= (chn.delay_mask < 0 || chn.delay_mask > 15) ?
                                -EINVAL : 0;
                        if (ret)
                                goto out;
                        break;
                case 8: /* inter-byte delay */
                        ret = kstrtoint(pos, 10, &chn.inter_byte_delay);
                        ret |= (chn.inter_byte_delay < 0 ||
                                chn.inter_byte_delay > 0xffff) ?
                                -EINVAL : 0;
                        if (ret)
                                goto out;
                        break;
                case 9: /* pre-deassert delay */
                        ret = kstrtoint(pos, 10, &chn.pre_deassert_delay);
                        ret |= (chn.pre_deassert_delay < 0 ||
                                chn.pre_deassert_delay > 0xffff) ?
                                -EINVAL : 0;
                        if (ret)
                                goto out;
                        break;
                case 10: /* post-assert delay */
                        ret = kstrtoint(pos, 10, &chn.post_assert_delay);
                        ret |= (chn.post_assert_delay < 0 ||
                                chn.post_assert_delay > 0xffff) ?
                                -EINVAL : 0;
                        if (ret)
                                goto out;
                        break;
                case 11: /* modalias */
                        chn.modalias = kstrdup(pos, GFP_KERNEL);
                        if (!chn.modalias) {
                                ret = -EINVAL;
                                goto out;
                        }
                        break;
                default:
                        ret = -EINVAL;
                        goto out;
                }

                /* move the parser position forward */
                pos = ++del;
                i++;
        } while (!eos);

        if (i < 11) {
                ret = -EINVAL;
                goto out;
        }

        chn.updated = 1;
        mutex_lock(&chip->chn_config_lock);
        if (chip->chn_configs[chn_id].updated) {
                ret = -EINVAL;
                goto unlock;
        }
	/* preserve pdata */
	chn.pdata = chip->chn_configs[chn_id].pdata;

        memcpy(&chip->chn_configs[chn_id], &chn, sizeof(chn));
        schedule_work(&chip->update_chn_config);
        ret = count;

unlock:
        mutex_unlock(&chip->chn_config_lock);

out:
        kfree(lbuf);
        return ret;
}
static DEVICE_ATTR_RW(channel_config);


static int channel_pdata_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
        struct usb_interface *intf;
	struct usb_device *udev;
        struct cp2130_device *chip;
        struct cp2130_channel *chn;
	int chn_id;
	int ret;

        intf = to_usb_interface(dev);
        if (!intf)
                return -EFAULT;

        chip = usb_get_intfdata(intf);
        if (!chip)
                return -EFAULT;

	udev = usb_get_dev(interface_to_usbdev(intf));
        if (!udev)
                return -EFAULT;

	/* first byte is channel id,
	   then follows binary platform data */
	if (count < 2)
		return -EINVAL;

	chn_id = buf[0];
        dev_dbg(&udev->dev, "received pdata for channel %u", chn_id);

	if (chn_id < 0 || chn_id > CP2130_NUM_GPIOS)
		return -EINVAL;

	chn = &chip->chn_configs[chn_id];

	/* pdata can be set only once */
	if (chn->pdata) {
		ret = -EINVAL;
		goto out;
	}

        dev_dbg(&udev->dev, "set pdata for channel %u", chn_id);

        mutex_lock(&chip->chn_config_lock);
	chn->pdata = kzalloc(count - 1, GFP_KERNEL);
	if (!chn->pdata) {
		ret = -ENOMEM;
		goto out;
	}

	if (!memcpy(chn->pdata, buf + 1, count - 1)) {
		kfree(chn->pdata);
		chn->pdata = NULL;
		ret = -EIO;
		goto out;
	}

	ret = count;
out:
	mutex_unlock(&chip->chn_config_lock);
        return ret;
}
static DEVICE_ATTR_WO(channel_pdata);

static char* cp2130_pin_mode_to_string(int val)
{
        switch (val) {
        case 0:  return "in        ";
        case 1:  return "open-drain";
        case 2:  return "push-pull ";
        case 3:  return "nCS       ";
        default: return "special   ";
        }
        return "";
}

static ssize_t otp_rom_show(struct device *dev,
                            struct device_attribute *attr, char *buf)
{
        struct usb_interface *intf;
	struct usb_device *udev;
        struct cp2130_device *chip;
        int pin;
        int i = 0;
        char out[256];
        ssize_t ret;

        intf = to_usb_interface(dev);
        if (!intf)
                return -EFAULT;

        chip = usb_get_intfdata(intf);
        if (!chip)
                return -EFAULT;

	udev = usb_get_dev(interface_to_usbdev(intf));
        if (!udev)
                return -EFAULT;

        mutex_lock(&chip->otprom_lock);

        ret = sprintf(out, "OTP lock status (ro):"
                      "\nvid\t\tpid\t\tmax_power\tpower_mode"
                      "\tversion\t\tmanu_2\t\tmanu_1"
                      "\t\tpriority\tproduct_1\tproduct_2\tserial"
                      "\t\tpin_config\n");
        strcat(buf, out);
        for (i = 0; i < 12; i++) {
                if (!!((chip->otprom_config.lock_byte >> i) & 1))
                        ret += sprintf(out, "unlocked\t");
                else
                        ret += sprintf(out, "locked\t\t");
                strcat(buf, out);
        }
        ret += sprintf(out, "\n\nOTP pin configuration (rw):\n");
        strcat(buf, out);

        for (i = 0; i < CP2130_NUM_GPIOS; i++) {
                ret += sprintf(out, "pin %d\t\t", i);
                strcat(buf, out);
        }
        strcat(buf, "\n");
        ret++;
        for (i = 0; i < CP2130_NUM_GPIOS; i++) {
                pin = chip->otprom_config.pin_config[i];
                ret += sprintf(out, "%s\t", cp2130_pin_mode_to_string(pin));
                strcat(buf, out);
        }

        ret += sprintf(out, "\n\nOTP pin configuration extra data(ro):"
		       "\nsuspend_level\tsuspend_mode\twakeup_mask\twakeup_match"
		       "\tdivider\n");
        strcat(buf, out);
        ret += sprintf(out, "%d\t\t%d\t\t%d\t\t%d\t\t%d\n",
                       chip->otprom_config.suspend_level,
                       chip->otprom_config.suspend_mode,
                       chip->otprom_config.wakeup_mask,
                       chip->otprom_config.wakeup_match,
                       chip->otprom_config.divider);
        strcat(buf, out);

        mutex_unlock(&chip->otprom_lock);

        return ret;
}

static int otp_rom_store(struct device *dev,
                         struct device_attribute *attr,
                         const char *buf, size_t count)
{
        int pin_config[CP2130_NUM_GPIOS];
        struct usb_interface *intf;
	struct usb_device *udev;
        struct cp2130_device *chip;
        char *lbuf;
        char *del; /* delimiter ',' */
        char *pos; /* current item position in buffer */
        int i, ret, eos;

        intf = to_usb_interface(dev);
        if (!intf)
                return -EFAULT;

        chip = usb_get_intfdata(intf);
        if (!chip)
                return -EFAULT;

	udev = usb_get_dev(interface_to_usbdev(intf));
        if (!udev)
                return -EFAULT;

        dev_dbg(&udev->dev, "received '%s' from 'otp_rom'", buf);

        if (!count)
                return -EINVAL;

        lbuf = kstrdup(buf, GFP_KERNEL);
        if (!lbuf)
                return -ENOMEM;

        pos = lbuf;
        eos = i = 0;
        do {
                /* search for next separator or end-of-string */
                del = strchr(pos, ',');
                if (!del)
                        del = strchr(pos, '\0');
                if (!del) {
                        ret = -EINVAL;
                        goto out;
                }

                if (*del == '\0')
                        eos = 1;
                *del = '\0';

                dev_dbg(&udev->dev, "parsing: %s(%d)", pos, i);

                if (i == CP2130_NUM_GPIOS) {
                        ret = -EINVAL;
                        goto out;
                }

                /* parse current item */
                if (*pos == 'x') {
                        pin_config[i] = -1;
                } else {
                        ret = kstrtoint(pos, 10, &(pin_config[i]));
                        ret |= (pin_config[i] < 0 ||
                                pin_config[i] > 3) ?
                                -EINVAL : 0;
                        if (ret)
                                goto out;
                }

                /* move the parser position forward */
                pos = ++del;
                i++;
        } while (!eos);

        if (i < 11) {
                ret = -EINVAL;
                goto out;
        }

        mutex_lock(&chip->otprom_lock);

        for (i = 0; i < CP2130_NUM_GPIOS; i++) {
                if (pin_config[i] < 0)
                        continue;
                printk(KERN_INFO "cp2130 read OTP: %d", pin_config[i]);
                chip->otprom_config.pin_config[i] = pin_config[i];
        }
        schedule_work(&chip->update_otprom);
        ret = count;

        mutex_unlock(&chip->otprom_lock);

out:
        kfree(lbuf);
        return ret;
}
static DEVICE_ATTR_RW(otp_rom);

static ssize_t irq_poll_interval_show(struct device *dev,
                                      struct device_attribute *attr, char *buf)
{
        struct usb_interface *intf;
	struct usb_device *udev;
        struct cp2130_device *chip;
        ssize_t ret;

        intf = to_usb_interface(dev);
        if (!intf)
                return -EFAULT;

        chip = usb_get_intfdata(intf);
        if (!chip)
                return -EFAULT;

	udev = usb_get_dev(interface_to_usbdev(intf));
        if (!udev)
                return -EFAULT;

        ret = sprintf(buf, "%d us\n", chip->irq_poll_interval);

        return ret;
}

static int irq_poll_interval_store(struct device *dev,
                                   struct device_attribute *attr,
                                   const char *buf, size_t count)
{
        struct usb_interface *intf;
	struct usb_device *udev;
        struct cp2130_device *chip;
        int ret, interval;

        intf = to_usb_interface(dev);
        if (!intf)
                return -EFAULT;

        chip = usb_get_intfdata(intf);
        if (!chip)
                return -EFAULT;

	udev = usb_get_dev(interface_to_usbdev(intf));
        if (!udev)
                return -EFAULT;

        ret = kstrtoint(buf, 10, &interval);

        /* we only allow numbers and nothing below 10us */
        if (ret || interval < 10)
                return -EINVAL;

        chip->irq_poll_interval = interval;

        return count;
}
static DEVICE_ATTR_RW(irq_poll_interval);

static int cp2130_spi_transfer_one_message(struct spi_master *master,
                                           struct spi_message *mesg)
{
	struct spi_transfer *xfer;
        struct cp2130_device *dev =
		(struct cp2130_device*) spi_master_get_devdata(master);
        char *urb;
	size_t urb_len;
        char ctrl_urb[2] = { 0 };
        int len, ret = 0, chn_id;
        struct cp2130_channel *chn;
	unsigned int recv_pipe, xmit_pipe;
	unsigned int xmit_ctrl_pipe;

	dev_dbg(&master->dev, "spi transfer one message");

	xmit_pipe = usb_sndbulkpipe(dev->udev, 0x01);
	recv_pipe = usb_rcvbulkpipe(dev->udev, 0x82);

	dev_dbg(&master->dev, "recv/xmit pipes: %u / %u",
		recv_pipe, xmit_pipe);

        /* search for spi setup of this device */
        for (chn_id = 0; chn_id < CP2130_NUM_GPIOS; chn_id++) {
                chn = &dev->chn_configs[chn_id];
                if (chn->chip == mesg->spi)
                        break;
        }

        mutex_lock(&dev->usb_bus_lock);

        if (chn_id == CP2130_NUM_GPIOS)
                goto out;

	xmit_ctrl_pipe = usb_sndctrlpipe(dev->udev, 0);
        if (chn_id != dev->current_channel) {
                dev_dbg(&dev->udev->dev, "load setup %d for channel %d",
                        chn->cs_en, chn_id);
                ctrl_urb[0] = chn_id;
                ctrl_urb[1] = chn->cs_en;
                ret = usb_control_msg(
                        dev->udev, xmit_ctrl_pipe,
                        CP2130_BREQ_SET_GPIO_CS,
                        USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_DIR_OUT,
                        0, 0,
                        ctrl_urb, 2, 200);
                if (ret < 2)
                        goto out;

                dev->current_channel = chn_id;
        }

	urb_len = CP2130_BULK_OFFSET_DATA;
	list_for_each_entry(xfer, &mesg->transfers, transfer_list) {
                if (xfer->tx_buf &&
                    ((xfer->len + CP2130_BULK_OFFSET_DATA) > urb_len))
                        urb_len = CP2130_BULK_OFFSET_DATA + xfer->len;
	}
	urb = kmalloc(urb_len, GFP_KERNEL);
	if (!urb) {
		ret = -ENOMEM;
		goto out;
	}

	/* iterate through all transfers */
	list_for_each_entry(xfer, &mesg->transfers, transfer_list) {
		dev_dbg(&master->dev, "spi transfer stats: %p, %p, %d",
			xfer->tx_buf, xfer->rx_buf, xfer->len);

                /* empty transfer */
                if (!xfer->tx_buf && !xfer->rx_buf) {
                        udelay(xfer->delay_usecs);
                        continue;
                }

		memset(urb, 0, urb_len);

		/* init length field */
		*((u32*) (urb + CP2130_BULK_OFFSET_LENGTH)) =
			__cpu_to_le32(xfer->len);

		/* copy SPI tx data */
		if (xfer->tx_buf)
			memcpy(urb + CP2130_BULK_OFFSET_DATA,
			       xfer->tx_buf, xfer->len);

		/* prepare URB and submit sync
		   CP2130 / AN792 p.7: 'any previous data transfer command
		   must complete before another data transfer command
		   is issued', so there is no advantage from using the
		   async USB API */

                if (xfer->tx_buf && xfer->rx_buf) {
			urb[CP2130_BULK_OFFSET_CMD] = CP2130_CMD_WRITEREAD;
			/* usb write */
			ret = usb_bulk_msg(dev->udev, xmit_pipe, urb,
					   CP2130_BULK_OFFSET_DATA + xfer->len,
					   &len, 200);
			dev_dbg(&master->dev, "usb write %d", ret);
			if (ret)
				break;
			/* usb read */
			ret = usb_bulk_msg(dev->udev, recv_pipe,
					   xfer->rx_buf, xfer->len,
					   &len, 200);
			dev_dbg(&master->dev, "usb read %d", ret);
			if (ret)
				break;
		} else if (!xfer->rx_buf) {
			/* prepare URB and submit sync */
			urb[CP2130_BULK_OFFSET_CMD] = CP2130_CMD_WRITE;
			/* usb write */
			ret = usb_bulk_msg(dev->udev, xmit_pipe, urb,
					   CP2130_BULK_OFFSET_DATA + xfer->len,
					   &len, 200);
			if (ret)
				break;
		} else if (!xfer->tx_buf) {
			/* prepare URB and submit sync */
			urb[CP2130_BULK_OFFSET_CMD] = CP2130_CMD_READ;
			/* usb write */
			ret = usb_bulk_msg(dev->udev, xmit_pipe, urb,
					   CP2130_BULK_OFFSET_DATA,
					   &len, 200);
			if (ret)
				break;
			/* usb read */
			ret = usb_bulk_msg(dev->udev, recv_pipe, xfer->rx_buf,
					   xfer->len, &len, 200);
			if (ret)
				break;
		}

		udelay(xfer->delay_usecs);
		mesg->actual_length += xfer->len;
        }

	kfree(urb);

out:
        mutex_unlock(&dev->usb_bus_lock);
	mesg->status = ret;
        if (ret)
                dev_err(&master->dev, "USB transfer failed with %d", ret);
        spi_finalize_current_message(master); /* signal done to queue */
        return ret;
}

static int cp2130_irq_from_pin(struct cp2130_device *dev, int pin)
{
        return dev->irq_chip.virq[pin];
}

static void cp2130_update_channel_config(struct work_struct *work)
{
        int i;
	char urb[8];
	int ret;
	unsigned int xmit_pipe;
        struct cp2130_device *dev = container_of(work,
                                                 struct cp2130_device,
                                                 update_chn_config);
        struct cp2130_channel *chn;

	dev_dbg(&dev->udev->dev, "control pipes: %u, %u",
		usb_sndctrlpipe(dev->udev, 0),
		usb_rcvctrlpipe(dev->udev, 0));

	xmit_pipe = usb_sndctrlpipe(dev->udev, 0);

        mutex_lock(&dev->chn_config_lock);
        for (i = 0; i < CP2130_NUM_GPIOS; i++) {
                chn = &dev->chn_configs[i];
                if (!chn->updated)
                        continue;

                if (chn->updated > 1)
                        continue;

                chn->updated++;

                dev_dbg(&dev->udev->dev, "update config of channel %d", i);
                urb[0] = i;

                mutex_lock(&dev->usb_bus_lock);

                dev_dbg(&dev->udev->dev, "set spi word");
		urb[1] = (chn->clock_freq << 0) |
                        (chn->cs_pin_mode << 3) |
                        (chn->polarity    << 4) |
                        (chn->clock_phase << 5);

                ret = usb_control_msg(
                        dev->udev, xmit_pipe,
                        CP2130_BREQ_SET_SPI_WORD,
                        USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_DIR_OUT,
                        0, 0,
                        urb, 2, 200);
                if (ret < 2)
                        goto error_unlock;

                dev_dbg(&dev->udev->dev, "set spi delay");
                urb[1] = chn->delay_mask;

                urb[2] = (chn->inter_byte_delay & 0xff00) >> 8;
                urb[3] = (chn->inter_byte_delay & 0x00ff) >> 0;

                urb[4] = (chn->post_assert_delay & 0xff00) >> 8;
                urb[5] = (chn->post_assert_delay & 0x00ff) >> 0;

                urb[6] = (chn->pre_deassert_delay & 0xff00) >> 8;
                urb[7] = (chn->pre_deassert_delay & 0x00ff) >> 0;
                ret = usb_control_msg(
                        dev->udev, xmit_pipe,
                        CP2130_BREQ_SET_SPI_DELAY,
                        USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_DIR_OUT,
                        0, 0,
                        urb, 8, 200);
                if (ret < 8)
                        goto error_unlock;

                /* configure irq pin as input if required */
                if (chn->irq_pin >= 0) {
                        urb[0] = chn->irq_pin;
                        urb[1] = 0; /* input */
                        urb[2] = 0; /* value, ignored for input */
                        ret = usb_control_msg(
                                dev->udev, xmit_pipe,
                                CP2130_BREQ_SET_GPIO_MODE,
                                USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_DIR_OUT,
                                0, 0,
                                urb, 3, 200);
                        if (ret < 3)
                                goto error_unlock;
                }

                mutex_unlock(&dev->usb_bus_lock);

                dev_dbg(&dev->udev->dev, "try register %s", chn->modalias);

                chn->chip = spi_alloc_device(dev->spi_master);
                if (!chn->chip)
                        goto error;

                chn->chip->max_speed_hz = dev->spi_master->max_speed_hz;
                chn->chip->chip_select = i;
                chn->chip->mode	= (chn->polarity << 1) | chn->clock_phase;
                chn->chip->bits_per_word = 8; /* we can only do this */
                chn->chip->irq = cp2130_irq_from_pin(dev, chn->irq_pin);

		chn->chip->dev.platform_data = chn->pdata;

                dev_dbg(&dev->udev->dev, "initialized %s chip", chn->modalias);
                strncpy(chn->chip->modalias, chn->modalias,
                        sizeof(chn->chip->modalias));

                ret = spi_add_device(chn->chip);
                if (ret)
                        goto error;

                dev_dbg(&dev->udev->dev, "%s probe complete", chn->modalias);
        }
        mutex_unlock(&dev->chn_config_lock);

        schedule_work(&dev->read_chn_config);
        return;

error_unlock:
        mutex_unlock(&dev->usb_bus_lock);

error:
        mutex_unlock(&dev->chn_config_lock);
        dev_err(&dev->udev->dev, "failed to configure channel %d", i);
}

static void cp2130_read_channel_config(struct work_struct *work)
{
        int i;
	char urb[32];
	int ret;
	unsigned int recv_pipe;
        struct cp2130_device *dev = container_of(work,
                                                 struct cp2130_device,
                                                 read_chn_config);
        struct cp2130_channel *chn;

	dev_dbg(&dev->udev->dev, "control pipes: %u, %u",
		usb_sndctrlpipe(dev->udev, 0),
		usb_rcvctrlpipe(dev->udev, 0));

	recv_pipe = usb_rcvctrlpipe(dev->udev, 0);

        mutex_lock(&dev->chn_config_lock);
        mutex_lock(&dev->usb_bus_lock);
        dev_dbg(&dev->udev->dev, "read channel configs");

        dev_dbg(&dev->udev->dev, "get spi word");
        ret = usb_control_msg(
                dev->udev, recv_pipe,
                CP2130_BREQ_GET_SPI_WORD,
                USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_DIR_IN,
                0, 0,
                urb, 11, 200);

        for (i = 0; i < CP2130_NUM_GPIOS; i++) {
                chn = &dev->chn_configs[i];

                chn->clock_freq = urb[i] & 7;
                chn->cs_pin_mode = !!(urb[i] & 8);
                chn->polarity = !!(urb[i] & 16);
                chn->clock_phase = !!(urb[i] & 32);
        }

        dev_dbg(&dev->udev->dev, "get delays");
        for (i = 0; i < CP2130_NUM_GPIOS; i++) {
                ret = usb_control_msg(
                        dev->udev, recv_pipe,
                        CP2130_BREQ_GET_SPI_DELAY,
                        USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_DIR_IN,
                        0, i,
                        urb, 8, 200);

                chn = &dev->chn_configs[i];

                chn->delay_mask = urb[1];
                chn->inter_byte_delay = urb[2] << 8;
                chn->inter_byte_delay |= urb[2] & 0xff;
                chn->post_assert_delay = urb[3] << 8;
                chn->post_assert_delay |= urb[4] & 0xff;
                chn->pre_deassert_delay = urb[5] << 8;
                chn->pre_deassert_delay |= urb[6] & 0xff;
        }

        mutex_unlock(&dev->usb_bus_lock);
        mutex_unlock(&dev->chn_config_lock);
}

static void cp2130_update_otprom(struct work_struct *work)
{
        int i;
        struct cp2130_device *dev = container_of(work,
                                                 struct cp2130_device,
                                                 update_otprom);
	char urb[0x14] = { 0 };
	int ret;
	unsigned int xmit_pipe;

	xmit_pipe = usb_sndctrlpipe(dev->udev, 0);

        mutex_lock(&dev->otprom_lock);

        for (i = 0; i < CP2130_NUM_GPIOS; i++)
                urb[i] = dev->otprom_config.pin_config[i];

        mutex_lock(&dev->usb_bus_lock);

        ret = usb_control_msg(
                dev->udev, xmit_pipe,
                CP2130_BREQ_SET_PIN_CONFIG,
                USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_DIR_OUT,
                CP2130_BREQ_MEMORY_KEY, 0,
                urb, 0x14, 200);

        if (ret)
                dev_err(&dev->udev->dev, "error writing OTP ROM pin config");

        mutex_unlock(&dev->usb_bus_lock);
        mutex_unlock(&dev->otprom_lock);

        schedule_work(&dev->read_otprom);
}

static void cp2130_read_otprom(struct work_struct *work)
{
        int i;
	char urb[32];
	int ret;
	unsigned int recv_pipe;
        struct cp2130_device *dev = container_of(work,
                                                 struct cp2130_device,
                                                 read_otprom);

	dev_dbg(&dev->udev->dev, "control pipes: %u, %u",
		usb_sndctrlpipe(dev->udev, 0),
		usb_rcvctrlpipe(dev->udev, 0));

	recv_pipe = usb_rcvctrlpipe(dev->udev, 0);

        mutex_lock(&dev->otprom_lock);
        mutex_lock(&dev->usb_bus_lock);
        dev_dbg(&dev->udev->dev, "read OTP ROM");

        dev_dbg(&dev->udev->dev, "get lock byte");
        ret = usb_control_msg(
                dev->udev, recv_pipe,
                CP2130_BREQ_GET_LOCK_BYTE,
                USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_DIR_IN,
                0, 0,
                urb, 2, 200);
        dev_info(&dev->udev->dev, "lock byte %02X %02X",
                 urb[0] & 0xff, urb[1] & 0x0f);
        dev->otprom_config.lock_byte = urb[0] & 0xff;
        dev->otprom_config.lock_byte |= (urb[1] & 0x0f) << 8;

        dev_dbg(&dev->udev->dev, "get pin config");
        ret = usb_control_msg(
                dev->udev, recv_pipe,
                CP2130_BREQ_GET_PIN_CONFIG,
                USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_DIR_IN,
                0, 0,
                urb, 0x14, 200);

        for (i = 0; i < CP2130_NUM_GPIOS; ++i) {
                dev_info(&dev->udev->dev, "pin %d: %02X",
                         i, urb[i]);
                dev->otprom_config.pin_config[i] = urb[i];
        }
        dev->otprom_config.suspend_level = (urb[i] << 8) | urb[i + 1];
        i += 2;
        dev->otprom_config.suspend_mode = (urb[i] << 8) | urb[i + 1];
        i += 2;
        dev->otprom_config.wakeup_mask = (urb[i] << 8) | urb[i + 1];
        i += 2;
        dev->otprom_config.wakeup_match = (urb[i] << 8) | urb[i + 1];
        i += 2;
        dev->otprom_config.divider = urb[i];

        mutex_unlock(&dev->usb_bus_lock);
        mutex_unlock(&dev->otprom_lock);
}

static void cp2130_read_gpios(struct work_struct *work)
{
	unsigned int recv_pipe;
	char urb[2];
        struct cp2130_device *dev = container_of(work,
                                                 struct cp2130_device,
                                                 irq_work);
        int i, set;
        unsigned long flags;

loop:
	recv_pipe = usb_rcvctrlpipe(dev->udev, 0);

        dev_dbg(&dev->udev->dev, "start read gpios");

        mutex_lock(&dev->usb_bus_lock);
        i = usb_control_msg(
                dev->udev, recv_pipe,
                CP2130_BREQ_GET_GPIO_VALUES,
                USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_DIR_IN,
                0, 0,
                urb, 2, 200);
        mutex_unlock(&dev->usb_bus_lock);

	dev->gpio_states[0] = urb[0];
	dev->gpio_states[1] = urb[1];

        if (i < 2) {
		dev_err(&dev->udev->dev, "failed to read gpios");
                goto next;
	}

        dev_dbg(&dev->udev->dev, "read gpios 1: %02X",
                urb[1] & 0xff & ~(1 | 2 | 4));
        dev_dbg(&dev->udev->dev, "read gpios 2: %02X",
                urb[0] & 0xff & ~(2 | 128));

        for (i = 0; i < CP2130_NUM_GPIOS; ++i) {
                if (!(dev->irq_chip.irq_mask & (1 << i)))
                        continue;

		switch (i) {
		case 0:
		case 1:
		case 2:
		case 3:
		case 4:
                        set = urb[1] & (8 << i);
			break;
		case 5:
                        set = urb[0] & (1 << 0);
			break;
		case 6:
		case 7:
		case 8:
		case 9:
		case 10:
                        set = urb[0] & (4 << (i - 6));
			break;
		}

                if (!set) {
                        dev_dbg(&dev->udev->dev, "issue irq on %d", i);
                        local_irq_save(flags);
                        generic_handle_irq(dev->irq_chip.virq[i]);
                        local_irq_restore(flags);
                }
        }

next:
	if (dev->irq_poll_interval < 0)
		return;

	usleep_range(dev->irq_poll_interval, dev->irq_poll_interval + 20);
	goto loop;
}

static void cp2130_gpio_irq_mask(struct irq_data *data)
{
	struct cp2130_gpio_irq *irq_dev = irq_data_get_irq_chip_data(data);
	struct cp2130_device *dev
		= container_of(irq_dev, struct cp2130_device, irq_chip);

        dev_dbg(&dev->udev->dev, "irq mask %lu", data->hwirq);
        irq_dev->irq_mask &= ~(1 << data->hwirq);
}

static void cp2130_gpio_irq_unmask(struct irq_data *data)
{
	struct cp2130_gpio_irq *irq_dev = irq_data_get_irq_chip_data(data);
	struct cp2130_device *dev
		= container_of(irq_dev, struct cp2130_device, irq_chip);

        dev_dbg(&dev->udev->dev, "irq unmask %lu", data->hwirq);
        irq_dev->irq_mask |= 1 << data->hwirq;
}

static int cp2130_gpio_irq_set_type(struct irq_data *data, unsigned int type)
{
	return 0;
}

static void cp2130_gpio_irq_bus_lock(struct irq_data *data)
{
	struct cp2130_gpio_irq *irq_dev = irq_data_get_irq_chip_data(data);
	mutex_lock(&irq_dev->irq_lock);
}

static void cp2130_gpio_irq_bus_unlock(struct irq_data *data)
{
	struct cp2130_gpio_irq *irq_dev = irq_data_get_irq_chip_data(data);
	mutex_unlock(&irq_dev->irq_lock);
}

static struct irq_chip cp2130_gpio_irq_chip = {
	.name		     = "gpio-cp2130",
	.irq_mask	     = cp2130_gpio_irq_mask,
	.irq_unmask	     = cp2130_gpio_irq_unmask,
	.irq_set_type	     = cp2130_gpio_irq_set_type,
	.irq_bus_lock	     = cp2130_gpio_irq_bus_lock,
	.irq_bus_sync_unlock = cp2130_gpio_irq_bus_unlock,
};

static int cp2130_gpio_irq_map(struct irq_domain *domain, unsigned int irq,
			       irq_hw_number_t hwirq)
{
	irq_set_chip_data(irq, domain->host_data);
	irq_set_chip(irq, &cp2130_gpio_irq_chip);
	irq_set_chip_and_handler(irq, &cp2130_gpio_irq_chip,
				 handle_simple_irq);
        irq_set_noprobe(irq);
	return 0;
}

static const struct irq_domain_ops cp2130_gpio_irq_domain_ops = {
	.map	= cp2130_gpio_irq_map,
};

static void cp2130_gpio_irq_remove(struct cp2130_device *dev);

static int cp2130_gpio_irq_probe(struct cp2130_device *dev)
{
	struct cp2130_gpio_irq *irq_dev = &dev->irq_chip;
	int i;

	mutex_init(&irq_dev->irq_lock);

	irq_dev->irq_domain = irq_domain_add_linear(
		dev->udev->dev.of_node, CP2130_NUM_GPIOS,
		&cp2130_gpio_irq_domain_ops, irq_dev);

	if (!irq_dev->irq_domain) {
		dev_err(&dev->udev->dev, "failed to register IRQ domain");
		return -ENOMEM;
	}

	irq_dev->irq_mask = 0;
	for (i = 0; i < CP2130_NUM_GPIOS; ++i) {
		irq_dev->virq[i] =
			irq_create_mapping(irq_dev->irq_domain, i);
		dev_dbg(&dev->udev->dev, "created virtual irq %d",
                        irq_dev->virq[i]);
	}

        INIT_WORK(&dev->irq_work, cp2130_read_gpios);
        schedule_work(&dev->irq_work);

	return 0;
}

static void cp2130_gpio_irq_remove(struct cp2130_device *dev)
{
	int i = 0;

	if (!dev->irq_chip.irq_domain)
		return;

	for (i = 0; i < CP2130_NUM_GPIOS; ++i) {
		irq_dispose_mapping(dev->irq_chip.virq[i]);
	}

	irq_domain_remove(dev->irq_chip.irq_domain);
}

static int cp2130_gpio_direction_input(struct gpio_chip *gc, unsigned off)
{
	struct cp2130_device *dev = gpiochip_get_data(gc);
	int ret;
	char urb[8];
	unsigned int xmit_pipe;

	xmit_pipe = usb_sndctrlpipe(dev->udev, 0);

	mutex_lock(&dev->usb_bus_lock);

	urb[0] = off;
	urb[1] = 0; /* input */
	urb[2] = 0; /* value, ignored for input */
	ret = usb_control_msg(
		dev->udev, xmit_pipe,
		CP2130_BREQ_SET_GPIO_MODE,
		USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_DIR_OUT,
		0, 0,
		urb, 3, 200);

	mutex_unlock(&dev->usb_bus_lock);

	return (ret == 3) ? 0 : -EIO;
}

static int cp2130_gpio_direction_output(struct gpio_chip *gc, unsigned off,
					int val)
{
	struct cp2130_device *dev = gpiochip_get_data(gc);
	int ret;
	char urb[8];
	unsigned int xmit_pipe;

	xmit_pipe = usb_sndctrlpipe(dev->udev, 0);

	mutex_lock(&dev->usb_bus_lock);

	urb[0] = off;
	urb[1] = 2; /* push-pull output */
	urb[2] = val; /* state */
	ret = usb_control_msg(
		dev->udev, xmit_pipe,
		CP2130_BREQ_SET_GPIO_MODE,
		USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_DIR_OUT,
		0, 0,
		urb, 3, 200);

	mutex_unlock(&dev->usb_bus_lock);

	return (ret == 3) ? 0 : -EIO;
}

static int cp2130_gpio_get_value(struct gpio_chip *gc, unsigned off)
{
	struct cp2130_device *dev = gpiochip_get_data(gc);
	int set;

	switch (off) {
	case 0:
	case 1:
	case 2:
	case 3:
	case 4:
		set = !!(dev->gpio_states[1] & (8 << off));
		break;
	case 5:
		set = !!(dev->gpio_states[0] & (1 << 0));
		break;
	case 6:
	case 7:
	case 8:
	case 9:
	case 10:
		set = !!(dev->gpio_states[0] & (4 << (off - 6)));
		break;
	default:
		set = -EINVAL;
		break;
	}
	return set;
}

static void cp2130_gpio_set_value(struct gpio_chip *gc, unsigned off, int val)
{
	cp2130_gpio_direction_output(gc, off, val);
}

static const char* cp2130_gpio_names[] = { "........-_cs0",
					   "........-_cs1",
					   "........-_cs2",
					   "........-_rtr",
					   "........-event_counter",
					   "........-clk_out",
					   "........-gpi",
					   "........-gpo",
					   "........-activity",
					   "........-suspend",
					   "........-_suspend",
};

int cp2130_probe(struct usb_interface *intf, const struct usb_device_id *id)
{
	struct usb_device *udev = usb_get_dev(interface_to_usbdev(intf));
	struct cp2130_device *dev;
        struct spi_master *spi_master;
	struct gpio_chip *gc;
	struct usb_host_interface *iface_desc = intf->cur_altsetting;
	struct usb_endpoint_descriptor *endpoint;
        int i, ret;

	ret = -ENOMEM;

        printk(KERN_DEBUG "cp2130 probe\n");

        if (!udev)
                return -ENODEV;

        dev = kzalloc(sizeof(struct cp2130_device), GFP_KERNEL);
        if (!dev)
                goto dev_err_out;

        dev->udev = udev;
        dev->intf = intf;

	mutex_init(&dev->usb_bus_lock);
	mutex_init(&dev->chn_config_lock);
	mutex_init(&dev->otprom_lock);

        dev->current_channel = -1;

        usb_set_intfdata(intf, dev);

	for (i = 0; i < iface_desc->desc.bNumEndpoints; ++i) {
		endpoint = &iface_desc->endpoint[i].desc;
		dev_dbg(&udev->dev, "ep addr: 0x%02x",
			endpoint->bEndpointAddress);
		if (usb_endpoint_is_bulk_in(endpoint))
			dev_dbg(&udev->dev, "bulk in ep addr: 0x%02x",
				endpoint->bEndpointAddress);
		if (usb_endpoint_is_bulk_out(endpoint))
			dev_dbg(&udev->dev, "bulk out ep addr: 0x%02x",
				endpoint->bEndpointAddress);
	}

        spi_master = spi_alloc_master(&udev->dev, sizeof(void*));
        if (!spi_master)
                goto err_out;

        spi_master_set_devdata(spi_master, (void*) dev);

        spi_master->min_speed_hz = 93 * 1000 + 800; /* 93.8kHz */
        spi_master->max_speed_hz = 12 * 1000 * 1000; /* 12 MHz */

        spi_master->bus_num = -1; /* dynamically assigned */
        spi_master->num_chipselect = CP2130_NUM_GPIOS;
        spi_master->mode_bits =
                SPI_MODE_0 | SPI_MODE_1 | SPI_MODE_2 | SPI_MODE_3;

        spi_master->flags = 0;
        spi_master->setup = cp2130_spi_setup;
        spi_master->cleanup = cp2130_spi_cleanup;
        spi_master->transfer_one_message = cp2130_spi_transfer_one_message;

        ret = spi_register_master(spi_master);

        if (ret) {
                dev_err(&udev->dev, "failed to register SPI master");
                spi_master_put(spi_master);
                dev->spi_master = NULL;
                goto err_out;
        }

        dev->spi_master = spi_master;
        dev_dbg(&udev->dev, "registered SPI master");

	/* now enable the gpio chip */
	gc = &dev->gpio_chip;
	gc->direction_input = cp2130_gpio_direction_input;
	gc->direction_output = cp2130_gpio_direction_output;
	gc->get = cp2130_gpio_get_value;
	gc->set = cp2130_gpio_set_value;
	gc->can_sleep = true;

	gc->base = -1; /* auto */
	gc->ngpio = CP2130_NUM_GPIOS;
	for (i = 0; i < CP2130_NUM_GPIOS; i++) {
		dev->gpio_names[i] = kstrdup(cp2130_gpio_names[i], GFP_KERNEL);
		memcpy(dev->gpio_names[i],
		       dev_name(&spi_master->dev), 3 + 5 /* spixxxxx */);
	}
	gc->names = (const char**) dev->gpio_names;
	gc->label = dev_name(&spi_master->dev);
	gc->owner = THIS_MODULE;

	ret = gpiochip_add_data(&dev->gpio_chip, dev);
	if (ret)
                dev_err(&udev->dev, "failed to register gpio chip");

	/* start irq polling */
        dev->irq_poll_interval = CP2130_IRQ_POLL_INTERVAL;
        cp2130_gpio_irq_probe(dev);
        dev_dbg(&udev->dev, "registered irq chip");

        /* create sysfs files */
        ret = device_create_file(&intf->dev,
                                 &dev_attr_channel_config);
        if (ret)
                dev_err(&udev->dev,
                        "failed to create channel_config sysfs entry");

        ret = device_create_file(&intf->dev,
                                 &dev_attr_channel_pdata);
        if (ret)
                dev_err(&udev->dev,
                        "failed to create channel_pdata sysfs entry");

        ret = device_create_file(&intf->dev,
                                 &dev_attr_otp_rom);
        if (ret)
                dev_err(&udev->dev,
                        "failed to create otp_rom sysfs entry");

        ret = device_create_file(&intf->dev,
                                 &dev_attr_irq_poll_interval);
        if (ret)
                dev_err(&udev->dev,
                        "failed to create irq_poll_interval sysfs entry");

        INIT_WORK(&dev->update_chn_config, cp2130_update_channel_config);
        INIT_WORK(&dev->read_chn_config, cp2130_read_channel_config);
        INIT_WORK(&dev->update_otprom, cp2130_update_otprom);
        INIT_WORK(&dev->read_otprom, cp2130_read_otprom);
        schedule_work(&dev->read_chn_config);
        schedule_work(&dev->read_otprom);

        return 0;

err_out:
        kfree(dev);
        if (spi_master)
                kfree(spi_master);

dev_err_out:
        return ret;
}

void cp2130_disconnect(struct usb_interface *intf)
{
	struct cp2130_device *dev = usb_get_intfdata(intf);
	int i = dev->irq_poll_interval;

        dev->irq_poll_interval = -1;
	usleep_range(i, i + 100); /* wait for worker to complete */

        cp2130_gpio_irq_remove(dev);
	gpiochip_remove_(&dev->gpio_chip);
	for (i = 0; i < CP2130_NUM_GPIOS; i++)
		kfree(dev->gpio_names[i]);

        /* remove sysfs files */
        device_remove_file(&intf->dev,
                           &dev_attr_channel_config);
        device_remove_file(&intf->dev,
                           &dev_attr_channel_pdata);
        device_remove_file(&intf->dev,
                           &dev_attr_otp_rom);
        device_remove_file(&intf->dev,
                           &dev_attr_irq_poll_interval);

        spi_unregister_master(dev->spi_master);
}

module_init(cp2130_init);
module_exit(cp2130_exit);
MODULE_AUTHOR("Jochen Henneberg <jh@henneberg-systemdesign.com>");
MODULE_DESCRIPTION("Silicon Labs CP2130 single chip USB-to-SPI brigde");
MODULE_LICENSE("GPL");

MODULE_ALIAS("cp2130");
