#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/usb.h>
#include <linux/fb.h>
#include <linux/time.h>
#include "it8951_types.h"

// TODO RGB888 to gray2/gray4 for other displays
// TODO add partial updates
// TODO clear display before disconnecting 

/* Driver name */
#define IT8951_DRV_ID "eink_usb_disp"

/* USB related defines */
#define IT8951_DRV_USB_VID 0x048D /* ITE Tech. Inc. */
#define IT8951_DRV_USB_PID 0x8951 /* IT8951 */
#define IT8951_DRV_LUN 0
#define IT8951_DRV_USB_TIMEOUT_MS 2500

/* IT8951 command related defines */
#define IT8951_DRV_INQUIRY_CMD 0x12
#define IT8951_DRV_CUSTOM_CMD 0xFE
#define IT8951_DRV_GET_SYS_OP 0x80
#define IT8951_DRV_READ_MEM_OP 0x81
#define IT8951_DRV_WRITE_MEM_OP 0x82
#define IT8951_DRV_DPY_AREA_OP 0x94
#define IT8951_DRV_LD_IMG_AREA_OP 0xA2
#define IT8951_DRV_PMIC_CTRL_OP 0xA3
#define IT8951_DRV_FAST_WRITE_MEM_OP 0xA5
#define IT8951_DRV_AUTO_RESET_OP 0xA7
#define IT8951_DRV_CBW_SIGNATURE 0x43425355 // 'USBD'
#define IT8951_DRV_GET_SYS_SIGNATURE 0x31353938 // '8951'
#define IT8951_DRV_GET_SYS_VERSION 0x00020001

/* Maximum memory transfer size for IT8951 */
#define IT8951_DRV_MAX_BLOCK_SIZE (60 * 1024U)

/* General defines */
#define IT8951_DRV_BITS_PER_BYTE 8
#define IT8951_DRV_DEFAULT_VCOM 2480
#define IT8951_DRV_MAX_FAST_REFRESHES 100

/* Macros */
#define ROUND_DOWN_TO_MULTIPLE(x, mul) (((x) / (mul)) * (mul))

/* Parameters */
static int vcom_value_mv = IT8951_DRV_DEFAULT_VCOM;
module_param(vcom_value_mv, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(vcom_value_mv, "VCOM voltage value in mV, without sign");

static int max_fast_refreshes = IT8951_DRV_MAX_FAST_REFRESHES;
module_param(max_fast_refreshes, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(max_fast_refreshes, "Number of fast refreshes before a deep refresh occurs");


static const struct usb_device_id it8951_usb_ids[] = {
        {USB_DEVICE(IT8951_DRV_USB_VID, IT8951_DRV_USB_PID)},
        { }
};

MODULE_DEVICE_TABLE(usb, it8951_usb_ids);

static bool device_connected = false;

static struct fb_fix_screeninfo it8951_fix = {
        .id = IT8951_DRV_ID,
        .type = FB_TYPE_PACKED_PIXELS,
        .visual = FB_VISUAL_MONO10,
        .accel = FB_ACCEL_NONE
};

static struct fb_var_screeninfo it8951_var = {
        .bits_per_pixel = 8, // TODO this can be reduced to 1 probably
        .red = {0, 1, 0},
        .green = {0, 1, 0},
        .blue = {0, 1, 0},
        .activate = FB_ACTIVATE_NOW,
        .vmode = FB_VMODE_NONINTERLACED
};

static int it8951_drv_usb_bulk_send(const struct it8951_device *dev, void *data, size_t size)
{
    int bytes_received;
    unsigned pipe;

    /* Get send pipe */
    pipe = usb_sndbulkpipe(dev->udev, dev->bulk_out_addr);

    /* Send data */
    return usb_bulk_msg(dev->udev, pipe, data, size, &bytes_received, IT8951_DRV_USB_TIMEOUT_MS);
}

static int it8951_drv_usb_bulk_recv(const struct it8951_device *dev, void *data, size_t size)
{
    int bytes_sent;
    unsigned pipe;

    /* Get receive pipe */
    pipe = usb_rcvbulkpipe(dev->udev, dev->bulk_in_addr);

    /* Receive data */
    return usb_bulk_msg(dev->udev, pipe, data, size, &bytes_sent, IT8951_DRV_USB_TIMEOUT_MS);
}

static struct cmd_block_wrapper *it8951_create_cbw(enum it8951_cmd_dir dir, size_t data_xfer_length)
{
    struct cmd_block_wrapper *cbw;

    /* Create command block wrapper */
    cbw = kzalloc(sizeof(*cbw), GFP_KERNEL);
    if (cbw == NULL) {
        return NULL;
    }

    /* Fill basic data */
    cbw->signature = IT8951_DRV_CBW_SIGNATURE;
    cbw->data_transfer_length = data_xfer_length;
    cbw->flags = dir;
    cbw->lun = IT8951_DRV_LUN;
    cbw->cmd_length = sizeof(cbw->cmd_data);

    return cbw;
}

static int it8951_read_csw(const struct it8951_device *dev)
{
    struct cmd_status_wrapper *csw;
    int ret;

    csw = kzalloc(sizeof(*csw), GFP_KERNEL);
    if (csw == NULL) {
        ret = -ENOMEM;
        goto out_error;
    }

    ret = it8951_drv_usb_bulk_recv(dev, csw, sizeof(*csw));
    if (ret != 0) {
        goto out_error;
    }
    if (csw->status != 0) {
        ret = -EIO;
    }

out_error:
    if (csw != NULL) {
        kfree(csw);
    }

    return ret;
}

static int it8951_get_info(const struct it8951_device *dev, struct it8951_dev_info *info)
{
    struct cmd_block_wrapper *cbw;
    int ret;

    /* Create command block wrapper */
    cbw = it8951_create_cbw(DIR_BULK_IN, sizeof(*info));
    if (cbw == NULL) {
        ret = -ENOMEM;
        goto out_error;
    }
    cbw->cmd_data[0] = IT8951_DRV_CUSTOM_CMD;
    *(uint32_t *)&cbw->cmd_data[2] = IT8951_DRV_GET_SYS_SIGNATURE;
    cbw->cmd_data[6] = IT8951_DRV_GET_SYS_OP;
    *(uint32_t *)&cbw->cmd_data[8] = IT8951_DRV_GET_SYS_VERSION;

    /* Send command block wrapper */
    ret = it8951_drv_usb_bulk_send(dev, cbw, sizeof(*cbw));
    if (ret != 0) {
        goto out_error;
    }

    /* Receive info */
    ret = it8951_drv_usb_bulk_recv(dev, info, sizeof(*info));
    if (ret != 0) {
        goto out_error;
    }

    /* Check command status */
    ret = it8951_read_csw(dev);

out_error:
    if (cbw != NULL) {
        kfree(cbw);
    }

    return ret;
}

static int it8951_set_vcom(const struct it8951_device *dev, int16_t vcom_value)
{
    struct cmd_block_wrapper *cbw;
    int ret;

    /* Create command block wrapper */
    cbw = it8951_create_cbw(DIR_BULK_OUT, 0);
    if (cbw == NULL) {
        ret = -ENOMEM;
        goto out_error;
    }
    cbw->cmd_data[0] = IT8951_DRV_CUSTOM_CMD;
    cbw->cmd_data[6] = IT8951_DRV_PMIC_CTRL_OP;
    *(uint16_t *)&cbw->cmd_data[7] = cpu_to_be16(vcom_value);
    cbw->cmd_data[9] = 1; // 1 - set VCOM value, 0 - ignore
    cbw->cmd_data[10] = 1; // 1 - set power flag, 0 - ignore
    cbw->cmd_data[11] = 1; // 1 - power on, 0 - power off

    /* Send command block wrapper */
    ret = it8951_drv_usb_bulk_send(dev, cbw, sizeof(*cbw));
    if (ret != 0) {
        goto out_error;
    }

    /* Check command status */
    ret = it8951_read_csw(dev);

out_error:
    if (cbw != NULL) {
        kfree(cbw);
    }

    return ret;
}

static int it8951_image_load_fast(const struct it8951_device *dev, uint8_t *image, size_t x, size_t y, size_t w, size_t h)
{
    struct cmd_block_wrapper *cbw;
    size_t offset = 0;
    size_t bytes_left;
    size_t chunk_size;
    int ret;

    /* Create command block wrapper */
    cbw = it8951_create_cbw(DIR_BULK_OUT, 0);
    if (cbw == NULL) {
        ret = -ENOMEM;
        goto out_error;
    }
    cbw->cmd_data[0] = IT8951_DRV_CUSTOM_CMD;
    cbw->cmd_data[6] = IT8951_DRV_FAST_WRITE_MEM_OP;

    /* Total image size */
    bytes_left = w * h;

    /* Send data in chunks of at most IT8951_DRV_MAX_BLOCK_SIZE bytes */
    while (bytes_left > 0) {
        chunk_size = min(IT8951_DRV_MAX_BLOCK_SIZE, bytes_left);

        /* Fill command block wrapper data */
        cbw->data_transfer_length = chunk_size;
        *(uint32_t *)&cbw->cmd_data[2] = cpu_to_be32(dev->img_mem_addr + offset);
        *(uint32_t *)&cbw->cmd_data[7] = cpu_to_be16(chunk_size);

        /* Send command block wrapper */
        ret = it8951_drv_usb_bulk_send(dev, cbw, sizeof(*cbw));
        if (ret != 0) {
            goto out_error;
        }

        /* Send image data */
        ret = it8951_drv_usb_bulk_send(dev, &image[offset], chunk_size);
        if (ret != 0) {
            goto out_error;
        }

        /* Check command status */
        ret = it8951_read_csw(dev);
        if (ret != 0) {
            goto out_error;
        }

        bytes_left -= chunk_size;
        offset += chunk_size;
    }

out_error:
    if (cbw != NULL) {
        kfree(cbw);
    }

    return ret;
}

static int it8951_display_refresh(const struct it8951_device *dev, enum it8951_refresh_mode mode, size_t x, size_t y, size_t w, size_t h)
{
    struct cmd_block_wrapper *cbw = NULL;
    struct it8951_dpy_area *area = NULL;
    int ret;

    /* Create command block wrapper */
    cbw = it8951_create_cbw(DIR_BULK_OUT, sizeof(*area));
    if (cbw == NULL) {
        ret = -ENOMEM;
        goto out_error;
    }
    cbw->cmd_data[0] = IT8951_DRV_CUSTOM_CMD;
    cbw->cmd_data[6] = IT8951_DRV_DPY_AREA_OP;

    /* Fill area payload */
    area = kzalloc(sizeof(*area), GFP_KERNEL);
    if (area == NULL) {
        ret = -ENOMEM;
        goto out_error;
    }
    area->img_buf_addr = cpu_to_be32(dev->img_mem_addr);
    area->mode = cpu_to_be32(mode);
    area->x = cpu_to_be32(x);
    area->y = cpu_to_be32(y);
    area->width = cpu_to_be32(w);
    area->height = cpu_to_be32(h);
    area->wait_ready = cpu_to_be32(1); // 1 - wait until refresh finished, 0 - do not wait

    /* Send CBW */
    ret = it8951_drv_usb_bulk_send(dev, cbw, sizeof(*cbw));
    if (ret != 0) {
        goto out_error;
    }

    /* Send data */
    ret = it8951_drv_usb_bulk_send(dev, area, sizeof(*area));
    if (ret != 0) {
        goto out_error;
    }

    /* Check command status */
    ret = it8951_read_csw(dev);

out_error:
    if (cbw != NULL) {
        kfree(cbw);
    }
    if (area != NULL) {
        kfree(area);
    }

    return ret;
}

static void it8951_display_update(struct fb_info *info, struct list_head *pagelist)
{
    struct it8951_device *dev = info->par;
    enum it8951_refresh_mode refresh_mode;
    int ret;

    /* Prevent crash on USB disconnection */
    if (!device_connected) {
        return;
    }

    // TODO convert to grayscale
    // TODO optimize this mirroring algorithm

    /* Mirror image */
    for (size_t i = 0; i < dev->width * dev->height; ++i) {
	    size_t row = i / info->fix.line_length;
        size_t pixel = i % info->fix.line_length;
        size_t img_offset = (row + 1) * info->fix.line_length - pixel - 1;

        dev->img_video_buf[img_offset] = (dev->fb_video_buf[i] != 0x00) ? 0xF0 : 0x00;
    }
    
    /* Load image */
    ret = it8951_image_load_fast(dev, dev->img_video_buf, 0, 0, dev->width, dev->height);
    if (ret != 0) {
        dev_err(&dev->interface->dev, "Failed to transfer image to controller, error: %d!\n", ret);
        return;
    }
	
    /* Select a refresh mode and perform the refresh */
    if (dev->fast_refresh_count >= max_fast_refreshes) {
	dev->fast_refresh_count = 0;
	refresh_mode = REFRESH_GC16;
    }
    else {
	++dev->fast_refresh_count;
	refresh_mode = REFRESH_DU2;
    }
    ret = it8951_display_refresh(dev, refresh_mode, 0, 0, dev->width, dev->height);
    if (ret != 0) {
        dev_err(&dev->interface->dev, "Failed to refresh, error: %d!\n", ret);
    }
}

static ssize_t it8951_write(struct fb_info *p, const char __user *buf, size_t count, loff_t *ppos)
{
    ssize_t ret;

    ret = fb_sys_write(p, buf, count, ppos);
    schedule_delayed_work(&p->deferred_work, p->fbdefio->delay);

    return ret;
}

static void it8951_fillrect(struct fb_info *p, const struct fb_fillrect *rect)
{
    sys_fillrect(p, rect);
    schedule_delayed_work(&p->deferred_work, p->fbdefio->delay);
}

static void it8951_copyarea(struct fb_info *p, const struct fb_copyarea *area)
{
    sys_copyarea(p, area);
    schedule_delayed_work(&p->deferred_work, p->fbdefio->delay);
}

static void it8951_imageblit(struct fb_info *p, const struct fb_image *image)
{
    sys_imageblit(p, image);
    schedule_delayed_work(&p->deferred_work, p->fbdefio->delay);
}

/* This is needed for fbcon to work in TRUECOLOR or DIRECTCOLOR visual */
static int it8951_setcolreg(unsigned regno, unsigned r, unsigned g, unsigned b, unsigned alpha, struct fb_info *info)
{
    uint32_t val;

    if (regno > 255) {
        return -EINVAL;
    }

    /* Create pseudo-palette */
    if ((info->fix.visual == FB_VISUAL_TRUECOLOR) || (info->fix.visual == FB_VISUAL_DIRECTCOLOR)) {
        if (regno > 15) {
            return -EINVAL;
        }

        val = (r << info->var.red.offset) | 
	      (g << info->var.green.offset) |
	      (b << info->var.blue.offset);


        ((uint32_t *)info->pseudo_palette)[regno] = val;
    }

    return 0;
}

static struct fb_ops it8951_fb_fops = {
        .owner = THIS_MODULE,
        .fb_read = fb_sys_read,
        .fb_write = it8951_write,
        .fb_fillrect = it8951_fillrect,
        .fb_copyarea = it8951_copyarea,
        .fb_imageblit = it8951_imageblit,
	    .fb_setcolreg = it8951_setcolreg
};

static struct fb_deferred_io it8951_fb_defio = {
        .delay	= HZ / 4,
        .deferred_io = &it8951_display_update,
};

static int it8951_drv_usb_probe(struct usb_interface *interface, const struct usb_device_id *id)
{
    struct it8951_device *dev;
    struct usb_endpoint_descriptor *bulk_in;
    struct usb_endpoint_descriptor *bulk_out;
    struct it8951_dev_info *info;
    int ret;

    /* Create device */
    dev = kzalloc(sizeof(*dev), GFP_KERNEL);
    if (dev == NULL) {
        dev_err(&interface->dev, "Failed to allocate memory for device struct!\n");
        ret = -ENOMEM;
        goto usb_error;
    }
    dev->udev = usb_get_dev(interface_to_usbdev(interface));
    dev->interface = interface;

    /* Set up the endpoint information */
    ret = usb_find_common_endpoints(interface->cur_altsetting, &bulk_in, &bulk_out, NULL, NULL);
    if (ret != 0) {
        dev_err(&interface->dev, "Failed to find bulk-in or bulk-out endpoints, error: %d!\n", ret);
        goto usb_error;
    }
    dev->bulk_in_addr = bulk_in->bEndpointAddress;
    dev->bulk_out_addr = bulk_out->bEndpointAddress;

    /* Store pointer to device in interface */
    usb_set_intfdata(interface, dev);

    /* Get controller info */
    info = kzalloc(sizeof(*info), GFP_KERNEL);
    if (info == NULL) {
        dev_err(&interface->dev, "Failed to allocate memory for controller info!\n");
        ret = -ENOMEM;
        goto usb_error;
    }
    ret = it8951_get_info(dev, info);
    if (ret) {
        dev_err(&interface->dev, "Failed to get controller info, error: %d!\n", ret);
        kfree(info);
        goto usb_error;
    }
    dev->width = be32_to_cpu(info->width);
    dev->height = be32_to_cpu(info->height);
    dev->img_mem_addr = be32_to_cpu(info->img_buf_addr);
    dev->fb_video_buf_size = dev->width * dev->height * it8951_var.bits_per_pixel / IT8951_DRV_BITS_PER_BYTE;
    dev->img_video_buf_size = dev->width * dev->height; // IT8951 always expects 1 byte per pixel
    
    /* Power on the display and set VCOM value */
    ret = it8951_set_vcom(dev, vcom_value_mv);
    if (ret != 0) {
        dev_err(&interface->dev, "Failed to set VCOM value, error: %d!\n", ret);
        kfree(info);
        goto usb_error;
    }

    dev_info(&interface->dev, "Controller info:\n");
    dev_info(&interface->dev, "\tCommand table version: 0x%08X\n", be32_to_cpu(info->version));
    dev_info(&interface->dev, "\tResolution: %zupx x %zupx\n", dev->width, dev->height);
    dev_info(&interface->dev, "\tImage buffer address: 0x%08X\n", dev->img_mem_addr);
    dev_info(&interface->dev, "\tVCOM value: -%umV\n", vcom_value_mv);

    kfree(info);

    /* Clear display */
    ret = it8951_display_refresh(dev, REFRESH_INIT, 0, 0, dev->width, dev->height);
    if (ret != 0) {
        dev_err(&interface->dev, "Failed to clear display, error: %d!\n", ret);
        goto usb_error;
    }

    /* Force first refresh to be deep */
    dev->fast_refresh_count = max_fast_refreshes; // TODO this is a workaround probably needed only for a specific display

    /* Allocate framebuffer */
    dev->fbinfo = framebuffer_alloc(0, &interface->dev);
    if (dev->fbinfo == NULL) {
        dev_err(&interface->dev, "Failed to allocate framebuffer!\n");
        ret = -ENOMEM;
        goto fb_error;
    }

    /* Allocate framebuffer video memory */
    dev->fb_video_buf = vmalloc(dev->fb_video_buf_size);
    if (dev->fb_video_buf == NULL) {
        dev_err(&interface->dev, "Failed to allocate framebuffer video memory!\n");
        ret = -ENOMEM;
        goto fb_error;
    }

    /* Allocate display video memory */
    dev->img_video_buf = kzalloc(dev->img_video_buf_size, GFP_KERNEL);
    if (dev->img_video_buf == NULL) {
        dev_err(&interface->dev, "Failed to allocate display video memory!\n");
        ret = -ENOMEM;
        goto fb_error;
    }

    /* Configure fixed parameters */
    it8951_fix.line_length = dev->width; // IT8951 always expects 1 byte per pixel
    it8951_fix.smem_start = (unsigned long)dev->fb_video_buf;
    it8951_fix.smem_len = dev->fb_video_buf_size;

    /* Configure variable parameters */
    it8951_var.xres = dev->width;
    it8951_var.yres = dev->height;
    it8951_var.xres_virtual = dev->width;
    it8951_var.yres_virtual = dev->height;
    it8951_var.width = dev->width;
    it8951_var.height = dev->height;

    /* Configure framebuffer device */
    dev->fbinfo->par = dev;
    dev->fbinfo->screen_base = (char __force __iomem *)dev->fb_video_buf;
    dev->fbinfo->screen_size = dev->fb_video_buf_size;
    dev->fbinfo->fbops = &it8951_fb_fops;
    dev->fbinfo->fix = it8951_fix;
    dev->fbinfo->var = it8951_var;
    dev->fbinfo->fbdefio = &it8951_fb_defio;
    dev->fbinfo->pseudo_palette = dev->pseudo_palette;
    dev->fbinfo->flags = FBINFO_VIRTFB;

    /* Initialize deferred IO */
    fb_deferred_io_init(dev->fbinfo);

    /* Register new framebuffer */
    ret = register_framebuffer(dev->fbinfo);
    if (ret < 0) {
        dev_err(&interface->dev, "Failed to register framebuffer, error: %d!\n", ret);
        goto fb_error;
    }

    /* Success! */
    device_connected = true;

    dev_info(&interface->dev, "IT8951 E-Ink USB display connected!\n");
    goto success;

fb_error:
    if (dev->img_video_buf != NULL) {
        kfree(dev->img_video_buf);
    }
    if (dev->fb_video_buf != NULL) {
        vfree(dev->fb_video_buf);
    }
    if (dev->fbinfo != NULL) {
        framebuffer_release(dev->fbinfo);
    }
usb_error:
    if (dev != NULL) {
        kfree(dev);
    }
success:
    return ret;
}

static void it8951_drv_usb_disconnect(struct usb_interface *interface)
{
    struct it8951_device *dev;

    dev = usb_get_intfdata(interface);
    usb_set_intfdata(interface, NULL);

    /* Device is not connected anymore */
    device_connected = false;

    if (dev != NULL) {
        dev_info(&interface->dev, "Cleaning up...\n");

        /* Cleanup deferred IO */
        fb_deferred_io_cleanup(dev->fbinfo);

        /* Remove framebuffer */
        if (dev->fbinfo != NULL) {
            unregister_framebuffer(dev->fbinfo);
            framebuffer_release(dev->fbinfo);
        }

        /* Free video memory */
        if (dev->fb_video_buf != NULL) {
            vfree(dev->fb_video_buf);
        }

        /* Free display memory */
        if (dev->img_video_buf != NULL) {
            kfree(dev->img_video_buf);
        }

        /* Free device data */
        kfree(dev);
    }

    dev_info(&interface->dev, "IT8951 E-Ink USB display disconnected!\n");
}

static struct usb_driver it8951_drv = {
        .name = IT8951_DRV_ID,
        .probe = it8951_drv_usb_probe,
        .disconnect = it8951_drv_usb_disconnect,
        .id_table = it8951_usb_ids,
        .supports_autosuspend = 0 // TODO is it ok?
};

module_usb_driver(it8951_drv)

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Lefucjusz <elektromarcin@gmail.com>");
MODULE_DESCRIPTION("IT8951 E-Ink USB framebuffer driver");
MODULE_VERSION("0.1");
