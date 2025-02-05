#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/usb.h>
#include <linux/fb.h>
#include <linux/time.h>
#include "it8951_types.h"

// TODO RGB888 to gray2/gray4
// TODO add parameter to set VCOM
// TODO check gotos and memory freeing
// TODO optimize data loading time
// TODO add partial updates

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
#define IT8951_DRV_MAX_BLOCK_SIZE (60 * 1024)

/* General defines */
#define IT8951_DRV_BITS_PER_BYTE 8

#define ROUND_DOWN_TO_MULTIPLE_OF(x, mul) (((x) / (mul)) * (mul))

static const struct usb_device_id it8951_usb_ids[] = {
        {USB_DEVICE(IT8951_DRV_USB_VID, IT8951_DRV_USB_PID)},
        { }
};

MODULE_DEVICE_TABLE(usb, it8951_usb_ids);

static struct fb_fix_screeninfo it8951_fix = {
        .id = IT8951_DRV_ID,
        .type = FB_TYPE_PACKED_PIXELS,
        .visual = FB_VISUAL_TRUECOLOR,
        .accel = FB_ACCEL_NONE
};

static struct fb_var_screeninfo it8951_var = {
        .bits_per_pixel = 8, /* RGB332 */
        .red = {5, 3, 0},
        .green = {2, 3, 0},
        .blue = {0, 2, 0},
        .activate = FB_ACTIVATE_NOW,
        .vmode = FB_VMODE_NONINTERLACED
};

static int it8951_drv_usb_bulk_send(const struct it8951_device *dev, void *data, size_t size)
{
    uint8_t *data_ptr = data;
    size_t bytes_left = size;
    size_t offset = 0;
    size_t chunk_size;
    int bytes_received;
    int ret;
    unsigned pipe;

    /* Get send pipe */
    pipe = usb_sndbulkpipe(dev->udev, dev->bulk_out_addr);

    /* Send data in packets */
    while (bytes_left > 0) {
        chunk_size = min(bytes_left, dev->bulk_out_size);
        ret = usb_bulk_msg(dev->udev, pipe, &data_ptr[offset], chunk_size, &bytes_received, IT8951_DRV_USB_TIMEOUT_MS);
        if (ret != 0) {
            return -EIO;
        }
        bytes_left -= chunk_size;
        offset += chunk_size;
    }

    return 0;
}

static int it8951_drv_usb_bulk_recv(const struct it8951_device *dev, void *data, size_t size) // TODO fix transfers above packet size
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

static int it8951_image_load(const struct it8951_device *dev, uint8_t *image, size_t x, size_t y, size_t w, size_t h)
{
    struct cmd_block_wrapper *cbw = NULL;
    struct it8951_ld_img *area = NULL;
    size_t bytes_sent = 0;
    size_t max_transfer_size;
    size_t total_size;
    size_t lines_sent;
    size_t chunk_size;
    int ret;

    /* Create command block wrapper */
    cbw = it8951_create_cbw(DIR_BULK_OUT, 0);
    if (cbw == NULL) {
        ret = -ENOMEM;
        goto out_error;
    }
    cbw->cmd_data[0] = IT8951_DRV_CUSTOM_CMD;
    cbw->cmd_data[6] = IT8951_DRV_LD_IMG_AREA_OP;

    /* Allocate command data */
    area = kzalloc(sizeof(*area), GFP_KERNEL);
    if (area == NULL) {
        ret = -ENOMEM;
        goto out_error;
    }

    /* Maximum transfer size to send an integer number of lines */
    max_transfer_size = ROUND_DOWN_TO_MULTIPLE_OF(IT8951_DRV_MAX_BLOCK_SIZE - sizeof(*area), w);

    /* Total image size */
    total_size = w * h;

    /* Send data in bands of IT8951_DRV_MAX_BLOCK_SIZE size */
    while (bytes_sent < total_size) {
        chunk_size = min(max_transfer_size, total_size - bytes_sent);
        lines_sent = bytes_sent / w;

        /* Set data transfer length */
        cbw->data_transfer_length = chunk_size + sizeof(*area);

        /* Send command block wrapper */
        ret = it8951_drv_usb_bulk_send(dev, cbw, sizeof(*cbw));
        if (ret != 0) {
            goto out_error;
        }

        /* Set command data */
        area->img_buf_addr = cpu_to_be32(dev->img_mem_addr);
        area->x = cpu_to_be32(x);
        area->y = cpu_to_be32(y + lines_sent);
        area->w = cpu_to_be32(w);
        area->h = cpu_to_be32(h - lines_sent);

        /* Send command data */
        ret = it8951_drv_usb_bulk_send(dev, area, sizeof(*area));
        if (ret != 0) {
            goto out_error;
        }

        /* Send image data */
        ret = it8951_drv_usb_bulk_send(dev, &image[bytes_sent], chunk_size);
        if (ret != 0) {
            goto out_error;
        }

        /* Check command status */
        ret = it8951_read_csw(dev);
        if (ret != 0) {
            goto out_error;
        }

        bytes_sent += chunk_size;
    }

out_error:
    if (area != NULL) {
        kfree(area);
    }
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
    struct fb_deferred_io_pageref *pageref;
    size_t y1, y2;
    size_t width, height;
    uint8_t r, g, b;
    size_t min_y = SIZE_MAX;
    size_t max_y = 0;
    size_t offset;

    if (list_empty(pagelist)) {
        dev_err(&dev->interface->dev, "Requested refresh with empty pages list!\n");
        return;
    }

    /* Compute size of refresh rectangle enclosing all refreshed pages */
    list_for_each_entry(pageref, pagelist, list) {
        /* Compute page span across y axis */
        y1 = pageref->offset / info->fix.line_length;
        y2 = (pageref->offset + PAGE_SIZE) / info->fix.line_length;

        /* Clip to screen size */
        if (y2 >= info->var.yres) {
            y2 = info->var.yres - 1;
        }

        /* Update span */
        if (y1 < min_y) {
            min_y = y1;
        }
        if (y2 > max_y) {
            max_y = y2;
        }
    }
    width = info->var.xres;
    height = max_y - min_y;

    /* Convert from RGB332 to grayscale */
    for (size_t i = 0; i < width * height; ++i) {
        offset = i + width * min_y;
        r = (dev->fb_video_buf[offset] >> 5) & 0x07;
        g = (dev->fb_video_buf[offset] >> 2) & 0x07;
        b = (dev->fb_video_buf[offset] >> 0) & 0x03;

        dev->img_video_buf[offset] = (11 * r) + (21 * g) + (10 * b);
    }

    printk("Loading from origin (%d; %d), width: %d, height: %d\n", 0, min_y, width, height);

//    ktime_t s = ktime_get_ns();
    int ret = it8951_image_load(dev, &dev->img_video_buf[offset], 0, min_y, width, height);
//    ktime_t e = ktime_get_ns();
//    printk("ret load: %d, time: %ums\n", ret, (e - s) / 1000000);

    ret = it8951_display_refresh(dev, 2, 0, min_y, width, height); // TODO check return
    printk("ret refresh: %d\n", ret);
}

static ssize_t it8951_write(struct fb_info *p, const char __user *buf, size_t count, loff_t *ppos)
{
    printk("write\n");
    ssize_t ret;

    ret = fb_sys_write(p, buf, count, ppos);
    schedule_delayed_work(&p->deferred_work, p->fbdefio->delay);

    return ret;
}

static void it8951_fillrect(struct fb_info *p, const struct fb_fillrect *rect)
{
    printk("fillrect\n");
    sys_fillrect(p, rect);
    schedule_delayed_work(&p->deferred_work, p->fbdefio->delay);
}

static void it8951_copyarea(struct fb_info *p, const struct fb_copyarea *area)
{
    printk("copyarea\n");
    sys_copyarea(p, area);
    schedule_delayed_work(&p->deferred_work, p->fbdefio->delay);
}

static void it8951_imageblit(struct fb_info *p, const struct fb_image *image)
{
    printk("imageblit\n");
    sys_imageblit(p, image);
    schedule_delayed_work(&p->deferred_work, p->fbdefio->delay);
}

static struct fb_ops it8951_fb_fops = {
        .owner = THIS_MODULE,
        .fb_read = fb_sys_read,
        .fb_write = it8951_write,
        .fb_fillrect = it8951_fillrect,
        .fb_copyarea = it8951_copyarea,
        .fb_imageblit = it8951_imageblit
};

static struct fb_deferred_io it8951_fb_defio = {
        .delay	= HZ / 2,
        .deferred_io = &it8951_display_update,
};

static int it8951_drv_usb_probe(struct usb_interface *interface, const struct usb_device_id *id)
{
    struct it8951_device *dev;
    struct usb_endpoint_descriptor *bulk_in;
    struct usb_endpoint_descriptor *bulk_out;
    struct it8951_dev_info *info;
    int ret = 0;

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
    dev->bulk_out_size = usb_endpoint_maxp(bulk_out);

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

    dev_info(&interface->dev, "Controller info:\n");
    dev_info(&interface->dev, "\tCommand table version: 0x%08X\n", be32_to_cpu(info->version));
    dev_info(&interface->dev, "\tResolution: %zupx x %zupx\n", dev->width, dev->height);
    dev_info(&interface->dev, "\tImage buffer address: 0x%08X\n", dev->img_mem_addr);

    kfree(info);

    /* Clear display */
    ret = it8951_display_refresh(dev, REFRESH_INIT, 0, 0, dev->width, dev->height);
    if (ret != 0) {
        dev_err(&interface->dev, "Failed to clear display, error: %d!\n", ret);
        goto usb_error;
    }

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
    dev->fbinfo->pseudo_palette = NULL; // TODO what's this?
    dev->fbinfo->flags = FBINFO_VIRTFB;

    /* Initialize deferred IO */
    fb_deferred_io_init(dev->fbinfo);

    /* Register new framebuffer */
    ret = register_framebuffer(dev->fbinfo);
    if (ret < 0) {
        dev_err(&interface->dev, "Failed to register framebuffer, error: %d!\n", ret);
        goto fb_error;
    }

//    printk("npagerefs: %d\n", dev->fbinfo->fbdefio->npagerefs);

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
