#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/usb.h>

#include <drm/drm_drv.h>
#include <drm/drm_connector.h>
#include <drm/drm_modes.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_fbdev_generic.h>
#include <drm/drm_simple_kms_helper.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_modeset_helper.h>
#include <drm/drm_gem_shmem_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_gem_atomic_helper.h>
#include <drm/drm_damage_helper.h>

#include "it8951_types.h"

// TODO resolve system lagging issue on some platforms (workqueue for refresh?)

/* Driver metadata */
#define IT8951_DISP_NAME "it8951"
#define IT8951_DISP_DESC "IT8951 E-Ink USB display driver"
#define IT8951_DISP_DATE "20250107"
#define IT8951_DISP_MAJOR 1
#define IT8951_DISP_MINOR 0

/* USB related defines */
#define IT8951_DISP_USB_VID 0x048D /* ITE Tech. Inc. */
#define IT8951_DISP_USB_PID 0x8951 /* IT8951 */
#define IT8951_DISP_LUN 0
#define IT8951_DISP_USB_TIMEOUT_MS 2500

/* IT8951 command related defines */
#define IT8951_DISP_INQUIRY_CMD 0x12
#define IT8951_DISP_CUSTOM_CMD 0xFE
#define IT8951_DISP_GET_SYS_OP 0x80
#define IT8951_DISP_READ_MEM_OP 0x81
#define IT8951_DISP_WRITE_MEM_OP 0x82
#define IT8951_DISP_DPY_AREA_OP 0x94
#define IT8951_DISP_LD_IMG_AREA_OP 0xA2
#define IT8951_DISP_PMIC_CTRL_OP 0xA3
#define IT8951_DISP_FAST_WRITE_MEM_OP 0xA5
#define IT8951_DISP_AUTO_RESET_OP 0xA7
#define IT8951_DISP_CBW_SIGNATURE 0x43425355 // 'USBD'
#define IT8951_DISP_GET_SYS_SIGNATURE 0x31353938 // '8951'
#define IT8951_DISP_GET_SYS_VERSION 0x00020001

/* General defines */
#define IT8951_DISP_MAX_BLOCK_SIZE (60 * 1024U)
#define IT8951_DISP_COLOR_MASK 0xF0
#define IT8951_DISP_BITS_PER_BYTE 8
#define IT8951_DISP_DEFAULT_VCOM 2480
#define IT8951_DISP_MAX_FAST_REFRESHES 100

/* Macros */
#define ROUND_DOWN_TO_MULTIPLE(x, mul) (((x) / (mul)) * (mul))
#define CLAMP(x, lo, hi) max(lo, min(x, hi))

/* Dithering related defines */
#define IT8951_DITHER_BLACK 0
#define IT8951_DITHER_WHITE 255
#define IT8951_DITHER_QUANTIZE(x) ((x > 127) ? IT8951_DITHER_WHITE : IT8951_DITHER_BLACK)

/* Parameters */
static int vcom_value_mv = IT8951_DISP_DEFAULT_VCOM;
module_param(vcom_value_mv, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(vcom_value_mv, "VCOM voltage value in mV, without sign");

static int max_fast_refreshes = IT8951_DISP_MAX_FAST_REFRESHES;
module_param(max_fast_refreshes, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(max_fast_refreshes, "Number of fast refreshes before a deep refresh occurs");

// #define USE_DITHERING // TODO dithering seems to not work very well with my display, disable for now

static int it8951_disp_usb_bulk_send(const struct it8951_device *dev, void *data, size_t size)
{
    int bytes_received;
    unsigned pipe;

    /* Get send pipe */
    pipe = usb_sndbulkpipe(dev->udev, dev->bulk_out_addr);

    /* Send data */
    return usb_bulk_msg(dev->udev, pipe, data, size, &bytes_received, IT8951_DISP_USB_TIMEOUT_MS);
}

static int it8951_disp_usb_bulk_recv(const struct it8951_device *dev, void *data, size_t size)
{
    int bytes_sent;
    unsigned pipe;

    /* Get receive pipe */
    pipe = usb_rcvbulkpipe(dev->udev, dev->bulk_in_addr);

    /* Receive data */
    return usb_bulk_msg(dev->udev, pipe, data, size, &bytes_sent, IT8951_DISP_USB_TIMEOUT_MS);
}

static struct cmd_block_wrapper *it8951_disp_create_cbw(enum it8951_cmd_dir dir, size_t data_xfer_length)
{
    struct cmd_block_wrapper *cbw;

    /* Create command block wrapper */
    cbw = kzalloc(sizeof(*cbw), GFP_KERNEL);
    if (cbw == NULL) {
        return NULL;
    }

    /* Fill basic data */
    cbw->signature = IT8951_DISP_CBW_SIGNATURE;
    cbw->data_transfer_length = data_xfer_length;
    cbw->flags = dir;
    cbw->lun = IT8951_DISP_LUN;
    cbw->cmd_length = sizeof(cbw->cmd_data);

    return cbw;
}

static int it8951_disp_read_csw(const struct it8951_device *dev)
{
    struct cmd_status_wrapper *csw;
    int ret;

    csw = kzalloc(sizeof(*csw), GFP_KERNEL);
    if (csw == NULL) {
        ret = -ENOMEM;
        goto out_error;
    }

    ret = it8951_disp_usb_bulk_recv(dev, csw, sizeof(*csw));
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

static int it8951_disp_get_info(const struct it8951_device *dev, struct it8951_dev_info *info)
{
    struct cmd_block_wrapper *cbw;
    int ret;

    /* Create command block wrapper */
    cbw = it8951_disp_create_cbw(DIR_BULK_IN, sizeof(*info));
    if (cbw == NULL) {
        ret = -ENOMEM;
        goto out_error;
    }
    cbw->cmd_data[0] = IT8951_DISP_CUSTOM_CMD;
    *(uint32_t *)&cbw->cmd_data[2] = IT8951_DISP_GET_SYS_SIGNATURE;
    cbw->cmd_data[6] = IT8951_DISP_GET_SYS_OP;
    *(uint32_t *)&cbw->cmd_data[8] = IT8951_DISP_GET_SYS_VERSION;

    /* Send command block wrapper */
    ret = it8951_disp_usb_bulk_send(dev, cbw, sizeof(*cbw));
    if (ret != 0) {
        goto out_error;
    }

    /* Receive info */
    ret = it8951_disp_usb_bulk_recv(dev, info, sizeof(*info));
    if (ret != 0) {
        goto out_error;
    }

    /* Check command status */
    ret = it8951_disp_read_csw(dev);

out_error:
    if (cbw != NULL) {
        kfree(cbw);
    }

    return ret;
}

static int it8951_disp_set_vcom(const struct it8951_device *dev, int16_t vcom_value)
{
    struct cmd_block_wrapper *cbw;
    int ret;

    /* Create command block wrapper */
    cbw = it8951_disp_create_cbw(DIR_BULK_OUT, 0);
    if (cbw == NULL) {
        ret = -ENOMEM;
        goto out_error;
    }
    cbw->cmd_data[0] = IT8951_DISP_CUSTOM_CMD;
    cbw->cmd_data[6] = IT8951_DISP_PMIC_CTRL_OP;
    *(uint16_t *)&cbw->cmd_data[7] = cpu_to_be16(vcom_value);
    cbw->cmd_data[9] = 1; // 1 - set VCOM value, 0 - ignore
    cbw->cmd_data[10] = 1; // 1 - set power flag, 0 - ignore
    cbw->cmd_data[11] = 1; // 1 - power on, 0 - power off

    /* Send command block wrapper */
    ret = it8951_disp_usb_bulk_send(dev, cbw, sizeof(*cbw));
    if (ret != 0) {
        goto out_error;
    }

    /* Check command status */
    ret = it8951_disp_read_csw(dev);

out_error:
    if (cbw != NULL) {
        kfree(cbw);
    }

    return ret;
}

/* TODO support partial loading */
static int it8951_disp_image_load_fast(const struct it8951_device *dev, uint8_t *image, size_t x, size_t y, size_t w, size_t h)
{
    struct cmd_block_wrapper *cbw;
    size_t bytes_left, chunk_size;
    size_t offset = 0;
    int ret;

    /* Create command block wrapper */
    cbw = it8951_disp_create_cbw(DIR_BULK_OUT, 0);
    if (cbw == NULL) {
        ret = -ENOMEM;
        goto out_error;
    }
    cbw->cmd_data[0] = IT8951_DISP_CUSTOM_CMD;
    cbw->cmd_data[6] = IT8951_DISP_FAST_WRITE_MEM_OP;

    /* Total image size */
    bytes_left = w * h;

    /* Send data in chunks of at most IT8951_DISP_MAX_BLOCK_SIZE bytes */
    while (bytes_left > 0) {
        chunk_size = min(IT8951_DISP_MAX_BLOCK_SIZE, bytes_left);

        /* Fill command block wrapper data */
        cbw->data_transfer_length = chunk_size;
        *(uint32_t *)&cbw->cmd_data[2] = cpu_to_be32(dev->img_mem_addr + offset);
        *(uint32_t *)&cbw->cmd_data[7] = cpu_to_be16(chunk_size);

        /* Send command block wrapper */
        ret = it8951_disp_usb_bulk_send(dev, cbw, sizeof(*cbw));
        if (ret != 0) {
            goto out_error;
        }

        /* Send image data */
        ret = it8951_disp_usb_bulk_send(dev, &image[offset], chunk_size);
        if (ret != 0) {
            goto out_error;
        }

        /* Check command status */
        ret = it8951_disp_read_csw(dev);
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

static int it8951_disp_refresh(const struct it8951_device *dev, enum it8951_refresh_mode mode, size_t x, size_t y, size_t w, size_t h)
{
    struct cmd_block_wrapper *cbw = NULL;
    struct it8951_dpy_area *area = NULL;
    int ret;

    /* Create command block wrapper */
    cbw = it8951_disp_create_cbw(DIR_BULK_OUT, sizeof(*area));
    if (cbw == NULL) {
        ret = -ENOMEM;
        goto out_error;
    }
    cbw->cmd_data[0] = IT8951_DISP_CUSTOM_CMD;
    cbw->cmd_data[6] = IT8951_DISP_DPY_AREA_OP;

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
    ret = it8951_disp_usb_bulk_send(dev, cbw, sizeof(*cbw));
    if (ret != 0) {
        goto out_error;
    }

    /* Send data */
    ret = it8951_disp_usb_bulk_send(dev, area, sizeof(*area));
    if (ret != 0) {
        goto out_error;
    }

    /* Check command status */
    ret = it8951_disp_read_csw(dev);

out_error:
    if (cbw != NULL) {
        kfree(cbw);
    }
    if (area != NULL) {
        kfree(area);
    }

    return ret;
}

static struct it8951_device *drm_to_it8951(struct drm_device *drm)
{
    return container_of(drm, struct it8951_device, drm);
}

static void it8951_disp_image_mirror(uint8_t *buffer, size_t width, size_t height)
{
    uint8_t *row_ptr;
    uint8_t tmp;
    size_t x, y;

    for (y = 0; y < height; ++y) {
        row_ptr = &buffer[y * width];
        for (x = 0; x < width / 2; ++x) {
            tmp = row_ptr[x];
            row_ptr[x] = row_ptr[width - x - 1];
            row_ptr[width - x - 1] = tmp;
        }
    }   
}

#if defined(USE_DITHERING)

static uint8_t it8951_disp_add_saturate(int8_t x, int8_t y)
{
    return CLAMP(x + y, IT8951_DITHER_BLACK, IT8951_DITHER_WHITE);
}

static void it8951_disp_sum_pixel(uint8_t *buffer, int8_t value, size_t x, size_t y, size_t width, size_t height)
{
    if ((x >= width) || (y >= height)) {
        return;
    }
    buffer[y * width + x] = it8951_disp_add_saturate(buffer[y * width + x], value);
}

static void it8951_disp_image_dither(uint8_t *buffer, size_t width, size_t height)
{
    uint8_t pixel_orig, pixel_quant;
    int8_t error;
    size_t x, y;

    for (y = 0; y < height; ++y) {
        for (x = 1; x < width; ++x) {
            /* Quantize pixel */
            pixel_orig = buffer[y * width + x];
            pixel_quant = IT8951_DITHER_QUANTIZE(pixel_orig);
            error = pixel_orig - pixel_quant;

            /* Propagate error */
            it8951_disp_sum_pixel(buffer, error * 7 / 16, x + 1, y, width, height);
            it8951_disp_sum_pixel(buffer, error * 3 / 16, x - 1, y + 1, width, height);
            it8951_disp_sum_pixel(buffer, error * 5 / 16, x, y + 1, width, height);
            it8951_disp_sum_pixel(buffer, error * 1 / 16, x + 1, y + 1, width, height);

            /* Update pixel to quantized one */
            buffer[y * width + x] = pixel_quant & IT8951_DISP_COLOR_MASK;
        }
    }
}

#endif

static void it8951_disp_image_quantize(uint8_t *buffer, size_t width, size_t height)
{
    size_t i;

    for (i = 0; i < width * height; ++i) {
        buffer[i] = IT8951_DITHER_QUANTIZE(buffer[i]);
    }
}

static void it8951_disp_image_transform(uint8_t *buffer, size_t width, size_t height)
{
    it8951_disp_image_mirror(buffer, width, height);
#if defined(USE_DITHERING)
    it8951_disp_image_dither(buffer, width, height);
#else
    it8951_disp_image_quantize(buffer, width, height);
#endif
}

static void it8951_disp_fb_dirty(struct drm_framebuffer *fb, const struct iosys_map *vmap, struct drm_rect *rect)
{
    struct it8951_device *dev = drm_to_it8951(fb->dev);
    unsigned img_video_pitch = 0;
    struct iosys_map img_video_map;
    struct drm_format_conv_state fmtcnv_state;
    enum it8951_refresh_mode refresh_mode;
    struct drm_rect clip;
    int ret, idx;

    /* Enter critical section */
    if (!drm_dev_enter(fb->dev, &idx)) {
        dev_err(&dev->interface->dev, "Failed to lock device for HW access (device disconnected?)\n");
        return;
    }

    /* Initialize format conversion state helper */
    drm_format_conv_state_init(&fmtcnv_state);

    /* TODO for now the driver doesn't support partial updates */
    clip.x1 = 0;
    clip.x2 = dev->width;
    clip.y1 = 0;
    clip.y2 = dev->height;

    /* Prepare framebuffer for CPU access */
    ret = drm_gem_fb_begin_cpu_access(fb, DMA_FROM_DEVICE);
    if (ret != 0) {
        dev_err(&dev->interface->dev, "Failed to degin framebuffer CPU access. error: %d\n", ret);
        goto out_error;
    }

    /* Get and convert framebuffer content */
    iosys_map_set_vaddr(&img_video_map, dev->img_video_buf);
    drm_fb_xrgb8888_to_gray8(&img_video_map, &img_video_pitch, vmap, fb, &clip, &fmtcnv_state);

    /* Release resources */
    drm_gem_fb_end_cpu_access(fb, DMA_FROM_DEVICE);
    drm_format_conv_state_release(&fmtcnv_state);

    /* Transform image to display format */
    it8951_disp_image_transform(dev->img_video_buf, drm_rect_width(&clip), drm_rect_height(&clip)); 

    /* Load image */
    ret = it8951_disp_image_load_fast(dev, dev->img_video_buf, clip.x1, clip.y1, drm_rect_width(&clip), drm_rect_height(&clip));
    if (ret != 0) {
        dev_err(&dev->interface->dev, "Failed to transfer image to controller, error: %d!\n", ret);
        goto out_error;
    }

    /* Refresh display */
    if (dev->fast_refresh_count >= max_fast_refreshes) {
        dev->fast_refresh_count = 0;
        refresh_mode = REFRESH_GC16;
    }
    else {
	    ++dev->fast_refresh_count;
	    refresh_mode = REFRESH_DU2;
    }
    ret = it8951_disp_refresh(dev, refresh_mode, rect->x1, rect->y1, drm_rect_width(rect), drm_rect_height(rect));
    if (ret != 0) {
        dev_err(&dev->interface->dev, "Failed to refresh, error: %d!\n", ret);
    }

out_error:
    drm_dev_exit(idx);
}


static enum drm_mode_status it8951_disp_pipe_mode_valid(struct drm_simple_display_pipe *pipe,
                                                        const struct drm_display_mode *mode)
{
    struct drm_crtc *crtc = &pipe->crtc;
    struct it8951_device *dev = drm_to_it8951(crtc->dev);

    return drm_crtc_helper_mode_valid_fixed(crtc, mode, &dev->mode);
}

static void it8951_disp_pipe_update(struct drm_simple_display_pipe *pipe,
                                    struct drm_plane_state *old_state)
{
    struct drm_plane_state *state = pipe->plane.state;
    struct drm_shadow_plane_state *shadow_plane_state = to_drm_shadow_plane_state(state);
    struct drm_rect rect;

    if (!pipe->crtc.state->active) {
        return;
    }

    if (drm_atomic_helper_damage_merged(old_state, state, &rect)) {
        it8951_disp_fb_dirty(state->fb, shadow_plane_state->data, &rect);
    }
}

static const struct drm_simple_display_pipe_funcs it8951_disp_pipe_funcs = {
        .mode_valid = it8951_disp_pipe_mode_valid,
        .update = it8951_disp_pipe_update,
        DRM_GEM_SIMPLE_DISPLAY_PIPE_SHADOW_PLANE_FUNCS
};

static int it8951_disp_connector_get_modes(struct drm_connector *connector)
{
    struct it8951_device *dev = drm_to_it8951(connector->dev);

    return drm_connector_helper_get_modes_fixed(connector, &dev->mode);
}

static const struct drm_connector_helper_funcs it8951_disp_connector_helper_funcs = {
        .get_modes = it8951_disp_connector_get_modes
};

static const struct drm_connector_funcs it8951_disp_connector_funcs = {
        .reset = drm_atomic_helper_connector_reset,
        .fill_modes = drm_helper_probe_single_connector_modes,
        .destroy = drm_connector_cleanup,
        .atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
        .atomic_destroy_state = drm_atomic_helper_connector_destroy_state
};

static const struct drm_mode_config_funcs it8951_disp_mode_config_funcs = {
        .fb_create = drm_gem_fb_create_with_dirty,
        .atomic_check = drm_atomic_helper_check,
        .atomic_commit = drm_atomic_helper_commit
};

static const uint32_t it8951_disp_formats[] = {
        DRM_FORMAT_XRGB8888
};

DEFINE_DRM_GEM_FOPS(it8951_disp_fops);

static struct drm_driver it8951_disp_drm_driver = {
        .driver_features = DRIVER_GEM | DRIVER_MODESET | DRIVER_ATOMIC,
        .name = IT8951_DISP_NAME,
        .desc = IT8951_DISP_DESC,
        .date = IT8951_DISP_DATE,
        .major = IT8951_DISP_MAJOR,
        .minor = IT8951_DISP_MINOR,
        .fops = &it8951_disp_fops,
        DRM_GEM_SHMEM_DRIVER_OPS
};

static int it8951_disp_usb_probe(struct usb_interface *interface, const struct usb_device_id *id)
{
    struct it8951_device *dev;
    struct drm_device *drm;
    struct usb_endpoint_descriptor *bulk_in, *bulk_out;
    struct it8951_dev_info *info;
    int ret;

    /* Create DRM device */
    dev = devm_drm_dev_alloc(&interface->dev, &it8951_disp_drm_driver, struct it8951_device, drm);
    if (IS_ERR(dev)) {
        ret = PTR_ERR(dev);
        dev_err(&interface->dev, "Failed to create DRM device, error: %d!\n", ret);
        return ret;
    }
    drm = &dev->drm;
    dev->udev = usb_get_dev(interface_to_usbdev(interface));

    /* Set up the endpoints information */
    ret = usb_find_common_endpoints(interface->cur_altsetting, &bulk_in, &bulk_out, NULL, NULL);
    if (ret != 0) {
        dev_err(&interface->dev, "Failed to find bulk-in or bulk-out endpoints, error: %d!\n", ret);
        goto out_error;
    }
    dev->bulk_in_addr = bulk_in->bEndpointAddress;
    dev->bulk_out_addr = bulk_out->bEndpointAddress;

    /* Store pointer to DRM device in interface */
    usb_set_intfdata(interface, drm);

    /* Get controller info */
    info = kzalloc(sizeof(*info), GFP_KERNEL);
    if (info == NULL) {
        ret = -ENOMEM;
        dev_err(&interface->dev, "Failed to allocate memory for controller info, error: %d!\n", ret);
        goto out_error;
    }
    ret = it8951_disp_get_info(dev, info);
    if (ret != 0) {
        dev_err(&interface->dev, "Failed to get controller info, error: %d!\n", ret);
        kfree(info);
        goto out_error;
    }
    dev->width = be32_to_cpu(info->width);
    dev->height = be32_to_cpu(info->height);
    dev->img_mem_addr = be32_to_cpu(info->img_buf_addr);
    dev->img_video_buf_size = dev->width * dev->height; // IT8951 always expects 1 byte per pixel

    /* Configure display mode */
    dev->mode.type = DRM_MODE_TYPE_DRIVER;
    dev->mode.clock = 1; // Dummy value just to satisfy some checks
    dev->mode.hdisplay = dev->mode.htotal = dev->width;
    dev->mode.hsync_start = dev->mode.hsync_end = dev->width;
    dev->mode.vdisplay = dev->mode.vtotal = dev->height;
    dev->mode.vsync_start = dev->mode.vsync_end = dev->height;
    dev->mode.width_mm = 400; // TODO these are just some dummy values
    dev->mode.height_mm = 200;

    /* Power on the display and set VCOM value */
    ret = it8951_disp_set_vcom(dev, vcom_value_mv);
    if (ret != 0) {
        dev_err(&interface->dev, "Failed to set VCOM value, error: %d!\n", ret);
        kfree(info);
        goto out_error;
    }

    /* Clear display */
    ret = it8951_disp_refresh(dev, REFRESH_INIT, 0, 0, dev->width, dev->height);
    if (ret != 0) {
        dev_err(&interface->dev, "Failed to clear display, error: %d!\n", ret);
        kfree(info);
        goto out_error;
    }

    /* Print controller info */
    dev_info(&interface->dev, "Controller info:\n");
    dev_info(&interface->dev, "\tCommand table version: 0x%08X\n", be32_to_cpu(info->version));
    dev_info(&interface->dev, "\tResolution: %zupx x %zupx\n", dev->width, dev->height);
    dev_info(&interface->dev, "\tImage buffer address: 0x%08X\n", dev->img_mem_addr);
    dev_info(&interface->dev, "\tVCOM value: -%umV\n", vcom_value_mv);

    kfree(info);

    /* Allocate display video buffer */
    dev->img_video_buf = kzalloc(dev->img_video_buf_size, GFP_KERNEL);
    if (dev->img_video_buf == NULL) {
        ret = -ENOMEM;
        dev_err(&interface->dev, "Failed to allocate display video memory, error: %d!\n", ret);
        goto out_error;
    }

    /* Initialize mode config */
    ret = drmm_mode_config_init(drm);
    if (ret != 0) {
        dev_err(&interface->dev, "Failed to initialize DRM mode config, error: %d!\n", ret);
        goto out_error;
    }
    drm->mode_config.funcs = &it8951_disp_mode_config_funcs;
    drm->mode_config.min_width = drm->mode_config.max_width = dev->mode.hdisplay;
    drm->mode_config.min_height = drm->mode_config.max_height = dev->mode.vdisplay;

    /* Initialize connector */
    drm_connector_helper_add(&dev->connector, &it8951_disp_connector_helper_funcs);
    ret = drm_connector_init(drm, &dev->connector, &it8951_disp_connector_funcs, DRM_MODE_CONNECTOR_USB);
    if (ret != 0) {
        dev_err(&interface->dev, "Failed to initialize DRM connector, error: %d!\n", ret);
        goto out_error;
    }

    /* Initialize simple display pipe */
    ret = drm_simple_display_pipe_init(drm, &dev->pipe, &it8951_disp_pipe_funcs,
                                       it8951_disp_formats, ARRAY_SIZE(it8951_disp_formats),
                                       NULL, &dev->connector);
    if (ret != 0) {
        dev_err(&interface->dev, "Failed to initialize pipe config, error: %d!\n", ret);
        goto out_error;
    }

    /* Call reset callbacks */
    drm_mode_config_reset(drm);

    /* Register new DRM device */
    ret = drm_dev_register(drm, 0);
    if (ret != 0) {
        dev_err(&interface->dev, "Failed to register new DRM device, error: %d!\n", ret);
        goto out_error;
    }

    /* Setup FBDEV emulation */
    drm_fbdev_generic_setup(drm, 0);

    dev_info(&interface->dev, "IT8951 E-Ink USB display connected!\n");

    goto out_success;

out_error:
    if (dev->img_video_buf != NULL) {
        kfree(dev->img_video_buf);
    }
out_success:
    return ret;
}


/* TODO: drm_dev_unplug seems to crash window system in Ubuntu on my
 * main PC, but works fine on RPi 5 running Ubuntu too... */
static void it8951_disp_usb_disconnect(struct usb_interface *interface)
{
    struct it8951_device *dev;
    struct drm_device *drm;

    dev_info(&interface->dev, "Cleaning up...\n");

    /* Get device pointers */
    drm = usb_get_intfdata(interface);
    dev = drm_to_it8951(drm);
    usb_set_intfdata(interface, NULL);

    /* Free display video buffer */
    if (dev->img_video_buf != NULL) {
        kfree(dev->img_video_buf);
    }

    /* Shutdown DRM device */
    drm_dev_unplug(drm);
    drm_atomic_helper_shutdown(drm);

    dev_info(&interface->dev, "IT8951 E-Ink USB display disconnected!\n");
}

static const struct usb_device_id it8951_disp_usb_ids[] = {
        {USB_DEVICE(IT8951_DISP_USB_VID, IT8951_DISP_USB_PID)},
        { }
};
MODULE_DEVICE_TABLE(usb, it8951_disp_usb_ids);

static struct usb_driver it8951_disp_usb_driver = {
        .name = IT8951_DISP_NAME,
        .id_table = it8951_disp_usb_ids,
        .probe = it8951_disp_usb_probe,
        .disconnect = it8951_disp_usb_disconnect
};

module_usb_driver(it8951_disp_usb_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Lefucjusz <elektromarcin@gmail.com>");
MODULE_DESCRIPTION(IT8951_DISP_DESC);
MODULE_VERSION("0.1");
