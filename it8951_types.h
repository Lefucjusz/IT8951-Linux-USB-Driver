#pragma once

#include <linux/usb.h>
#include <linux/fb.h>
#include <linux/types.h>

struct cmd_block_wrapper
{
    uint32_t signature;
    uint32_t tag;
    uint32_t data_transfer_length;
    uint8_t flags;
    uint8_t lun;
    uint8_t cmd_length;
    uint8_t cmd_data[16];
} __attribute__((packed));

struct cmd_status_wrapper
{
    uint32_t signature;
    uint32_t tag;
    uint32_t data_residue;
    uint8_t status;
} __attribute__((packed));

struct it8951_device
{
    /* USB-related data */
    struct usb_device *udev;
    struct usb_interface *interface;
    uint8_t bulk_in_addr;
    uint8_t bulk_out_addr;

    /* IT8951-related data */
    uint32_t img_mem_addr;
    uint8_t *img_video_buf;
    size_t img_video_buf_size;
    size_t width;
    size_t height;
    size_t fast_refresh_count;

    /* Framebuffer-related data */
    struct fb_info *fbinfo;
    uint8_t *fb_video_buf;
    size_t fb_video_buf_size;
    uint32_t pseudo_palette[16];
};

struct it8951_dev_info
{
    uint32_t std_cmd_num;
    uint32_t ext_cmd_num;
    uint32_t signature;
    uint32_t version;
    uint32_t width;
    uint32_t height;
    uint32_t update_buf_addr;
    uint32_t img_buf_addr;
    uint32_t temperature_segment;
    uint32_t disp_mode;
    uint32_t frame_count[8];
    uint32_t buffer_count;
    uint32_t reserved[9];
    void *command_table;
} __attribute__((packed));

struct it8951_ld_img
{
    uint32_t img_buf_addr;
    uint32_t x;
    uint32_t y;
    uint32_t w;
    uint32_t h;
} __attribute__((packed));

struct it8951_dpy_area
{
    uint32_t img_buf_addr;
    uint32_t mode;
    uint32_t x;
    uint32_t y;
    uint32_t width;
    uint32_t height;
    uint32_t wait_ready;
} __attribute__((packed));

enum it8951_cmd_dir
{
    DIR_BULK_OUT = 0x00,
    DIR_BULK_IN = 0x80
};

enum it8951_refresh_mode
{
    REFRESH_INIT = 0,
    REFRESH_DU2,
    REFRESH_GC16
};
