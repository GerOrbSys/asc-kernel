/*
* Driver for the MT9J003 Sensor
*
* Copyright (C) 2014, Dimitri Emmerich
*
* Author: Dimitri Emmerich <dimitri.emmerich@gmail.com>
*
*/
#include <linux/module.h>
#include <linux/videodev2.h>
#include <linux/slab.h>
#include <linux/log2.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <media/v4l2-device.h>
#include <media/v4l2-common.h>
#include <media/v4l2-chip-ident.h>
#include <linux/string.h>
#define MAX_FRMIVALS	 1

/* Debug functions */
static int debug;
module_param(debug, bool, 0644);
MODULE_PARM_DESC(debug, "Debug level (0-1)");

static int tau320_get_control(struct v4l2_subdev *, struct v4l2_control *);
static int tau320_set_control(struct v4l2_subdev *, struct v4l2_control *);
static int tau320_queryctrl(struct v4l2_subdev *, struct v4l2_queryctrl *);

static const struct v4l2_fmtdesc tau320_formats[] = {
{
.index = 0,
.type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
.description = "Bayer (sRGB) 10 bit",
.pixelformat = V4L2_PIX_FMT_SGRBG10,
},
};
static const unsigned int tau320_num_formats = ARRAY_SIZE(tau320_formats);

struct capture_size {
unsigned long width;
unsigned long height;
};
struct frame_table {
struct capture_size framesize;
struct v4l2_fract frameintervals[MAX_FRMIVALS];
unsigned int num_frmivals;
};


/* Array of image sizes supported by tau320.  These must be ordered from
* smallest image size to largest.
*/
const static struct frame_table tau320_frame_table[] = {
{
/* QVGA */
.framesize = { 320, 240 },
.frameintervals = {
{ .numerator =1, .denominator = 30 },
},
.num_frmivals = 1
}
};

static const unsigned int tau320_num_frmsizes = ARRAY_SIZE(tau320_frame_table);

/*VIDEO OPS .s_stream*/
static int tau320_s_stream(struct v4l2_subdev *sd, int enable)
{
return 0;
}

/*VIDEO OPS .try_fmt*/
static int tau320_try_fmt(struct v4l2_subdev *sd,
  struct v4l2_format *f)
{
int ifmt, isize;

if (f->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
return -EINVAL;

for (ifmt = 0; ifmt < tau320_num_formats; ifmt++) {
if (f->fmt.pix.pixelformat == tau320_formats[ifmt].pixelformat) {
break;
}
}
/* Is requested pixelformat not found on sensor? */
if (ifmt == tau320_num_formats) {
v4l2_err(sd, "pixel format %d, not found on sensor\n",f->fmt.pix.pixelformat);
return -EINVAL;
}

for (isize = 0; isize < tau320_num_frmsizes; isize ++) {
if (f->fmt.pix.width == tau320_frame_table[isize].framesize.width &&
   f->fmt.pix.height == tau320_frame_table[isize].framesize.height){
/* Valid size */
return 0;
}
}

/* Return the closer size */
for (isize = 0; isize < tau320_num_frmsizes; isize ++) {
if (f->fmt.pix.width < tau320_frame_table[isize].framesize.width &&
   f->fmt.pix.height < tau320_frame_table[isize].framesize.height){
break;
}
}
f->fmt.pix.width = tau320_frame_table[isize].framesize.width;
f->fmt.pix.height = tau320_frame_table[isize].framesize.height;

return 0;
}

/*VIDEO OPS .s_fmt*/
static int tau320_set_fmt(struct v4l2_subdev *sd,
  struct v4l2_format *f)
{
return tau320_try_fmt(sd,f);
}

/*VIDEO OPS .enum_framesizes*/
static int tau320_enum_framesizes(struct v4l2_subdev *sd,
struct v4l2_frmsizeenum *frms)
{
int ifmt;

for (ifmt = 0; ifmt < tau320_num_formats; ifmt++) {
if (frms->pixel_format == tau320_formats[ifmt].pixelformat) {
break;
}
}
/* Is requested pixelformat not found on sensor? */
if (ifmt == tau320_num_formats) {
v4l2_err(sd, "pixel format %d, not found on sensor\n",frms->pixel_format);
return -EINVAL;
}

/* Do we already reached all discrete framesizes? */
if (frms->index >= tau320_num_frmsizes){
return -EINVAL;
}

frms->type = V4L2_FRMSIZE_TYPE_DISCRETE;
frms->discrete.width = tau320_frame_table[frms->index].framesize.width;
frms->discrete.height = tau320_frame_table[frms->index].framesize.height;

return 0;
}
/*VIDEO OPS .enum_frameintervals*/
static int tau320_enum_frameintervals(struct v4l2_subdev *sd,
struct v4l2_frmivalenum *frmi)
{
int ifmt;
struct v4l2_frmsizeenum frms;

for (ifmt = 0; ifmt < tau320_num_formats; ifmt++) {
if (frmi->pixel_format == tau320_formats[ifmt].pixelformat) {
break;
}
}
/* Is requested pixelformat not found on sensor? */
if (ifmt == tau320_num_formats) {
v4l2_err(sd, "pixel format %d, not found on sensor\n",frms.pixel_format);
return -EINVAL;
}

frms.index = 0;
frms.pixel_format = frmi->pixel_format;

/* Do we already reached all discrete framesizes? */
while (tau320_enum_framesizes(sd,&frms) >= 0) {
if (((frmi->width == frms.discrete.width) &&
(frmi->height == frms.discrete.height))){
break;
}
frms.index++;
}
if (frms.index >= tau320_num_frmsizes){
v4l2_err(sd, "Frame size:width=%d and height=%d, not supported on sensor\n",
frmi->width,frmi->height);
return -EINVAL;
}

if (frmi->index >= tau320_frame_table[frms.index].num_frmivals)
return -EINVAL;

frmi->type = V4L2_FRMSIZE_TYPE_DISCRETE;
frmi->discrete.numerator =
tau320_frame_table[frms.index].frameintervals[frmi->index].numerator;
frmi->discrete.denominator =
tau320_frame_table[frms.index].frameintervals[frmi->index].denominator;

return 0;

}

/*CORE OPS .init*/
static int tau320_Init(struct v4l2_subdev *sd, u32 val)
{
return 0;
}

/*CORE OPS .g_chip_ident*/
static int tau320_get_chip_id(struct v4l2_subdev *sd,
      struct v4l2_dbg_chip_ident *id)
{
return 0;
}

/*CORE OPS .queryctrl*/
static int tau320_queryctrl(struct v4l2_subdev *sd,
   struct v4l2_queryctrl *qctrl)
{
return 0;
}

/*CORE OPS .g_ctrl*/
static int tau320_get_control(struct v4l2_subdev *sd,
      struct v4l2_control *ctrl)
{
return 0;
}

/*CORE OPS .s_ctrl*/
static int tau320_set_control(struct v4l2_subdev *sd,
      struct v4l2_control *ctrl)
{
return 0;
}

/*SUBDEV->CORE OPS*/
static const struct v4l2_subdev_core_ops tau320_core_ops = {
.g_chip_ident = tau320_get_chip_id,
.init = tau320_Init,
.queryctrl = tau320_queryctrl,
.g_ctrl	= tau320_get_control,
.s_ctrl	= tau320_set_control,
};

/*SUBDEV->VIDEO OPS*/
static const struct v4l2_subdev_video_ops tau320_video_ops = {
.s_fmt = tau320_set_fmt,
.try_fmt = tau320_try_fmt,
.s_stream = tau320_s_stream,
.enum_framesizes = tau320_enum_framesizes,
.enum_frameintervals = tau320_enum_frameintervals,
};

/*FUNCTIONS TO USE BY THE SUBDEV*/
static const struct v4l2_subdev_ops tau320_ops = {
.core = &tau320_core_ops,
.video = &tau320_video_ops,
};

struct v4l2_subdev tau320_subdev;
EXPORT_SYMBOL(tau320_subdev);

/* Platform driver */
/*tau320_probe:Initializing the device, mapping I/O memory, registering
the interrupt handlers. The bus infrastructure provides methods to
get the addresses, interrupts numbers and other device-specific
information. Registering the device to the proper kernel framework*/
static int tau320_probe(struct platform_device *pdev)
{
v4l2_subdev_init(&tau320_subdev,&tau320_ops);
strcpy(tau320_subdev.name, "tau320_camera");
tau320_subdev.owner	= THIS_MODULE;

return 0;
}
static int tau320_remove(struct platform_device *pdev)
{
return 0;
}


/*Instantiating a platform structure*/
static struct platform_driver tau320_driver = {
.probe = tau320_probe,
.remove =	tau320_remove,
.driver	=	{
.name	=	"tau320_camera",
.owner	=	THIS_MODULE,
},
};
/*register the driver as an init process*/
static int __init tau320_init(void)
{
platform_driver_register(&tau320_driver);
return 0;
}

/*unregister the driver*/
static void __exit tau320_cleanup(void)
{
platform_driver_unregister(&tau320_driver);
}

module_init(tau320_init);
module_exit(tau320_cleanup);

MODULE_DESCRIPTION("FLIR TAU 320 Camera driver");
MODULE_AUTHOR("David Soto <david.soto@ridgerun.com>");
MODULE_LICENSE("GPL");

