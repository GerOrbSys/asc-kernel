/*
 * Driver for MT9J003 CMOS Image Sensor from Aptina
 *
 * Copyright (C) 2015, Dimitri Emmerich <me@dimatosaurus.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/videodev2.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/log2.h>
#include <linux/gpio.h>

#include <media/v4l2-device.h>
#include <media/v4l2-common.h>
#include <media/v4l2-chip-ident.h>

/* mt9j003 constants */

#define MT9J003_CHIP_VERSION_VALUE  0x2C01

#define MT9J003_PIXEL_ARRAY_WIDTH		3856
#define MT9J003_PIXEL_ARRAY_HEIGHT	2764

#define MT9J003_COLUMN_START_DEF    112
#define MT9J003_ROW_START_DEF				8

#define MT9J003_DEFAULT_WIDTH    720//3664
#define MT9J003_DEFAULT_HEIGHT   480//2748

#define MT9J003_WINDOW_WIDTH_MAX    3664
#define MT9J003_WINDOW_HEIGHT_MAX   2748
#define MT9J003_WINDOW_HEIGHT_MIN   0

#define MT9J003_MIN_HEIGHT 2
#define MT9J003_MIN_WIDTH 2

#define MT9J003_MAX_HEIGHT 2856
#define MT9J003_MAX_WIDTH 2764

#define MT9J003_ROW_START_MIN       0
#define MT9J003_COLUMN_START_MIN    0

#define MT9J003_WINDOW_WIDTH_MIN		2
#define MT9J003_WINDOW_HEIHGT_MIN   2

#define MT9J003_POWER_GPIO					30

/* MT9J003 selected register addresses */
#define MT9J003_MODEL_ID								0x3000
#define MT9J003_RESET				  					0x301A

/* PLL Registers */
#define MT9J003_VT_PIX_CLK_DIV					0x0300
#define MT9J003_VT_SYS_CLK_DIV 					0x0302
#define MT9J003_PRE_PLL_CLK_DIV					0x0304
#define MT9J003_PLL_MULTIPLIER					0X0306
#define MT9J003_OP_PIX_CLK_DIV					0x0308
#define MT9J003_OP_SYS_CLK_DIV					0x030A
#define MT9J003_ROW_SPEED			    			0x3016

/* FOV from array */
#define MT9J003_X_ADDR_START						0x3004
#define MT9J003_X_ADDR_END							0x3008
#define MT9J003_Y_ADDR_START						0x3002
#define MT9J003_Y_ADDR_END							0x3006

/* Binning / Summing */
#define MT9J003_BINNING			 					  	0x3040
#define MT9J003_BINNING_X_ODD_INC(n)			((n) << 6)
#define MT9J003_BINNING_Y_ODD_INC(n)    	((n) << 0)
#define MT9J003_BINNING_XY_BIN_ENABLE(n)	((n) << 10)
#define MT9J003_BINNING_X_BIN_ENABLE(n)   ((n) << 11)
#define MT9J003_BINNING_LOW_POWER_MODE(n) ((n) << 9)
#define MT9J003_BINNING_BIN_SUM(n)				((n) << 12)
#define MT9J003_BINNING_Y_SUM_ENABLE(n)		((n) << 13)

/* Gain */
#define MT9J003_RED_GAIN 0x3056
#define MT9J003_BLUE_GAIN 0x3058
#define MT9J003_GREEN_1_GAIN 0x3056
#define MT9J003_GREEN_2_GAIN 0x305C

/* Scaling and Cropping */
#define MT9J003_SCALING_MODE						0x400
#define MT9J003_M_SCALE				  				0x404
#define MT9J003_X_OUTPUT_SIZE						0x34C
#define MT9J003_Y_OUTPUT_SIZE						0x34E

/* Row Timing */
#define MT9J003_LINE_LENGTH_PCK					0x0342
#define MT9J003_FRAME_LENGTH_LINES			0x0340
#define MT9J003_FINE_CORRECTION					0x3010
#define MT9J003_FINE_INT_TIME						0x3014
#define MT9J003_COURSE_INT_TIME 				0x202
#define MT9J003_EXTRA_DELAY							0x3018

/* Column correction */
#define MT9J003_COLUMN_SAMPLE 					0x30D4

/* Power optimization */
#define MT9J003_LOW_POWER_TIMING				0x3170

/* Debug functions */

static int debug;
module_param(debug, bool, 0644);
MODULE_PARM_DESC(debug, "Debug level (0-1)");

static const struct v4l2_fmtdesc mt9j003_formats[] = {
	{
		.index = 0,
		.type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
		.description = "Bayer (sRGB) 10 bit",
		.pixelformat = V4L2_PIX_FMT_SGRBG10,
	},
};
static const unsigned int mt9j003_num_formats = ARRAY_SIZE(mt9j003_formats);

static const struct v4l2_queryctrl mt9j003_controls[] = {
{
		.id		= V4L2_CID_GAIN,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "Gain",
		.minimum	= 0,
		.maximum	= 127,
		.step		= 1,
		.default_value	= 64,
		.flags		= V4L2_CTRL_FLAG_SLIDER,
	}, {
		.id		= V4L2_CID_EXPOSURE,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "Exposure",
		.minimum	= 1,
		.maximum	= 255,
		.step		= 1,
		.default_value	= 255,
		.flags		= V4L2_CTRL_FLAG_SLIDER,
	}, {
			.id     = V4L2_CID_RED_BALANCE,
			.type       = V4L2_CTRL_TYPE_INTEGER,
			.name       = "Red Balance",
			.minimum    = 1,
			.maximum    = 1024,
			.step       = 1,
			.default_value  = 8
	}, {
			.id     = V4L2_CID_BRIGHTNESS,
			.type       = V4L2_CTRL_TYPE_INTEGER,
			.name       = "Brightness (Green 1 Balance)",
			.minimum    = 1,
			.maximum    = 1024,
			.step       = 1,
			.default_value  = 8
	}, {
			.id     = V4L2_CID_AUTOGAIN,
			.type       = V4L2_CTRL_TYPE_INTEGER,
			.name       = "Auto Gain (Green 2 Balance)",
			.minimum    = 1,
			.maximum    = 1024,
			.step       = 1,
			.default_value  = 8
	}, {
			.id     = V4L2_CID_BLUE_BALANCE,
			.type       = V4L2_CTRL_TYPE_INTEGER,
			.name       = "Blue Balance",
			.minimum    = 1,
			.maximum    = 1024,
			.step       = 1,
			.default_value  = 8
	}
};

/* Device structure */

static const unsigned int mt9j003_num_controls = ARRAY_SIZE(mt9j003_controls);

struct mt9j003 {
	struct v4l2_subdev sd;
	int model;
	u32 x_current;
	u32 y_current;
	u16 xskip;
	u16 yskip;
	u32 xbin;
	u32 ybin;
	u32 width;
	u32 height;
	unsigned short gain;
	u32 exposure;
	unsigned short mirror_column;
	unsigned short mirror_row;
	u32 x_min;
	u32 y_min;
	u32 width_min;
	u32 height_min;
	u32 width_max;
	u32 height_max;
};

static inline struct mt9j003 *to_mt9j003(struct v4l2_subdev *sd)
{
	return container_of(sd, struct mt9j003, sd);
}

/* Register write/read functions anipulation */

/**
 * Writes to Register
 */
static u16 reg_read(struct i2c_client *client, const u16 reg)
{
	u16 _reg = cpu_to_be16(reg);
	u8 buf[2];
	int ret;
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 2,
			.buf = (u8*)&_reg,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD, /* 1 */
			.len = 2,
			.buf = buf,
		},
	};

	ret = i2c_transfer(client->adapter, msgs, 2);

	if (ret < 0)
	{
		v4l_err(client, "I2C Error: Failed to read from 9x%x error %d", reg, ret);;
		return ret;
	}
	return (buf[0] << 8) | buf[1];
}

static int reg_write(struct i2c_client *client, const u16 reg, const u16 data)
{
	int ret;
	u8 buf[4];
	struct i2c_msg msg;

	u16 test;

	u16 _reg = cpu_to_be16(reg);
	u16 _data = cpu_to_be16(data);

	buf[0] = _reg & 0xff;
	buf[1] = _reg >> 8;
	buf[2] = _data & 0xff;
	buf[3] = _data >> 8;

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = 4;
	msg.buf = buf;

	ret = i2c_transfer(client->adapter, &msg, 1);
	if(ret >= 0) {
		printk(KERN_INFO "mj9j003: REGISTER 0x%04X, 0x%04X\n", reg, data);
		return 0;
	}

	v4l_err(client, "I2C Error: Failed to write at 0x%x error %d\n", reg, ret);

	return ret;
}

/* Power functions */

/**
 * Turns power on to the sensor module
 */
static int mt9j003_power_on(struct mt9j003 *mt9j003)
{
	int ret;

	/* Power on */
	if (gpio_is_valid(30)) {
		printk(KERN_INFO "camera: power on");
		gpio_direction_output(30, 1);
		msleep(50);
		ret = 0;
	} else {
		// Failed
		ret = 1;
	}

	return 0;
}

/**
 * Turns power off to the senor module
 */
static void mt9j003_power_off(struct mt9j003 *mt9j003)
{
	int ret;

	/* Power on */
	if (gpio_is_valid(30)) {
		printk(KERN_INFO "camera: power off");
		gpio_direction_output(30, 0);
		msleep(50);
	}
}

/**
 * Sets power on/ff
 */
static int __mt9j003_set_power(struct mt9j003 *mt9j003, bool on)
{
	int ret;

	if (!on) {
		mt9j003_power_off(mt9j003);
		return 0;
	}

	ret = mt9j003_power_on(mt9j003);
	if (ret < 0)
		return ret;

	return 0;
}


/* Register setup */

/**
 * Sets up the plls on the sensor.<
 * Array Clock: 160 MHz
 * Output Clock: 80 MHz
*/
static int mt9j003_setpll(struct i2c_client *client)
{
	reg_write(client, MT9J003_VT_PIX_CLK_DIV, 2);
	reg_write(client, MT9J003_VT_SYS_CLK_DIV, 2);

	reg_write(client, MT9J003_PRE_PLL_CLK_DIV, 1);
	reg_write(client, MT9J003_PLL_MULTIPLIER, 32);

	reg_write(client, MT9J003_OP_PIX_CLK_DIV, 8);
	reg_write(client, MT9J003_OP_SYS_CLK_DIV, 1);

	reg_write(client, MT9J003_ROW_SPEED,
		(1 << 0) | (1 << 8));

	return 1;
}

/**
 * Sets defaults values for sensor as suggested in data sheet
 */
static int mt9j003_set_default_regs(struct i2c_client *client)
{
	// Recommend default register settings
	reg_write(client, 0x316C,0x0429);
  reg_write(client, 0x3174,0x8000);
  reg_write(client, 0x3E40,0xDC05);
  reg_write(client, 0x3E42,0x6E22);
  reg_write(client, 0x3E44,0xDC22);
  reg_write(client, 0x3E46,0xFF00);
  reg_write(client, 0x3ED4,0xF998);
  reg_write(client, 0x3ED6,0x9789);
  reg_write(client, 0x3EDE,0xE41A);
  reg_write(client, 0x3EE0,0xA43F);
  reg_write(client, 0x3EE2,0xA4BF);
  reg_write(client, 0x3EEC,0x1C21);
}

/* V4L2 Sub-device operations */

 static int mt9j003_init(struct v4l2_subdev *sd, u32 val)
{
	printk(KERN_INFO "MT9J003: INIT");

	int ret;
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	mt9j003_power_on(sd);

	return 0;
}

static int mt9j003_set_params(struct v4l2_subdev *sd,
	struct v4l2_rect *rect, u16 xskip, u16 yskip)
{
	struct mt9j003 *mt9j003 = to_mt9j003(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	u16 reg;
	u16 xbin=1, ybin=1, width, height, left, top;

	width = rect->width * xskip;
	height = rect->height * yskip;
	left = rect->left;
	top = rect->top;

	if(xskip & 1)
			xbin = 1;
	else if (xskip & 2)
			xbin = 2;
	else if (xskip & 4)
			xbin = 4;

	if(yskip & 1)
			ybin = 1;
	else if (yskip & 2)
			ybin = 2;
	else if (yskip & 4)
			ybin = 4;

  left = ((left) & ~(2 * xbin -1)) + 1 * xbin * mt9j003->mirror_row;
  top = ((top) & ~(2 * ybin -1)) + 1 * ybin * mt9j003->mirror_column;

	mt9j003->width  = rect->width,
	mt9j003->height = height,
	mt9j003->x_current = rect->left;
	mt9j003->y_current = rect->top;
	mt9j003->ybin = ybin;
	mt9j003->xbin = xbin;
	mt9j003->xskip = xskip;
	mt9j003->yskip = yskip;

	reg_write(client, 0xEDE, 0xE412);

	// Unset video registers
	reg = reg_read(client, 0x3EDC);
	reg |= (1 << 7);
	reg_write(client, 0x3EDC, reg | (1 << 7));

	reg = reg_read(client, 3178);
	reg &= ~0x030;
	reg &= ~0x0C0;
	reg_write(client, 0x3178, reg);

  // Disable sampling
  reg_write(client, 0x400, 0x02);
	reg_write(client, 0x306E, 0x9080);

	// FOV of array
	reg_write(client, MT9J003_X_ADDR_START, left);
	reg_write(client, MT9J003_X_ADDR_END, mt9j003->width);
	reg_write(client, MT9J003_Y_ADDR_START, top);
	reg_write(client, MT9J003_Y_ADDR_END, mt9j003->height);

	// Power optimization
	reg_write(client, MT9J003_LOW_POWER_TIMING, 0x0071);

	// Bining and summing
	reg_write(client, MT9J003_BINNING,
		MT9J003_BINNING_X_ODD_INC(xskip) |
	  MT9J003_BINNING_Y_ODD_INC(yskip) |
		MT9J003_BINNING_BIN_SUM(0) |
		MT9J003_BINNING_Y_SUM_ENABLE(ybin) |
		MT9J003_BINNING_X_BIN_ENABLE(xbin) |
		MT9J003_BINNING_XY_BIN_ENABLE(0) |
		MT9J003_BINNING_LOW_POWER_MODE(1));

	// Scaling and cropping
	reg_write(client, MT9J003_SCALING_MODE, 0);
	reg_write(client, MT9J003_M_SCALE, 16);
	reg_write(client, MT9J003_X_OUTPUT_SIZE, mt9j003->width);
	reg_write(client, MT9J003_Y_OUTPUT_SIZE, mt9j003->height);

	// Row timing
	reg_write(client, MT9J003_LINE_LENGTH_PCK, 5000);
	reg_write(client, MT9J003_FRAME_LENGTH_LINES, 3000);

	// TODO: adjust integratin time
	reg_write(client, MT9J003_COURSE_INT_TIME, 0x44c);
	reg_write(client, MT9J003_FINE_CORRECTION, 72);
	reg_write(client, MT9J003_FINE_INT_TIME, 522);

	reg_write(client, MT9J003_EXTRA_DELAY, 0);

	// Column correction
	reg_write(client, 0x30d4, 0x9080);

	// Restart frame
	reg_write(client, 0x301A, (1 << 1));

	return 0;
}

static int mt9j003_s_stream(struct v4l2_subdev *sd, int enable)
{
	printk(KERN_INFO "MT9J003: STREAM");

	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct mt9j003 *mt9j003 = to_mt9j003(sd);
	int ret;

	if(enable) {
		reg_write(client, 0x301A, 1);
		msleep(300);

		// Enable streaming / Reset Register
		reg_write(client, 0x301A, 0x10DC);

		// Set pll
		mt9j003_setpll(client);

		// See default configuration file
		reg_write(client, 0x3ecc, 0x0fe4);
		reg_write(client, 0x31AE, 0x0301);



		// Set default register settings
		mt9j003_set_default_regs(client);

		//***Change the data pedestal and SMIA encoded data
		reg_write(client, 0x301A, 0x0010);	// Reserved
		reg_write(client, 0x3064, 0x0805);	// Reserved
		reg_write(client, 0x301E, 0x00A8);	// Reserved
    //reg_write(client, 0x3064, 0 << 8); //Disable c Encoding



		// Enable parallel streaming
		reg_write(client, MT9J003_RESET, 0x10DC);
	}
	else
	{
		reg_write(client, MT9J003_RESET, 0x0018);
	}

	return 0;
}

const struct v4l2_queryctrl *mt9j003_find_qctrl(u32 id)
{
	int i;

	for (i = 0; i < mt9j003_num_controls; i++) {
		if (mt9j003_controls[i].id == id)
			return &mt9j003_controls[i];
	}
	return NULL;
}

static void recalculate_limits(struct mt9j003 *mt9j003,
                   u16 xskip, u16 yskip)
{
    mt9j003->x_min = (2 * xskip - 1) & ~(2 * xskip - 1);
    mt9j003->y_min = (2 * yskip - 1) & ~(2 * yskip - 1);
    mt9j003->width_min = (MT9J003_MIN_WIDTH + 2 * xskip - 1) & ~(2 * xskip - 1);
    mt9j003->height_min = (MT9J003_MIN_HEIGHT + 2 * yskip - 1) & ~(2 * yskip - 1);
    mt9j003->width_max = MT9J003_MAX_WIDTH / xskip;
    mt9j003->height_max = MT9J003_MAX_HEIGHT / yskip;
}


static int mt9j003_set_fmt(struct v4l2_subdev *sd,
			   struct v4l2_format *f)
{
	struct mt9j003 *mt9j003 = to_mt9j003(sd);
	int ret;
	u16 xskip, yskip;
	struct v4l2_rect rect = {
		.left	= mt9j003->x_current,
		.top	= mt9j003->y_current,
		.width	= f->fmt.pix.width,
		.height	= f->fmt.pix.height,
	};

	for (xskip = 8; xskip > 1; xskip--)
			if (rect.width * xskip <= MT9J003_MAX_WIDTH)
					break;

	for (yskip = 8; yskip > 1; yskip--)
			if (rect.height * yskip <= MT9J003_MAX_HEIGHT)
					break;

	if (xskip != yskip){
		if (xskip > yskip)
			xskip = yskip;
		else
		yskip = xskip;
	}

	recalculate_limits(mt9j003, xskip, yskip);
	ret = mt9j003_set_params(sd, &rect, xskip, yskip);

	return ret;
}

static int mt9j003_try_fmt(struct v4l2_subdev *sd,
			   struct v4l2_format *f)
{
	struct v4l2_pix_format *pix = &f->fmt.pix;

	if (pix->height < MT9J003_MIN_HEIGHT)
			pix->height = MT9J003_MIN_HEIGHT;
	if (pix->height > MT9J003_MAX_HEIGHT)
			pix->height = MT9J003_MAX_HEIGHT;
	if (pix->width < MT9J003_MIN_WIDTH)
			pix->width = MT9J003_MIN_WIDTH;
	if (pix->width > MT9J003_MAX_WIDTH)
			pix->width = MT9J003_MAX_WIDTH;

	pix->width &= ~0x01; /* has to be even */
	pix->height &= ~0x01; /* has to be even */
	return 0;
	return 0;
}

static int mt9j003_queryctrl(struct v4l2_subdev *sd,
                struct v4l2_queryctrl *qctrl)
{
    const struct v4l2_queryctrl *temp_qctrl;

    temp_qctrl = mt9j003_find_qctrl(qctrl->id);
    if (!temp_qctrl) {
        v4l2_dbg(1, debug,sd, "control id %d not supported", qctrl->id);
        return -EINVAL;
    }
    memcpy(qctrl, temp_qctrl, sizeof(*qctrl));
    return 0;
}


static int mt9j003_get_chip_id(struct v4l2_subdev *sd,
			       struct v4l2_dbg_chip_ident *id)
{
	struct mt9j003 *mt9j003 = to_mt9j003(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);;

	if (id->match.type != V4L2_CHIP_MATCH_I2C_ADDR)
		return -EINVAL;

	if (id->match.addr != client->addr)
		return -ENODEV;

	id->ident	= mt9j003->model;
	id->revision	= 0;

	return 0;
}


static int mt9j003_get_control(struct v4l2_subdev *, struct v4l2_control *);
static int mt9j003_set_control(struct v4l2_subdev *, struct v4l2_control *);

static const struct v4l2_subdev_core_ops mt9j003_core_ops = {
	.g_chip_ident = mt9j003_get_chip_id,
	.init = mt9j003_init,
	.queryctrl = mt9j003_queryctrl,
	.g_ctrl	= mt9j003_get_control,
	.s_ctrl	= mt9j003_set_control,
};

static const struct v4l2_subdev_video_ops mt9j003_video_ops = {
	.s_fmt = mt9j003_set_fmt,
	.try_fmt = mt9j003_try_fmt,
	.s_stream = mt9j003_s_stream,
};

static const struct v4l2_subdev_ops mt9j003_ops = {
	.core = &mt9j003_core_ops,
	.video = &mt9j003_video_ops,
};

static int mt9j003_get_control(struct v4l2_subdev *sd,
			       struct v4l2_control *ctrl)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct mt9j003 *mt9j003 = to_mt9j003(sd);
	int data;
	int reg = -1;
	bool gain = false;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
			ctrl->value = mt9j003->exposure;
			break;

	case V4L2_CID_RED_BALANCE:
			reg = MT9J003_RED_GAIN;
			gain = true;
			break;
	case V4L2_CID_BLUE_BALANCE:
			reg = MT9J003_BLUE_GAIN;
			gain = true;
			break;
	case V4L2_CID_BRIGHTNESS:
			reg = MT9J003_GREEN_1_GAIN;
			gain = true;
			break;
	case V4L2_CID_AUTOGAIN:
			reg = MT9J003_GREEN_2_GAIN;
			gain = true;
			break;
	}
	if (gain){
			data = reg_read(client, reg);
			if ( (data & 0x7f40)  == 0 )
					ctrl->value = data;
			else if ((data & 0x7f00) == 0)
					ctrl->value = ((data & 0x003f) << 1);
			else
					ctrl->value = ((data & 0xff00)>>5) + 64;
	}
	return 0;
}

static int mt9j003_set_control(struct v4l2_subdev *sd,
			       struct v4l2_control *ctrl)
{
	struct mt9j003 *mt9j003 = to_mt9j003(sd);
	const struct v4l2_queryctrl *qctrl = NULL;
	int data;
	int ret;
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	if (NULL == ctrl)
		return -EINVAL;

	switch(ctrl->id) {

		case V4L2_CID_EXPOSURE:
        if (ctrl->value > qctrl->maximum ||
            ctrl->value < qctrl->minimum){
            printk("Exposure range is exceed:%d\n",(int)ctrl->value);
            return -EINVAL;
        } else {
						// TODO: add implementation and test with new cameras
        }
        break;
		case V4L2_CID_RED_BALANCE:
    case V4L2_CID_BLUE_BALANCE:
    case V4L2_CID_BRIGHTNESS:
    case V4L2_CID_AUTOGAIN:
        /* This control will be used to modify Gain */
        if (ctrl->value > qctrl->maximum || ctrl->value < qctrl->minimum){
            printk("I receive a value that exceeds the range:%d\n",(int)ctrl->value);
            return -EINVAL;
        }
				// TODO: adjust
        data = ctrl->value;
        v4l2_dbg(1, debug, sd, "Setting gain %d\n", data);
        switch (ctrl->id) {
            case V4L2_CID_RED_BALANCE:
                ret = reg_write(client, MT9J003_RED_GAIN, data);
                if (ret < 0){
                    printk("Fail setting Red Gain register\n");
                    return -EIO;
                }
                break;
            case V4L2_CID_BLUE_BALANCE:
                ret = reg_write(client, MT9J003_BLUE_GAIN, data);
                if (ret < 0){
                    printk("Fail setting Blue Gain register\n");
                    return -EIO;
                }
                break;
            case V4L2_CID_BRIGHTNESS:
                ret = reg_write(client, MT9J003_GREEN_1_GAIN, data);
                if (ret < 0){
                    printk("Fail setting Green1 Gain register\n");
                    return -EIO;
                }
                break;
            case V4L2_CID_AUTOGAIN:
                ret = reg_write(client, MT9J003_GREEN_2_GAIN, data);
                if (ret < 0){
                    printk("Fail setting Green2 Gain register\n");
                    return -EIO;
                }
                break;
        }
        break;
	};

	return 0;
}

/* V4L2 subdev internal operations */

/**
 * Powers on the sensor and checks model id
 */
static int mt9j003_detect(struct i2c_client *client)
{
	s32 data;
	int ret;
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct mt9j003 *mt9j003 = to_mt9j003(sd);
	int count = 0;

	ret = mt9j003_power_on(mt9j003);

	if (ret < 0) {
		dev_err(&client->dev, "MT9J003 power up failed\n");
		return ret;
	}

	data = reg_read(client, MT9J003_MODEL_ID);

	if (data != MT9J003_CHIP_VERSION_VALUE) {
		while(count ++ < 5) {
			data = reg_read(client, MT9J003_MODEL_ID);
			msleep(5);
		}
		dev_err(&client->dev, "MT9J003 not detected, wrong version "
			"0x%04x\n", data);
		return -ENODEV;
	}

	mt9j003_power_off(mt9j003);

	return 0;
}

/**
 * Probes for the sensor
 */
static int mt9j003_probe(struct i2c_client *client,
			 const struct i2c_device_id *did)
{
	printk(KERN_ERR "MT9J003: PROBE");

	struct mt9j003 *mt9j003;
	struct v4l2_subdev *sd;
	int ret;

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_WORD_DATA)) {
		dev_warn(&client->dev,
			 "I2C-Adapter doesn't support I2C_FUNC_SMBUS_WORD\n");
		return -EIO;
	}

	if (!client->dev.platform_data) {
		dev_err(&client->dev, "No platform data!!\n");
		return -ENODEV;
	}

	mt9j003 = kzalloc(sizeof(struct mt9j003), GFP_KERNEL);
	if (!mt9j003)
		return -ENOMEM;

	ret = mt9j003_detect(client);

	if (ret)
		goto clean;

	// Setup device
	mt9j003->x_min      = 0;
	mt9j003->y_min      = 0;
	mt9j003->width      = MT9J003_DEFAULT_WIDTH;
	mt9j003->height     = MT9J003_DEFAULT_HEIGHT;

	mt9j003->width_min  = MT9J003_MIN_WIDTH;
	mt9j003->width_max  = MT9J003_MAX_WIDTH;
	mt9j003->height_min = MT9J003_MIN_HEIGHT;
	mt9j003->height_max = MT9J003_MAX_HEIGHT;

	mt9j003->xskip = 1;
	mt9j003->yskip = 1;
	mt9j003->xbin = 1;
	mt9j003->ybin = 1;
	mt9j003->mirror_column = 0;
	mt9j003->mirror_row = 0;


	/* Register with V4L2 layer as slave device */
	sd = &mt9j003->sd;
	v4l2_i2c_subdev_init(sd, client, &mt9j003_ops);

	v4l2_info(sd, "%s decoder driver registered !!\n", sd->name);
	return 0;

clean:
	kfree(mt9j003);
	return ret;
}

static int mt9j003_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct mt9j003 *mt9j003 = to_mt9j003(sd);

	v4l2_device_unregister_subdev(sd);

	kfree(mt9j003);
	return 0;
}

static const struct i2c_device_id mt9j003_id[] = {
	{ "mt9j003", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, mt9j003_id);

static struct i2c_driver mt9j003_i2c_driver = {
	.driver = {
		.name = "mt9j003",
	},
	.probe		= mt9j003_probe,
	.remove		= mt9j003_remove,
	.id_table	= mt9j003_id,
};

static int __init mt9j003_mod_init(void)
{
	return i2c_add_driver(&mt9j003_i2c_driver);
}

static void __exit mt9j003_mod_exit(void)
{
	i2c_del_driver(&mt9j003_i2c_driver);
}

module_init(mt9j003_mod_init);
module_exit(mt9j003_mod_exit);

MODULE_DESCRIPTION("Aptina MT9J003 Camera driver");
MODULE_AUTHOR("Dimitri Emmerich <dimitri.emmerich@gmail.com");
MODULE_LICENSE("GPL v2");
