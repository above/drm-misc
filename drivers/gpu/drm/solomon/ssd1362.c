// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2021, 2022 Above Agency AB
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/dma-buf.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/property.h>
#include <linux/spi/spi.h>
#include <linux/irq.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_drv.h>
#include <drm/drm_gem.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_gem_shmem_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_gem_atomic_helper.h>
#include <drm/drm_simple_kms_helper.h>
#include <drm/drm_damage_helper.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_vblank.h>

#define DISPLAY_WIDTH 256 // Default, overridden from DT entry if present
#define DISPLAY_HEIGHT 64
#define DEFAULT_CONTRAST 0x40

#define HIGH 1
#define LOW 0
#define DC_DATA_MODE 1
#define DC_COMMAND_MODE 0

struct ssd1362 {
	struct drm_device drm;
	struct drm_simple_display_pipe pipe;
	struct drm_connector connector;
	struct drm_display_mode mode;
	struct drm_property *contrast_prop;
	struct gpio_desc *reset;
	struct gpio_desc *boost;
	struct gpio_desc *dc;
	struct gpio_desc *fr;
	struct spi_device *spi;
	uint32_t width;
	uint32_t height;
	uint32_t rotation;
	bool enabled;
	bool display_is_on;
	uint8_t contrast;
	uint8_t contrast_active;
	int dc_data_mode;
	uint8_t *tx_buf;
};

static const uint8_t ssd1362_lut[16] = {
	0xB8, // Set LUT command
	// gamma=0.75 min(ceilf(pow((i=0..15 + 1) / 15.0, gamma)), 255)
	34, 57, 77, 96, 113, 129, 145, 160, 175, 189, 203, 217, 230, 243, 255
};

static void set_dc_mode(struct ssd1362 *s, int mode)
{
	if (s->dc_data_mode != mode) {
		gpiod_set_value(s->dc, mode);
		s->dc_data_mode = mode;
	}
}

static int write_regs(struct ssd1362 *s, const uint8_t *regs, size_t nregs)
{
	set_dc_mode(s, DC_COMMAND_MODE);
	return spi_write(s->spi, regs, nregs);
}

static int write_data(struct ssd1362 *s, const uint8_t *data, size_t len)
{
	size_t max_chunk;
	struct spi_transfer tr = {
		.bits_per_word = 8,
	};
	struct spi_message m = {};
	size_t chunk;
	int ret;

	set_dc_mode(s, DC_DATA_MODE);

	max_chunk = spi_max_transfer_size(s->spi);
	if (max_chunk == -1)
		max_chunk = len;
	spi_message_init_with_transfers(&m, &tr, 1);

	while (len) {
		chunk = min(len, max_chunk);

		tr.tx_buf = data;
		tr.len = chunk;
		data += chunk;
		len -= chunk;

		ret = spi_sync(s->spi, &m);
		if (ret)
			return ret;
	}

	return 0;
}

// Convert MSB of G8 data to G4
static void convert_block(struct ssd1362 *s, const uint8_t *src,
	int xs, int xe, int ys, int ye)
{
	int w = xe - xs;
	int dst_w = w >> 1;
	int h = ye - ys;
	int src_pad = s->width - w;
	int x, y;
	uint8_t *g4 = s->tx_buf;

	src += ys * w + xs;
	for (y = 0; y < h; y++) {
		for (x = 0; x < dst_w; x++) {
			uint8_t s1 = *src++;
			uint8_t s2 = *src++;
			*g4++ = (s1 & 0xf0) | ((s2 >> 4) & 0xf);
		}
		src += src_pad;
	}
}

static irqreturn_t ssd1362_irq_handler(int irq, void *arg)
{
	struct drm_device *drm = arg;
	struct ssd1362 *s = drm->dev_private;

	if (gpiod_get_value(s->fr)) {
		drm_handle_vblank(drm, 0);
		return IRQ_HANDLED;
	}

	return IRQ_NONE;
}

static int ssd1362_enable_vblank(struct drm_simple_display_pipe *pipe)
{
	struct drm_crtc *crtc = &pipe->crtc;
	struct drm_device *drm = crtc->dev;
	struct ssd1362 *s = drm->dev_private;

	if (s->fr)
		enable_irq(gpiod_to_irq(s->fr));

	return 0;
}

static void ssd1362_disable_vblank(struct drm_simple_display_pipe *pipe)
{
	struct drm_crtc *crtc = &pipe->crtc;
	struct drm_device *drm = crtc->dev;
	struct ssd1362 *s = drm->dev_private;

	if (s->fr)
		disable_irq(gpiod_to_irq(s->fr));
}

static int init_display(struct ssd1362 *s)
{
	int ret = 0;

	struct gpio_desc *reset = s->reset;

	gpiod_set_value(reset, HIGH);
	udelay(100);
	gpiod_set_value(reset, LOW);
	udelay(100);
	gpiod_set_value(reset, HIGH);
	usleep_range(1000, 2000);

	write_regs(s, (uint8_t []){ 0xAF, 0xA6 }, 2); // Display on, Set display all pixels OFF
	msleep(250); // Let it stay on for a bit

	ret = write_regs(s, (uint8_t []){
		0xAE, // Display off

		0x81, // Set Contrast Current
		s->contrast,

		0xAB, // Function selection
			  // 0x00 - Disable Internal VDD (Vci 1.65V - 2.6V)
			  // 0x01 - Enable Internal VDD (Vci 2.6V - 3.5V)
		0x01,

		0xAD, // Select external or internal IREF
		// 0x9E - Enable internal Iref during display on
		// 0x8E - Selext external Iref
		0x9E,

		0xA1, // Set vertical display start line
		0x00,

		0xA2, // Set vertical display offset
		0x00,

		0xA8, // Set multiplex ratio
		0x3F, // Default value

		0xB1, // Set phase length
		0x81,

		0xB3, // Display Clock Divider
		0xF0, // Using a default value

		0xBC, // Set pre-charge voltage level
		0x04, // Using a default value

		0xBE, // Set voltage vcomh
		0x05, // Reset, 0x82 x Vcc

		0xB6, // Second precharge period
		0x0F,

		// Set orientation from 'rotate' property in DT
		0xA0,
		s->rotation == 180 ? 0x41 : 0x52,
	}, 25);
	s->contrast_active = s->contrast;

	ret = write_regs(s, ssd1362_lut, 16);

	return ret;
}

static void display_on(struct ssd1362 *s)
{
	if (s->display_is_on)
		return;
	s->display_is_on = true;

	if (s->boost) {
		gpiod_set_value(s->boost, HIGH);
		mdelay(1);
	}
	write_regs(s, (uint8_t []){ 0xAF, 0xA4 }, 2); // Display on, Set display normal

	msleep(250);
}

static void set_window(struct ssd1362 *s, struct drm_rect *rect)
{
	write_regs(s, (uint8_t []){
		0x15, // Set column start, end address
		rect->x1,
		rect->x2 - 1,
		0x75, // Set row start, end address
		rect->y1,
		rect->y2 - 1
	}, 6);
}

static void update(struct ssd1362 *s, struct drm_plane_state *state, struct drm_rect *rect)
{
	struct drm_framebuffer *fb = state->fb;
	struct iosys_map map[DRM_FORMAT_MAX_PLANES];
	struct iosys_map map_data[DRM_FORMAT_MAX_PLANES];
	void *vaddr;
	size_t len;
	int ret;

	// round start/end to even pair of G4 nibbles
	rect->x1 = rect->x1 & ~1;
	rect->x2 = (rect->x2 + 1) & ~1;

	set_window(s, rect);

	ret = drm_gem_fb_vmap(fb, map, map_data);
	if (ret) {
		dev_err_once(s->drm.dev, "drm_gem_fb_vmap");
		goto out;
	}

	vaddr = map_data[0].vaddr;

	ret = drm_gem_fb_begin_cpu_access(fb, DMA_FROM_DEVICE);
	if (ret) {
		dev_err_once(s->drm.dev, "drm_gem_fb_begin_cpu_access\n");
		goto vunmap;
	}

	convert_block(s, vaddr, rect->x1, rect->x2, rect->y1, rect->y2);

	drm_gem_fb_end_cpu_access(fb, DMA_FROM_DEVICE);
vunmap:
	drm_gem_fb_vunmap(fb, map);

	len = (rect->x2 - rect->x1) * (rect->y2 - rect->y1);
	ret = write_data(s, s->tx_buf, len/2);
	if (ret)
		dev_err_ratelimited(s->drm.dev, "spi error %d", ret);
out:
	;
}

static void ssd1362_pipe_enable(struct drm_simple_display_pipe *pipe,
	struct drm_crtc_state *crtc_state,
	struct drm_plane_state *plane_state)
{
	struct drm_crtc *crtc = &pipe->crtc;
	struct drm_device *drm = crtc->dev;
	struct ssd1362 *s = drm->dev_private;
	int idx;

	if (s->enabled)
		return;

	if (!drm_dev_enter(pipe->crtc.dev, &idx))
		return;

	s->enabled = true;

	init_display(s);

	s->display_is_on = false;

	drm_crtc_vblank_on(crtc);

	drm_dev_exit(idx);
}

static void ssd1362_pipe_disable(struct drm_simple_display_pipe *pipe)
{
	struct drm_crtc *crtc = &pipe->crtc;
	struct drm_device *drm = crtc->dev;
	struct ssd1362 *s = drm->dev_private;

	if (!s->enabled)
		return;

	s->enabled = false;

	drm_crtc_vblank_off(crtc);

	write_regs(s, (uint8_t []){ 0xA6, 0xAE }, 2); // Set display all pixels OFF, Display off
	if (s->boost) {
		mdelay(1);
		gpiod_set_value(s->boost, LOW);
	}

	s->display_is_on = false;
}

static void ssd1362_pipe_update(struct drm_simple_display_pipe *pipe,
	struct drm_plane_state *old_state)
{
	struct drm_crtc *crtc = &pipe->crtc;
	struct drm_device *drm = crtc->dev;
	struct ssd1362 *s = drm->dev_private;
	struct drm_plane_state *state = pipe->plane.state;
	struct drm_rect rect;
	int idx;

	if (!drm_dev_enter(pipe->crtc.dev, &idx))
		return;

	if (s->enabled && drm_atomic_helper_damage_merged(old_state, state, &rect)) {
		display_on(s);

		if (s->contrast_active != s->contrast) {
			s->contrast_active = s->contrast;
			write_regs(s, (uint8_t []){ 0x81, s->contrast }, 2); // Set Contrast Current
		}

		update(s, state, &rect);
	}

	drm_dev_exit(idx);

	if (crtc->state->event) {
		spin_lock_irq(&crtc->dev->event_lock);
		if (s->fr && crtc->state->active && drm_crtc_vblank_get(crtc) == 0)
			drm_crtc_arm_vblank_event(crtc, crtc->state->event);
		else
			drm_crtc_send_vblank_event(crtc, crtc->state->event);
		crtc->state->event = NULL;
		spin_unlock_irq(&crtc->dev->event_lock);
	}
}

static int ssd1362_get_property(struct drm_connector *connector,
				const struct drm_connector_state *state,
				struct drm_property *property,
				uint64_t *val)
{
	struct drm_device *drm = connector->dev;
	struct ssd1362 *s = drm->dev_private;

	if (property == s->contrast_prop) {
		*val = s->contrast;
		return 0;
	}

	return -EINVAL;
}

static int ssd1362_set_property(struct drm_connector *connector,
				struct drm_connector_state *state,
				struct drm_property *property,
				uint64_t val)
{
	struct drm_device *drm = connector->dev;
	struct ssd1362 *s = drm->dev_private;

	if (property == s->contrast_prop) {
		s->contrast = val;

		return 0;
	}

	return -EINVAL;
}

static int ssd1362_connector_get_modes(struct drm_connector *connector)
{
	struct drm_device *drm = connector->dev;
	struct ssd1362 *s = drm->dev_private;
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(connector->dev, &s->mode);
	if (!mode)
		return 0;

	if (mode->name[0] == '\0')
		drm_mode_set_name(mode);

	mode->type |= DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(connector, mode);

	if (mode->width_mm) {
		connector->display_info.width_mm = mode->width_mm;
		connector->display_info.height_mm = mode->height_mm;
	}

	return 1;
}

static const struct drm_connector_helper_funcs ssd1362_connector_hfuncs = {
	.get_modes = ssd1362_connector_get_modes,
};

static const struct drm_connector_funcs ssd1362_connector_funcs = {
	.reset = drm_atomic_helper_connector_reset,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = drm_connector_cleanup,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
	.atomic_get_property = ssd1362_get_property,
	.atomic_set_property = ssd1362_set_property,
};

static const struct drm_simple_display_pipe_funcs ssd1362_pipe_funcs = {
	.enable     = ssd1362_pipe_enable,
	.disable    = ssd1362_pipe_disable,
	.update     = ssd1362_pipe_update,
	.enable_vblank = ssd1362_enable_vblank,
	.disable_vblank = ssd1362_disable_vblank,
};

static const struct drm_mode_config_funcs ssd1362_mode_config_funcs = {
	.fb_create = drm_gem_fb_create_with_dirty,
	.atomic_check = drm_atomic_helper_check,
	.atomic_commit = drm_atomic_helper_commit,
};

DEFINE_DRM_GEM_FOPS(ssd1362_fops);

static void ssd1362_release(struct drm_device *drm)
{
	struct ssd1362 *s = drm->dev_private;

	drm_mode_config_cleanup(drm);
	kfree(s);
}

static struct drm_driver ssd1362_driver = {
	.driver_features    = DRIVER_GEM | DRIVER_MODESET | DRIVER_ATOMIC,
	.fops           = &ssd1362_fops,
	.release        = ssd1362_release,
	DRM_GEM_SHMEM_DRIVER_OPS,
	.name           = "ssd1362",
	.desc           = "Solomon SSD1362",
	.date           = "20210910",
	.major          = 1,
	.minor          = 0,
};

static const struct of_device_id ssd1362_of_match[] = {
	{ .compatible = "futaba,elw2106aa" },
	{ },
};
MODULE_DEVICE_TABLE(of, ssd1362_of_match);

static const struct spi_device_id ssd1362_id[] = {
	{ "ssd1362", 0 },
	{ },
};
MODULE_DEVICE_TABLE(spi, ssd1362_id);

static int ssd1362_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct ssd1362 *s;
	struct drm_device *drm;
	int ret;
	int irq;
	static const uint32_t format = DRM_FORMAT_R8;
	static const uint64_t modifiers[] = {
		DRM_FORMAT_MOD_LINEAR,
		DRM_FORMAT_MOD_INVALID
	};

	if (!dev->coherent_dma_mask) {
		ret = dma_coerce_mask_and_coherent(dev, DMA_BIT_MASK(32));
		if (ret)
			dev_warn(dev, "Failed to set dma mask %d\n", ret);
	}

	s = devm_drm_dev_alloc(dev, &ssd1362_driver, struct ssd1362, drm);
	if (IS_ERR(s)) {
		return PTR_ERR(s);
	}

	drm = &s->drm;
	s->spi = spi;
	s->contrast = DEFAULT_CONTRAST;
	s->contrast_active = -1;
	s->dc_data_mode = -1;

	drm->dev_private = s;

	s->reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(s->reset)) {
		dev_err(dev, "Failed to get gpio 'reset'\n");
		return PTR_ERR(s->reset);
	}

	s->boost = devm_gpiod_get(dev, "boost", GPIOD_OUT_LOW);
	if (IS_ERR(s->boost)) {
		dev_info(dev, "DT property 'boost' not set, assuming fixed supply rail\n");
		/* Not fatal, keep going */
	}

	s->dc = devm_gpiod_get(dev, "dc", GPIOD_OUT_LOW);
	if (IS_ERR(s->dc)) {
		dev_err(dev, "Failed to get gpio 'dc'\n");
		return PTR_ERR(s->dc);
	}

	s->fr = devm_gpiod_get(dev, "fr", GPIOD_IN);
	if (IS_ERR(s->fr)) {
		dev_info(dev, "DT property 'fr' not set, vblank support disabled\n");
		/* Not fatal, keep going */
	}

	s->width = DISPLAY_WIDTH;
	s->height = DISPLAY_HEIGHT;
	s->rotation = 0;

	ret = of_property_read_u32(dev->of_node, "width", &s->width);
	if (ret) {
		dev_info(dev, "DT property 'width' not set, using default %d\n", DISPLAY_WIDTH);
		/* Not fatal, keep going */
	}

	ret = of_property_read_u32(dev->of_node, "height", &s->height);
	if (ret) {
		dev_info(dev, "DT property 'height' not set, using default %d\n", DISPLAY_HEIGHT);
		/* Not fatal, keep going */
	}

	ret = of_property_read_u32(dev->of_node, "rotate", &s->rotation);
	if (ret) {
		dev_info(dev, "DT property 'rotate' not set, using default 0\n");
		/* Not fatal, keep going */
	}

	s->tx_buf = devm_kmalloc(drm->dev, s->width * s->height / 2, GFP_KERNEL);
	if (!s->tx_buf) {
		kfree(s);
		return -ENOMEM;
	}

	drm_mode_config_init(drm);
	drm->mode_config.funcs = &ssd1362_mode_config_funcs;
	drm->mode_config.min_width = s->width;
	drm->mode_config.max_width = s->width;
	drm->mode_config.min_height = s->height;
	drm->mode_config.max_height = s->height;
	drm->mode_config.preferred_depth = 8;

	{
		struct drm_display_mode ssd1362_mode = {
			DRM_SIMPLE_MODE(s->width, s->height, 48, 12),
		};
		drm_mode_copy(&s->mode, &ssd1362_mode);
	}
	drm_connector_helper_add(&s->connector, &ssd1362_connector_hfuncs);
	ret = drm_connector_init(drm, &s->connector, &ssd1362_connector_funcs,
		DRM_MODE_CONNECTOR_SPI);
	if (ret)
		return ret;

	s->contrast_prop = drm_property_create_range(drm, 0, "contrast", 0x00, 0xFF);
	if (s->contrast_prop)
		drm_object_attach_property(&s->connector.base, s->contrast_prop, s->contrast);

	ret = drm_simple_display_pipe_init(drm, &s->pipe, &ssd1362_pipe_funcs, &format, 1,
		modifiers, &s->connector);
	if (ret)
		return ret;

	drm_plane_enable_fb_damage_clips(&s->pipe.plane);

	drm_mode_config_reset(drm);

	spi_set_drvdata(spi, drm);

	ret = drm_dev_register(drm, 0);
	if (ret)
		return ret;

	drm_fbdev_generic_setup(drm, 0);

	if (s->fr) {
		drm_vblank_init(drm, 1);
		irq = gpiod_to_irq(s->fr);
		if (irq > 0) {
			ret = devm_request_irq(dev, irq, ssd1362_irq_handler, IRQF_TRIGGER_RISING,
				ssd1362_driver.name, drm);
			if (ret) {
				dev_err(dev, "Failed to register irq");
				return ret;
			}

			disable_irq(irq);
		} else {
			dev_err(dev, "Failed to get irq number: %d\n", irq);
		}
	}

	dev_info(dev, "ssd1362 probe succes\n");

	return 0;
}

static void ssd1362_remove(struct spi_device *spi)
{
	struct drm_device *drm = spi_get_drvdata(spi);

	drm_dev_unplug(drm);
	drm_atomic_helper_shutdown(drm);
}

static void ssd1362_shutdown(struct spi_device *spi)
{
	drm_atomic_helper_shutdown(spi_get_drvdata(spi));
}

static int __maybe_unused ssd1362_pm_suspend(struct device *dev)
{
	return drm_mode_config_helper_suspend(dev_get_drvdata(dev));
}

static int __maybe_unused ssd1362_pm_resume(struct device *dev)
{
	drm_mode_config_helper_resume(dev_get_drvdata(dev));

	return 0;
}

static const struct dev_pm_ops ssd1362_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(ssd1362_pm_suspend, ssd1362_pm_resume)
};

static struct spi_driver ssd1362_spi_driver = {
	.driver = {
		.name = "ssd1362",
		.owner = THIS_MODULE,
		.of_match_table = ssd1362_of_match,
		.pm = &ssd1362_pm_ops,
	},
	.id_table = ssd1362_id,
	.probe = ssd1362_probe,
	.remove = ssd1362_remove,
	.shutdown = ssd1362_shutdown,
};
module_spi_driver(ssd1362_spi_driver);

MODULE_DESCRIPTION("SSD1362 OLED Driver");
MODULE_AUTHOR("Above");
MODULE_LICENSE("GPL");
