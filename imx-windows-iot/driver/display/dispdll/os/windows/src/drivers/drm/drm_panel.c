/*
 * Copyright (C) 2013, NVIDIA Corporation.  All rights reserved.
 * Copyright 2022,2024 NXP
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sub license,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the
 * next paragraph) shall be included in all copies or substantial portions
 * of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include <stddef.h>
#include <linux/err.h>
#include <linux/device.h>
#include <drm/drm_panel.h>
#include <drm/drm_print.h>

#define DRM_PANEL_MAX_NUM 16

/**
 * DOC: drm panel
 *
 * The DRM panel helpers allow drivers to register panel objects with a
 * central registry and provide functions to retrieve those panels in display
 * drivers.
 *
 * For easy integration into drivers using the &drm_bridge infrastructure please
 * take look at drm_panel_bridge_add() and devm_drm_panel_bridge_add().
 */

static struct drm_panel *panel_list[DRM_PANEL_MAX_NUM] = { 0 };

/**
 * drm_panel_init - initialize a panel
 * @panel: DRM panel
 * @dev: parent device of the panel
 * @funcs: panel operations
 * @connector_type: the connector type (DRM_MODE_CONNECTOR_*) corresponding to
 *	the panel interface
 *
 * Initialize the panel structure for subsequent registration with
 * drm_panel_add().
 */
void drm_panel_init(struct drm_panel *panel, struct device *dev,
		    const struct drm_panel_funcs *funcs)
{
	panel->dev = dev;
	panel->funcs = funcs;
}

/**
 * drm_panel_add - add a panel to the global registry
 * @panel: panel to add
 *
 * Add a panel to the global registry so that it can be looked up by display
 * drivers.
 */
void drm_panel_add(struct drm_panel *panel)
{
	int i;

	for (i = 0; i < DRM_PANEL_MAX_NUM; i++) {
		if (!panel_list[i]) {
			panel_list[i] = panel;
			return;
		}
	}
	DRM_ERROR("drm_panel: No free space to add new panel\n");

}

/**
 * drm_panel_remove - remove a panel from the global registry
 * @panel: DRM panel
 *
 * Removes a panel from the global registry.
 */
void drm_panel_remove(struct drm_panel *panel)
{
	int i;

	for (i = 0; i < DRM_PANEL_MAX_NUM; i++) {
		if (panel_list[i] == panel) {
			panel_list[i] = NULL;
			return;
		}
	}
	DRM_ERROR("drm_panel: Panel being removed not found.\n");
}

/**
 * drm_panel_prepare - power on a panel
 * @panel: DRM panel
 *
 * Calling this function will enable power and deassert any reset signals to
 * the panel. After this has completed it is possible to communicate with any
 * integrated circuitry via a command bus.
 *
 * Return: 0 on success or a negative error code on failure.
 */
int drm_panel_prepare(struct drm_panel *panel)
{
	if (!panel)
		return -EINVAL;

	if (panel->funcs && panel->funcs->prepare)
		return panel->funcs->prepare(panel);

	return 0;
}

/**
 * drm_panel_unprepare - power off a panel
 * @panel: DRM panel
 *
 * Calling this function will completely power off a panel (assert the panel's
 * reset, turn off power supplies, ...). After this function has completed, it
 * is usually no longer possible to communicate with the panel until another
 * call to drm_panel_prepare().
 *
 * Return: 0 on success or a negative error code on failure.
 */
int drm_panel_unprepare(struct drm_panel *panel)
{
	if (!panel)
		return -EINVAL;

	if (panel->funcs && panel->funcs->unprepare)
		return panel->funcs->unprepare(panel);

	return 0;
}

/**
 * drm_panel_enable - enable a panel
 * @panel: DRM panel
 *
 * Calling this function will cause the panel display drivers to be turned on
 * and the backlight to be enabled. Content will be visible on screen after
 * this call completes.
 *
 * Return: 0 on success or a negative error code on failure.
 */
int drm_panel_enable(struct drm_panel *panel)
{
	int ret;

	if (!panel)
		return -EINVAL;

	if (panel->funcs && panel->funcs->enable) {
		ret = panel->funcs->enable(panel);
		if (ret < 0)
			return ret;
	}

	return 0;
}

/**
 * drm_panel_disable - disable a panel
 * @panel: DRM panel
 *
 * This will typically turn off the panel's backlight or disable the display
 * drivers. For smart panels it should still be possible to communicate with
 * the integrated circuitry via any command bus after this call.
 *
 * Return: 0 on success or a negative error code on failure.
 */
int drm_panel_disable(struct drm_panel *panel)
{
	if (!panel)
		return -EINVAL;

	if (panel->funcs && panel->funcs->disable)
		return panel->funcs->disable(panel);

	return 0;
}

/**
 * drm_panel_get_modes - probe the available display modes of a panel
 * @panel: DRM panel
 *
 * Return: The number of modes available from the panel on success or a
 * negative error code on failure.
 */
int drm_panel_get_modes(struct drm_panel *panel)
{
	if (!panel)
		return -EINVAL;

	if (panel->funcs && panel->funcs->get_modes)
		return panel->funcs->get_modes(panel);

	return -EOPNOTSUPP;
}

/**
 * of_drm_find_panel - look up a panel using a device tree node
 * @np: device tree node of the panel
 *
 * Searches the set of registered panels for one that matches the given device
 * tree node. If a matching panel is found, return a pointer to it.
 *
 * Return: A pointer to the panel registered for the specified device tree
 * node or an ERR_PTR() if no panel matching the device tree node can be found.
 *
 * Possible error codes returned by this function:
 *
 * - EPROBE_DEFER: the panel device has not been probed yet, and the caller
 *   should retry later
 * - ENODEV: the device is not available (status != "okay" or "ok")
 */
struct drm_panel *of_drm_find_panel(const struct device_node *np)
{
	int i;
	const char *np_cmptbl = NULL;
	const char *host_cmptbl = NULL;
	int np_cmptbl_len = 0, host_cmptbl_len = 0;
	u32 np_instance = 0, host_instance = 0;

	int ret;

	if (!np)
		return ERR_PTR(-ENODEV);

	np_cmptbl = (const char*)of_get_property(np, "compatible", &np_cmptbl_len);
	ret = of_property_read_u32_array(np, "instance", &np_instance, 1);
	if (ret || !np_cmptbl)
		return ERR_PTR(-ENODEV);

	for (i = 0; i < DRM_PANEL_MAX_NUM; i++) {
		if (panel_list[i] && panel_list[i]->dev) {
			host_cmptbl = (const char *)of_get_property(&panel_list[i]->dev->of_node, "compatible", &host_cmptbl_len);
			ret = of_property_read_u32_array(&panel_list[i]->dev->of_node, "instance", &host_instance, 1);
			if (ret || !host_cmptbl || (host_cmptbl_len != np_cmptbl_len)) {
				continue;
			}
			ret = strcmp(np_cmptbl, host_cmptbl);
			if (!ret && (np_instance == host_instance)) {
				return panel_list[i];
			}
		}
	}

	return ERR_PTR(-EPROBE_DEFER);
}
