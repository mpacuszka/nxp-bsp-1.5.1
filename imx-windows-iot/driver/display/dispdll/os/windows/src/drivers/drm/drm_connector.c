/*
 * Copyright (c) 2016 Intel Corporation
 * Copyright 2022 NXP
 *
 * Permission to use, copy, modify, distribute, and sell this software and its
 * documentation for any purpose is hereby granted without fee, provided that
 * the above copyright notice appear in all copies and that both that copyright
 * notice and this permission notice appear in supporting documentation, and
 * that the name of the copyright holders not be used in advertising or
 * publicity pertaining to distribution of the software without specific,
 * written prior permission.  The copyright holders make no representations
 * about the suitability of this software for any purpose.  It is provided "as
 * is" without express or implied warranty.
 *
 * THE COPYRIGHT HOLDERS DISCLAIM ALL WARRANTIES WITH REGARD TO THIS SOFTWARE,
 * INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS, IN NO
 * EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE FOR ANY SPECIAL, INDIRECT OR
 * CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE,
 * DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER
 * TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
 * OF THIS SOFTWARE.
 */

#include <drm/drm_connector.h>
#include <drm/drm_edid.h>

/**
 * drm_connector_update_edid_property - update the edid property of a connector
 * @connector: drm connector
 * @edid: new value of the edid property
 *
 * This function creates a new blob modeset object and assigns its id to the
 * connector's edid property.
 * Since we also parse tile information from EDID's displayID block, we also
 * set the connector's tile property here. See drm_connector_set_tile_property()
 * for more details.
 *
 * Returns:
 * Zero on success, negative errno on failure.
 */
int drm_connector_update_edid_property(struct drm_connector *connector,
				       const struct edid *edid)
{
	size_t size = 0;

	/* ignore requests to set edid when overridden */
	if (connector->override_edid)
		return 0;

	if (edid)
		size = EDID_LENGTH * (1 + edid->extensions);

	/* Set the display info, using edid if available, otherwise
	 * resetting the values to defaults. This duplicates the work
	 * done in drm_add_edid_modes, but that function is not
	 * consistently called before this one in all drivers and the
	 * computation is cheap enough that it seems better to
	 * duplicate it rather than attempt to ensure some arbitrary
	 * ordering of calls.
	 */
	if (edid)
		drm_add_display_info(connector, edid);
	else
		drm_reset_display_info(connector);

	return 0;
}