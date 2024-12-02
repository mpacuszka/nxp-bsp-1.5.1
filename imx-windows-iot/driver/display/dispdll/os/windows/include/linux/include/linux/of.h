/* SPDX-License-Identifier: GPL-2.0+ */
#ifndef _LINUX_OF_H
#define _LINUX_OF_H
/*
 * Definitions for talking to the Open Firmware PROM on
 * Power Macintosh and other computers.
 *
 * Copyright (C) 1996-2005 Paul Mackerras.
 * Copyright 2022 NXP
 *
 * Updates for PPC64 by Peter Bergner & David Engebretsen, IBM Corp.
 * Updates for SPARC64 by David S. Miller
 * Derived from PowerPC and Sparc prom.h files by Stephen Rothwell, IBM Corp.
 */

struct property {
	char	*name;
	int	length;
	void	*value;
	struct property *next;
};

struct device_node {
	const char *name;
	const char *full_name;
	struct	property *properties;
	struct	device_node *child;
	int num_childs;
	unsigned long _flags;
	void	*data;
};

bool of_property_read_bool(const struct device_node *np,
	const char *propname);

int of_property_read_u32_array(const struct device_node *np,
	const char *propname,
	u32 *out_values, size_t sz);

static inline int of_property_read_u32(const struct device_node *np,
	const char *propname,
	u32 *out_value)
{
	return of_property_read_u32_array(np, propname, out_value, 1);
}

extern int of_property_read_string(const struct device_node *np,
	const char *propname,
	const char **out_string);
extern int of_property_match_string(const struct device_node *np,
	const char *propname,
	const char *string);

const void *of_get_property(const struct device_node *node,
    const char *name, int *lenp);

#if !defined(of_compat_cmp)
#define of_compat_cmp(s1, s2, l)	_stricmp((s1), (s2))
#endif

extern int of_device_is_compatible(const struct device_node *device,
				const char *);
#endif /* _LINUX_OF_H */
