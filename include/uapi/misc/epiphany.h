/*
 * Copyright (C) 2015 Adapteva Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * The full GNU General Public License is included in this distribution in the
 * file called COPYING.
 */

#ifndef _UAPI_MISC_EPIPHANY_H
#define _UAPI_MISC_EPIPHANY_H

#include <linux/types.h>
#include <linux/ioctl.h>

#define E_MESH_MAX_ARRAYS	256
#define E_LINK_MAX_MEM_MAPPINGS	256

enum e_link_side {
	E_SIDE_N = 0,
	E_SIDE_E,
	E_SIDE_S,
	E_SIDE_W,
	E_SIDE_MAX
};

enum e_connection_type {
	E_CONN_DISCONNECTED = 0,
	E_CONN_ELINK,
	E_CONN_ARRAY,
	E_CONN_MAX
};

enum e_chip_type {
	E_CHIP_INVAL = 0,
	E_CHIP_E16G301,
	E_CHIP_E64G401,
	E_CHIP_MAX
};

struct e_mappings_info {
	__u32 nmappings;
	struct {
		__u32 emesh_addr;
		__u32 size;
	} mappings[0];
};

struct e_array_info {
	__u32 id;
	enum e_chip_type chip_type;
	__u32 chip_rows;
	__u32 chip_cols;
	enum e_link_side parent_side;
	dev_t mesh_dev;
	struct {
		enum e_connection_type type;
		union {
			dev_t dev;
			__u32 id;
		};
	} connections[E_SIDE_MAX];
};

struct e_elink_info {
	dev_t dev;
	__u32 version;
	enum e_connection_type connection_type;
	union {
		struct e_array_info array;
		__u32 remote_elink_id;
	};
};

struct e_mesh_info {
	dev_t dev;
	enum e_chip_type chip_type;
	__u32 narrays;
	struct e_array_info arrays[E_MESH_MAX_ARRAYS];
};

#define E_IOCTL_MAGIC  'E'
#define E_IO(nr)		_IO(E_IOCTL_MAGIC, nr)
#define E_IOR(nr, type)		_IOR(E_IOCTL_MAGIC, nr, type)
#define E_IOW(nr, type)		_IOW(E_IOCTL_MAGIC, nr, type)
#define E_IOWR(nr, type)	_IOWR(E_IOCTL_MAGIC, nr, type)

/**
 * If you add an IOC command, please update the
 * EPIPHANY_IOC_MAXNR macro
 */

#define E_IOCTL_RESET			E_IO(0x00)
#define E_IOCTL_ELINK_PROBE		E_IOR(0x01, struct e_elink_info)
#define E_IOCTL_MESH_PROBE		E_IOR(0x02, struct e_array_info)
#define E_IOCTL_GET_MAPPINGS		E_IOR(0x03, struct e_mappings_info)

#define E_IOCTL_MAXNR			0x03

#endif /* _UAPI_MISC_EPIPHANY_H */
