/*
 * Copyright (C) 2015 Adapteva Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _UAPI_MISC_EPIPHANY_H
#define _UAPI_MISC_EPIPHANY_H

#include <linux/ioctl.h>

#define EPIPHANY_IOC_MAGIC  'E'

/**
 * If you add an IOC command, please update the
 * EPIPHANY_IOC_MAXNR macro
 */

#define EPIPHANY_IOC_MAXNR  (-1)

#endif /* _UAPI_MISC_EPIPHANY_H */
