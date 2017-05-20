/*
 * Copyright (C) 2004 MontaVista Software, Inc.
 * Copyright (C) 2014 The Chromium OS Authors.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef __ASM_TIME_H
#define __ASM_TIME_H

struct timespec;
typedef void (*clock_access_fn)(struct timespec *);
extern int register_persistent_clock(clock_access_fn read_boot,
		clock_access_fn read_persistent);

#endif
