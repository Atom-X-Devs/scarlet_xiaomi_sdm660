/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2023 Tashfin Shakeer Rhythm <tashfinshakeerrhythm@gmail.com>.
 */

#ifndef _KPROFILES_H_
#define _KPROFILES_H_

#ifdef CONFIG_KPROFILES
void kp_set_mode_rollback(unsigned int level, unsigned int duration_ms);
void kp_set_mode(unsigned int level);
int kp_active_mode(void);
#else
static inline void kp_set_mode_rollback(unsigned int level,
					unsigned int duration_ms) {}
static inline void kp_set_mode(unsigned int level) {}
static inline int kp_active_mode(void) { return 0; }
#endif

#endif /* _KPROFILES_H_ */
