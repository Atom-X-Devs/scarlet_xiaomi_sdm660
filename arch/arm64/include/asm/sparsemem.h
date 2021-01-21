/*
 * Copyright (C) 2012 ARM Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef __ASM_SPARSEMEM_H
#define __ASM_SPARSEMEM_H

#ifdef CONFIG_SPARSEMEM
#define MAX_PHYSMEM_BITS	CONFIG_ARM64_PA_BITS
#ifndef CONFIG_MEMORY_HOTPLUG
/*
 * Section size must be at least 512MB for 64K base
 * page size config. Otherwise it will be less than
 * (MAX_ORDER - 1) and the build process will fail.
 */
#ifdef CONFIG_ARM64_64K_PAGES
#define SECTION_SIZE_BITS 29

#else

/*
 * Section size must be at least 128MB for 4K base
 * page size config. Otherwise PMD based huge page
 * entries could not be created for vmemmap mappings.
 * 16K follows 4K for simplicity.
 */
#define SECTION_SIZE_BITS 27
#endif /* CONFIG_ARM64_64K_PAGES */

#else
#define SECTION_SIZE_BITS	CONFIG_HOTPLUG_SIZE_BITS
#endif
#endif /* CONFIG_SPARSEMEM*/

#endif
