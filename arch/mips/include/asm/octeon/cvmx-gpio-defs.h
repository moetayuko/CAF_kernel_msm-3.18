/***********************license start***************
 * Author: Cavium Networks
 *
 * Contact: support@caviumnetworks.com
 * This file is part of the OCTEON SDK
 *
 * Copyright (c) 2003-2017 Cavium, Inc.
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, Version 2, as
 * published by the Free Software Foundation.
 *
 * This file is distributed in the hope that it will be useful, but
 * AS-IS and WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE, TITLE, or
 * NONINFRINGEMENT.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this file; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
 * or visit http://www.gnu.org/licenses/.
 *
 * This file may also be available under a different license from Cavium.
 * Contact Cavium Networks for more information
 ***********************license end**************************************/

#ifndef __CVMX_GPIO_DEFS_H__
#define __CVMX_GPIO_DEFS_H__

#define CVMX_GPIO_INT_CLR	(CVMX_ADD_IO_SEG(0x0001070000000898ull))
#define CVMX_GPIO_TX_SET	(CVMX_ADD_IO_SEG(0x0001070000000888ull))

static inline uint64_t CVMX_GPIO_BIT_CFGX(unsigned long offset)
{
	switch (cvmx_get_octeon_family()) {
	case OCTEON_CNF71XX & OCTEON_FAMILY_MASK:
	case OCTEON_CN30XX & OCTEON_FAMILY_MASK:
	case OCTEON_CN31XX & OCTEON_FAMILY_MASK:
	case OCTEON_CN38XX & OCTEON_FAMILY_MASK:
	case OCTEON_CN50XX & OCTEON_FAMILY_MASK:
	case OCTEON_CN52XX & OCTEON_FAMILY_MASK:
	case OCTEON_CN56XX & OCTEON_FAMILY_MASK:
	case OCTEON_CN58XX & OCTEON_FAMILY_MASK:
	case OCTEON_CN61XX & OCTEON_FAMILY_MASK:
	case OCTEON_CN63XX & OCTEON_FAMILY_MASK:
	case OCTEON_CN66XX & OCTEON_FAMILY_MASK:
	case OCTEON_CN68XX & OCTEON_FAMILY_MASK:
	case OCTEON_CN70XX & OCTEON_FAMILY_MASK:
		return CVMX_ADD_IO_SEG(0x0001070000000800ull) + (offset * 8);
	default:
		return CVMX_ADD_IO_SEG(0x0001070000000900ull) + (offset * 8);
	};
}


union cvmx_gpio_bit_cfgx {
	uint64_t u64;
	struct cvmx_gpio_bit_cfgx_s {
		__BITFIELD_FIELD(uint64_t reserved_17_63:47,
		__BITFIELD_FIELD(uint64_t synce_sel:2,
		__BITFIELD_FIELD(uint64_t clk_gen:1,
		__BITFIELD_FIELD(uint64_t clk_sel:2,
		__BITFIELD_FIELD(uint64_t fil_sel:4,
		__BITFIELD_FIELD(uint64_t fil_cnt:4,
		__BITFIELD_FIELD(uint64_t int_type:1,
		__BITFIELD_FIELD(uint64_t int_en:1,
		__BITFIELD_FIELD(uint64_t rx_xor:1,
		__BITFIELD_FIELD(uint64_t tx_oe:1,
		;))))))))))
	} s;
};

#endif
