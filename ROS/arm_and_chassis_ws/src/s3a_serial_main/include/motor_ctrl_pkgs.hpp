
#pragma once

#include "type_shorts.h"
#include "motor_ctrl_cfg.hpp"
#include "CRC8.hpp"

#define PKG_MAGIC 0xba

struct pkg_header {
	u8 magic;
	u8 daddr : 4;
	u8 saddr : 4;
	u8 type : 4;
	u8 id : 4;
};
enum pkg_addrs {
	pkg_addr__master = 0,
	pkg_addr__housekeeper = 1,
	pkg_addr__motor_id_0 = 2,
	pkg_addr__broadcast = 0xf
};


// Errors from slaves point of view.
#define motor_ctrl_pkg_errs_defs \
	motor_ctrl_pkg_errs_def(pkg_err_code__none) \
	motor_ctrl_pkg_errs_def(pkg_err_code__crc_error) \
	motor_ctrl_pkg_errs_def(pkg_err_code__wrong_pkg_magic) \
	motor_ctrl_pkg_errs_def(pkg_err_code__some_slave_ordering) \
	motor_ctrl_pkg_errs_def(pkg_err_code__master_responding) \
	motor_ctrl_pkg_errs_def(pkg_err_code__nonexisting_pkg_type) \
	motor_ctrl_pkg_errs_def(pkg_err_code__watchdog_stop)

#define motor_ctrl_pkg_errs_def(n) n,
enum pkg_err_codes {
	motor_ctrl_pkg_errs_defs
	pkg_err_code__last
};


#define motor_ctrl_pkgs_defs \
	motor_ctrl_pkgs_def1(err_get_req) \
	motor_ctrl_pkgs_def2( \
		err_get_rsp, \
		{ \
			u8 err_code : 4; \
			u8 : 4; \
			union { \
				pkg_header header; \
			} data; \
		} \
	) \
	motor_ctrl_pkgs_def1(addr_get_req) \
	motor_ctrl_pkgs_def2( \
		addr_get_rsp, \
		{ \
			u8 addr : 4; \
		} \
	) \
	motor_ctrl_pkgs_def2( \
		pos_set, \
		{ \
			i8 pos[N_JOINTS]; \
		} \
	) \
	motor_ctrl_pkgs_def1(pos_get_req) \
	motor_ctrl_pkgs_def2( \
		pos_get_rsp, \
		{ \
			i8 pos[N_JOINTS]; \
		} \
	) \
	motor_ctrl_pkgs_def2( \
		calib_pos_set, \
		{ \
			i16 pos; \
		} \
	)
	
#define pkg_type(n) pkg_type__##n

#define motor_ctrl_pkgs_def1(n) pkg_type(n),
#define motor_ctrl_pkgs_def2(n, f) pkg_type(n),
	enum pkg_types {
		motor_ctrl_pkgs_defs
		pkg_type(last)
	};


#define pkg_payload(n) pkg_payload__##n
#undef motor_ctrl_pkgs_def1
#undef motor_ctrl_pkgs_def2
#define motor_ctrl_pkgs_def1(n)
#define motor_ctrl_pkgs_def2(n, f) \
	typedef struct __attribute__((packed)) f pkg_payload(n);
	
	motor_ctrl_pkgs_defs
	
	
#undef motor_ctrl_pkgs_def1
#undef motor_ctrl_pkgs_def2
#define motor_ctrl_pkgs_def1(n) 0,
#define motor_ctrl_pkgs_def2(n, f) sizeof(pkg_payload(n)),
	static u8 pkg_payload_sizes[16] = {
		motor_ctrl_pkgs_defs
		0
	};
// err_get_rsp is largers.
#define pkg_payload_max_size (sizeof(pkg_header)+1)

#define pkg(n) pkg__##n
#undef motor_ctrl_pkgs_def1
#undef motor_ctrl_pkgs_def2
#define motor_ctrl_pkgs_def1(n) \
	struct __attribute__((packed)) pkg(n) { \
		pkg_header header; \
		u8 crc; \
	};
#define motor_ctrl_pkgs_def2(n, f) \
	struct __attribute__((packed)) pkg(n) { \
		pkg_header header; \
		pkg_payload(n) payload; \
		u8 crc; \
	};
	
	motor_ctrl_pkgs_defs

#define pkg_without_paylod_init(p, n, a, pkg_id) \
	pkg(n) p = { \
		.header = { \
			.magic = PKG_MAGIC, \
			.daddr = a, \
			.saddr = addr, \
			.type = pkg_type(n), \
			.id = pkg_id \
		}, \
	}; \
	p.crc = CRC8().add(p.header).get_crc();
#define pkg_with_paylod_init(p, n, a, pkg_id, pl) \
	pkg(n) p = { \
		.header = { \
			.magic = PKG_MAGIC, \
			.daddr = a, \
			.saddr = addr, \
			.type = pkg_type(n), \
			.id = pkg_id \
		}, \
		.payload = pl \
	}; \
	p.crc = CRC8().add(p.header).add(p.payload).get_crc();


#if !__AVR__

#undef motor_ctrl_pkgs_def1
#undef motor_ctrl_pkgs_def2
#define motor_ctrl_pkgs_def1(n) #n,
#define motor_ctrl_pkgs_def2(n, f) #n,
	static const char* pkg_type_names[] = {
		motor_ctrl_pkgs_defs
		""
	};

#define n_pkg_type (sizeof(pkg_type_names)/sizeof(pkg_type_names[0])-1)


#undef motor_ctrl_pkg_errs_def
#define motor_ctrl_pkg_errs_def(n) #n,
static const char* pkg_err_names[] = {
	motor_ctrl_pkg_errs_defs
	""
};

#define pkg_err_num (sizeof(pkg_err_names)/sizeof(pkg_err_names[0])-1)


#include <iterator>
#include <iostream>
#include <iomanip>

static std::ostream& operator<<(std::ostream& os, const pkg_header& h) {
	std::ios old_state(nullptr);
	old_state.copyfmt(os);
	os
		<< "pkg_header{"
		<< "magic = " << "0x" << std::hex << (int)h.magic << std::dec << ", "
		<< "daddr = " << (int)h.daddr << ", "
		<< "saddr = " << (int)h.saddr << ", "
		<< "type = " << (int)h.type;
	if((int)h.type < n_pkg_type){
		os << " \"" << pkg_type_names[h.type] << "\"";
	}
	os << ", ";
	os
		<< "id = " << (int)h.id;
	os << "}";
	os.copyfmt(old_state);
	return os;
}

static std::ostream& operator<<(
	std::ostream& os,
	const pkg_payload(err_get_rsp)& pl
) {
	os
		<< "pkg_payload__err_get_rsp{"
		<< "err_code = " << (int)pl.err_code;
	if((int)pl.err_code < pkg_err_num){
		os << " \"" << pkg_err_names[pl.err_code] << "\"";
	}
	os << ", ";
	os
		<< "data.header = " << pl.data.header;
	os << "}";
	return os;
}

#endif
