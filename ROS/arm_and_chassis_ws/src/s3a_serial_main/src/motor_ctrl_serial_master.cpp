
#include "motor_ctrl_serial_master.hpp"

//TODO error rate.

#if TEST_OUTSIDE_ROS

#define LOG_ERROR(x) do{ cerr << "ERROR: " << x << endl; }while(0)
#define LOG_WARN(x) do{ cerr << "WARN: " << x << endl; }while(0)
#define LOG_INFO(x) do{ cerr << "INFO: " << x << endl; }while(0)
#define LOG_DEBUG(x) do{ cerr << "DEBUG: " << x << endl; }while(0)

#else

#include <ros/ros.h>
#define ROS_NODE NAME "robot_hardware_interface"
#define LOG_ERROR(x) ROS_ERROR_STREAM(x)
#define LOG_WARN(x) ROS_WARN_STREAM(x)
#define LOG_INFO(x) ROS_INFO_STREAM(x)
#define LOG_DEBUG(x) ROS_DEBUG_STREAM(x)

#endif

#include "motor_ctrl_pkgs.hpp"

#include "glob.hpp"
#include "UART.hpp"

#include <unistd.h> // sleep()

#include <algorithm>
#include <sstream>
#include <set>


Motor_Ctrl_Serial_Master::Motor_Ctrl_Serial_Master() :
	cnt_tx(0),
	cnt_rx_exp(0),
	errors(0),
	error_occured(false)
{

	vector<string> dev_fns = glob(UART_DEV_PATTERN);

	for(auto dev_fn: dev_fns){
		auto sp = new UART(
			dev_fn,
			BAUDRATE
		);
		serial_ports.emplace_back(sp);
		slave_routing_table.emplace_back(dev_fn, sp);
	}
	
	//TODO Solve this problem on first read of UART.
	// -H It is because Arduino FW start 1 sec after reset.
	// -H Global problem with non-RT drivers on Linux.
	sleep(3);
	
	//TODO Do this twice, because addr_get_req could be lost.
	
	// Send reqest.
	cnt_rx_exp = cnt_tx;
	for(int i = 0; i < slave_routing_table.size(); i++){
		pkg_without_paylod_init(
			p,
			addr_get_req,
			pkg_addr__broadcast,
			cnt_tx.id++
		);
		slave_routing_table[i].serial_port->write(p);
	}
	
	set<u8> all_addrs;
	// Receive responses.
	for(int i = 0; i < slave_routing_table.size(); i++){
		auto sp = slave_routing_table[i].serial_port;
		
		pkg_header h;
		bool read_ok;
		read_ok = sp->read(h);
		if(!read_ok){
			// Nothing to read, go to the next serial port.
			continue;
		}
		if(h.magic != PKG_MAGIC){
			errors++; error_occured = true;
			cerr << endl << "magic mismatch!" << endl;
			cerr << h << endl;
		}
		// Expecting broadcasted addr_get_req as first pkg.
		if(
			!(
				h.daddr == pkg_addr__broadcast &&
				h.type == pkg_type(addr_get_req)
			)
		){
			errors++; error_occured = true;
			cerr << endl 
				<< "Did not received broadcasted packet first!" << endl;
			cerr << h << endl;
		}else{
			sp->skip(pkg_payload_sizes[h.type] + 1);
		}
		error_flush(sp);
		
		bool do_break = false;
		do{
			pkg_payload(addr_get_rsp) pl;
			do_break = rx_pkg(
				true,
				sp,
				-1,
				pkg_type(addr_get_rsp),
				[&](UART* sp, CRC8& crc){
					bool read_ok = sp->read(pl);
					crc.add(pl);
					return read_ok;
				},
				[&](const pkg_header& h){
					u8 a = pl.addr;
					if(a != h.saddr){
						errors++; error_occured = true;
						LOG_ERROR("saddr mismatch addr!");
						LOG_DEBUG("observed: " << (int)a);
						LOG_DEBUG("expecting: " << (int)h.saddr);
						LOG_DEBUG(h);
						return;
					}
					
					if(all_addrs.count(a)){
						LOG_WARN(
							"Have 2 Arduinos with same addr = "
								<< (int)a << "!"
						);
					}else{
						all_addrs.insert(a);
					}
					
					switch(a){
						case pkg_addr__master:
						case pkg_addr__broadcast:
							LOG_ERROR(
								"Slave making himself addr = "
									<< (int)a << "!"
							);
							break;
						case pkg_addr__housekeeper:
							slave_routing_table[i].addrs.push_front(a);
							housekeeper = Shortcut(
								slave_routing_table[i].dev_fn,
								sp,
								a
							);
							break;
						case pkg_addr__motor_id_0:
							slave_routing_table[i].addrs.push_front(a);
							arm = Shortcut(
								slave_routing_table[i].dev_fn,
								sp,
								a
							);
							break;
						default:  // Other IDs.
							slave_routing_table[i].addrs.push_front(a);
							motor_addr_to_serial_port[a] = sp;
							break;
					}
				}
			);
		}while(!do_break);
		
		cnt_rx_exp.id++;
	}
	
	
	
	size_t max_slaves_on_port = 0;
	for(const auto sop : slave_routing_table){
		max_slaves_on_port = max(max_slaves_on_port, sop.addrs.size());
	}
	
	table_of_synced.resize(max_slaves_on_port);
	for(int i = 0; i < max_slaves_on_port; i++){
		for(const auto sop : slave_routing_table){
			int ri = max_slaves_on_port-1-i;
			if(ri < sop.addrs.size()){
				table_of_synced[i].emplace_back(
					sop.dev_fn,
					sop.serial_port,
					sop.addrs[ri]
				);
			}
		}
	}
	
	
	
	ostringstream oss;
	oss << endl;
	oss << "Slave Routing Table:" << endl;
	oss << "dev fn" << "\t\t" << "type and addr in chain" << endl;
	for(const auto sop : slave_routing_table){
		oss << sop.dev_fn;
		for(const auto addr : sop.addrs){
			switch(addr){
				case pkg_addr__master:
				case pkg_addr__broadcast:
					oss << "\t ";
					break;
				case pkg_addr__housekeeper:
					oss << "\th";
					break;
				default:  // Motors.
					oss << "\tm";
					break;
			}
			oss << (int)addr;
		}
		oss << endl;
	}
	LOG_INFO(oss.str());
	
	oss.str("");
	oss << endl;
	oss << "Table of synced:" << endl;
	oss << "iter" << "\t\t" << "dev_fn addr" << endl;
	for(int i = 0; i < table_of_synced.size(); i++){
		oss << i;
		for(auto s : table_of_synced[i]){
			oss << "\t" << s.dev_fn << " m" << (int)s.addr;
		}
		oss << endl;
	}
	LOG_INFO(oss.str());
}

Motor_Ctrl_Serial_Master::~Motor_Ctrl_Serial_Master() {
	for(auto sp : serial_ports) {
		delete sp;
	}
}

void Motor_Ctrl_Serial_Master::check_for_errors() {
	cnt_rx_exp = cnt_tx;
	for(const auto sop : slave_routing_table){
		pkg_without_paylod_init(
			p,
			err_get_req,
			pkg_addr__broadcast,
			cnt_tx.id++
		);
		sop.serial_port->write(p);
	}
	
	for(const auto sop : slave_routing_table){
		rx_pkg(
			false,
			sop.serial_port,
			pkg_addr__master,
			pkg_type(err_get_req),
			[&](UART* sp, CRC8& crc){
				return true;
			},
			[&](const pkg_header& h){
			}
		);
		
		// Reverse order.
		for(auto pa = sop.addrs.rbegin(); pa != sop.addrs.rend(); pa++){
			pkg_payload(err_get_rsp) pl;
			rx_pkg(
				false,
				sop.serial_port,
				*pa,
				pkg_type(err_get_rsp),
				[&](UART* sp, CRC8& crc){
					bool read_ok = sp->read(pl);
					crc.add(pl);
					return read_ok;
				},
				[&](const pkg_header& h){
					if(pl.err_code != pkg_err_code__none){
						errors++;
						cerr << endl << pl << endl;
					}
				}
			);
		}
		
		cnt_rx_exp.id++;
	}
}

void Motor_Ctrl_Serial_Master::set_pos(const vector<i8>& pos) {
	pkg_payload(pos_set) pl;
	for(int i = 0; i < N_JOINTS; i++){
		pl.pos[i] = pos[i];
	}
	pkg_with_paylod_init(
		p,
		pos_set,
		arm.addr,
		cnt_tx.id++,
		pl
	);
	arm.serial_port->write(p);
}

void Motor_Ctrl_Serial_Master::get_pos(vector<i8>& pos) {
	cnt_rx_exp = cnt_tx;
	pkg_without_paylod_init(
		p,
		pos_get_req,
		arm.addr,
		cnt_tx.id++
	);
	arm.serial_port->write(p);
	
	pkg_payload(pos_get_rsp) pl;
	rx_pkg(
		false,
		arm.serial_port,
		arm.addr,
		pkg_type(pos_get_rsp),
		[&](UART* sp, CRC8& crc){
			bool read_ok = sp->read(pl);
			crc.add(pl);
			return read_ok;
		},
		[&](const pkg_header& h){
			for(int i = 0; i < N_JOINTS; i++){
				pos[i] = pl.pos[i];
			}
		}
	);
	cnt_rx_exp.id++;
}

void Motor_Ctrl_Serial_Master::calib_pos(u8 id, i16 pos) {
	u8 a = id + pkg_addr__motor_id_0;
	pkg_with_paylod_init(
		p,
		calib_pos_set,
		a,
		cnt_tx.id++,
		pkg_payload(calib_pos_set)({
			.pos = pos
		})
	);
	if(motor_addr_to_serial_port.count(a)){
		motor_addr_to_serial_port[a]->write(p);
	}
}

void Motor_Ctrl_Serial_Master::error_flush(UART* sp) {
	if(error_occured){
		error_occured = false;
		while(sp->peek() != PKG_MAGIC){
			u8 c = sp->get();
			cerr << "flushing: "
				<< (int)c
				<< " 0x" << hex << (int)c << dec
				<< endl;
		}
	}
}

bool Motor_Ctrl_Serial_Master::rx_pkg(
	bool try_header,
	UART* sp,
	int addr, // -1 to skip
	u8 exp_pkg_type,
	function<bool(UART*, CRC8& crc)> read_payload,
	function<void(const pkg_header& h)> process_payload
) {
	pkg_header h;
	bool read_ok;
	if(try_header){
		read_ok = sp->try_read(h);
		if(!read_ok){
			// Nothing more to receive, go to the next serial port.
			return true;
		}
	}else{
		read_ok = sp->read(h);
		if(!read_ok){
			errors++;
			cerr << endl << "Lacking header!" << endl;
			return false;
		}
	}
	if(h.magic != PKG_MAGIC){
		errors++; error_occured = true;
		cerr << endl << "Slave does not know magic word!" << endl;
		cerr << "observed: "
			<< "0x" << hex << (int)h.magic << dec << endl;
		cerr << "expecting: "
			<< "0x" << hex << (int)PKG_MAGIC << dec << endl;
		cerr << h << endl;
	}
	if(h.daddr != pkg_addr__broadcast){
		// It is not broadcasted.
		if(h.daddr != pkg_addr__master){
			errors++; error_occured = true;
			cerr << endl << "Treacherous slaves making a rebelion: "
				<< "They answer other than their master!" << endl;
			cerr << "observed: " << (int)h.saddr << endl;
			cerr << "expecting: " << (int)pkg_addr__master << endl;
			cerr << h << endl;
		}
	}
	if(addr != -1){
		if(h.saddr != addr){
			errors++; error_occured = true;
			cerr << endl << "Rude slave taking a word to master "
				<< "without given premission to talk!" << endl;
			cerr << "observed: " << (int)h.saddr << endl;
			cerr << "expecting: " << (int)addr << endl;
			cerr << h << endl;
		}
	}
	if(h.type != exp_pkg_type){
		errors++; error_occured = true;
		cerr << endl 
			<< "Drunken slave sending pkg header of wrong type!" << endl;
		cerr << "observed: " << (int)h.type << endl;
		cerr << "expecting: " << (int)exp_pkg_type << endl;
		cerr << h << endl;
	}
	if(h.id != cnt_rx_exp.id){
		errors++; error_occured = true;
		cerr << endl << "Sleepy slave does not track fabula!" << endl;
		cerr << "observed: " << (int)h.id << endl;
		cerr << "expecting: " << (int)cnt_rx_exp.id << endl;
		cerr << h << endl;
	}
	if(!error_occured){
		CRC8 crc;
		crc.add(h);
		read_ok = read_payload(sp, crc);
		if(!read_ok){
			errors++;
			cerr << endl << "Lacking payload!" << endl;
		}else{
			u8 obs_crc;
			read_ok = sp->read(obs_crc);
			if(!read_ok){
				errors++;
				cerr << endl << "Lacking crc!" << endl;
			}else{
				if(obs_crc != crc.get_crc()){
					errors++;
					cerr << endl << "CRC error!" << endl;
				}else{
					process_payload(h);
				}
			}
		}
	}
	error_flush(sp);
	
	return false;
}
