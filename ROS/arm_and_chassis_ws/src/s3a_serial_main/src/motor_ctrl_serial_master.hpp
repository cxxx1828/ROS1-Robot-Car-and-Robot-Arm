
#pragma once

#include "type_shorts.h"
#include <string>
#include <vector>
#include <deque>
#include <map>
#include <functional>
using namespace std;

class UART;
class pkg_header;
class CRC8;

class Motor_Ctrl_Serial_Master {
	public:
		Motor_Ctrl_Serial_Master();
		~Motor_Ctrl_Serial_Master();
		
		void check_for_errors();
		void set_pos(const vector<i8>& pos);
		// All pos in pulses.
		void get_pos(vector<i8>& pos);
		void calib_pos(u8 id, i16 pos);//TODO Remove
		
	private:
		const u8 addr = 0; // Master addr.
		struct Cnt {
			u8 id : 4;
			Cnt(u8 x) :
				id(x)
			{
			}
		};
		Cnt cnt_tx;
		Cnt cnt_rx_exp;
		
		size_t errors;
		bool error_occured;
		void error_flush(UART* sp);
		bool rx_pkg(
			bool try_header,
			UART* sp,
			int addr, // -1 to skip
			u8 exp_pkg_type,
			function<bool(UART*, CRC8&)> read_payload,
			function<void(const pkg_header& h)> process_payload
		);

		
		vector<UART*> serial_ports;
		struct Slaves_On_Port {
			string dev_fn;
			UART* serial_port;
			deque<u8> addrs;
			
			Slaves_On_Port(
				string dev_fn,
				UART* serial_port
			) :
				dev_fn(dev_fn),
				serial_port(serial_port)
			{}
		};
		vector<Slaves_On_Port> slave_routing_table;
		
		struct Shortcut {
			string dev_fn;
			UART* serial_port;
			u8 addr;
			
			Shortcut() :
				dev_fn(""),
				serial_port(nullptr),
				addr(-1)
			{}
			Shortcut(
				string dev_fn,
				UART* serial_port,
				u8 addr
			) :
				dev_fn(dev_fn),
				serial_port(serial_port),
				addr(addr)
			{}
		};
		Shortcut housekeeper;
		map<u8, UART*> motor_addr_to_serial_port;
		vector<vector<Shortcut>> table_of_synced;
		Shortcut arm;
};
