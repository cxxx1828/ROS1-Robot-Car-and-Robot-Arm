#pragma once

#include <stdint.h>

#define CRC8_DEFAULT_POLYNOME 0x07
#define CRC8_POLYNOME CRC8_DEFAULT_POLYNOME

class CRC8 {
public:
	CRC8() {
		restart();
	}
	
	void restart() {
		_crc = 0;
	}

	CRC8& add(uint8_t value) {
		_crc ^= value;
		for(uint8_t i = 8; i; i--){
			_crc <<= 1;
			if(_crc & (1 << 7)){
				_crc ^= CRC8_POLYNOME;
			}
		}
		return *this;
	}
	
	CRC8& add(const uint8_t* array, uint16_t length) {
		while(length--){
			add(*array++);
		}
		return *this;
	}
	
	template<typename T>
	CRC8& add(const T& t) {
		add((uint8_t*)&t, sizeof(T));
		return *this;
	}

	uint8_t get_crc() const {
		return _crc;
	}

private:
	uint8_t  _crc;
};


// -- END OF FILE --
