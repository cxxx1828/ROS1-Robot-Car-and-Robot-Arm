
#ifndef UART_HPP
#define UART_HPP

#include <string>
#include <vector>
#include <stdint.h>
using namespace std;

class UART {
public:
	UART(const string& dev_fn, int baud_rate);
	~UART();
	template<typename T>
	void write(const T& t) {
		write(reinterpret_cast<const void*>(&t), sizeof(T));
	}
	template<typename T>
	bool read(T& t) {
		read(reinterpret_cast<void*>(&t), sizeof(T));
		return true; // For simpler interchanging with try_read().
	}
	template<typename T>
	bool try_read(T& t) {
		return read(reinterpret_cast<void*>(&t), sizeof(T), true);
	}
	void skip(size_t count);
	/**
	 * Read as much as can.
	 */
	vector<uint8_t> read();
	uint8_t peek();
	uint8_t get();
private:
	int uart_fd;
	uint8_t peek_buf;
	bool peek_buf_empty;
	void write(const void* buf, size_t count);
	bool read(void* buf, size_t count, bool non_block = false);
};

#endif // UART_HPP
