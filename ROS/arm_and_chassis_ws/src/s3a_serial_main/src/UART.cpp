
#include "UART.hpp"
#include <cstdio>

// Used for UART
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

// Used for error handling.
#include <stdexcept>
#include <sstream>
#include <errno.h>
	

UART::UART(const string& dev_fn, int baud_rate) {
	peek_buf_empty = true;

	// Open in non blocking read/write mode.
	// The flags (defined in fcntl.h):
	//	O_RDWR
	//		read/write mode.
	//	O_NONBLOCK
	//		Enables nonblocking mode.
	//		Instead of blocking,
	//		read can return immediately
	//		with a failure status
	//		if there is no input immediately available.
	//		Likewise, write requests can also return immediately
	//		with a failure status
	//		if the output cannot be written immediately.
	//	O_NOCTTY
	//		When set and path identifies a terminal device,
	//		open() shall not cause the terminal device
	//		to become the controlling terminal for the process.
	uart_fd = open(dev_fn.c_str(), O_RDWR | O_NONBLOCK | O_NOCTTY);
	if (uart_fd == -1){
		throw runtime_error(
			"ERROR: Unable to open UART!"\
				"Ensure it is not in use by another application."
		);
	}
	
	int baud_rate_code;
	switch(baud_rate){
		case 1200:
			baud_rate_code = B1200;
			break;
		case 2400:
			baud_rate_code = B2400;
			break;
		case 4800:
			baud_rate_code = B4800;
			break;
		case 9600:
			baud_rate_code = B9600;
			break;
		case 19200:
			baud_rate_code = B19200;
			break;
		case 38400:
			baud_rate_code = B38400;
			break;
		case 57600:
			baud_rate_code = B57600;
			break;
		case 115200:
			baud_rate_code = B115200;
			break;
		case 230400:
			baud_rate_code = B230400;
			break;
		case 460800:
			baud_rate_code = B460800;
			break;
		case 500000:
			baud_rate_code = B500000;
			break;
		case 576000:
			baud_rate_code = B576000;
			break;
		case 921600:
			baud_rate_code = B921600;
			break;
		case 1000000:
			baud_rate_code = B1000000;
			break;
		case 1152000:
			baud_rate_code = B1152000;
			break;
		case 1500000:
			baud_rate_code = B1500000;
			break;
		case 2000000:
			baud_rate_code = B2000000;
			break;
		case 2500000:
			baud_rate_code = B2500000;
			break;
		case 3000000:
			baud_rate_code = B3000000;
			break;
		case 3500000:
			baud_rate_code = B3500000;
			break;
		case 4000000:
			baud_rate_code = B4000000;
			break;
		default:
			throw runtime_error("ERROR: Unexisting baud rate!");
			break;
	}
	
	//CONFIGURE THE UART
	//The flags (defined in /usr/include/termios.h - see http://pubs.opengroup.org/onlinepubs/007908799/xsh/termios.h.html):
	//	Baud rate:- B1200, B2400, B4800, B9600, B19200, B38400, B57600, B115200, B230400, B460800, B500000, B576000, B921600, B1000000, B1152000, B1500000, B2000000, B2500000, B3000000, B3500000, B4000000
	//	CSIZE:- CS5, CS6, CS7, CS8
	//	CLOCAL - Ignore modem status lines
	//	CREAD - Enable receiver
	//	IGNPAR - Ignore characters with parity errors
	//	ICRNL - Map CR to NL on input (Use for ASCII comms where you want to auto correct end of line characters - don't use for bianry comms!)
	//	PARENB - Parity enable
	//	PARODD - Odd parity (else even)
	struct termios options;
	tcgetattr(uart_fd, &options);
	options.c_cflag = baud_rate_code | CS8 | CLOCAL | CREAD;
	options.c_iflag = IGNPAR;
	options.c_oflag = 0;
	options.c_lflag = 0;
	tcflush(uart_fd, TCIFLUSH);
	tcsetattr(uart_fd, TCSANOW, &options);
}

UART::~UART() {
	close(uart_fd);
}

void UART::write(const void* buf, size_t count) {
	int written = ::write(uart_fd, buf, count);
	if(written != count){
		ostringstream oss;
		oss << "UART TX error: " 
			<< written << " of " << count << " B written "
			<< "with errno = " << errno << "!";
		throw runtime_error(oss.str());
	}
}

/**
 * @return true if successful.
 */
bool UART::read(void* buf, size_t count, bool non_block) {
	uint8_t* it = reinterpret_cast<uint8_t*>(buf);
	while(count != 0){
		int rx_length;
		if(!peek_buf_empty){
			rx_length = 1;
			*it = peek_buf;
			peek_buf_empty = true;
		}else{
			rx_length = ::read(uart_fd, reinterpret_cast<void*>(it), count);
		}
		
		if(rx_length < 0){
			// An error occured aka no more bytes.
			if(non_block){
				return false;
			}
		}else if(rx_length < count){
			count -= rx_length;
			it += rx_length;
		}else{
			break;
		}
	}
	return true;
}

void UART::skip(size_t count) {
	uint8_t c;
	while(count != 0){
		int rx_length;
		if(!peek_buf_empty){
			rx_length = 1;
			peek_buf_empty = true;
		}else{
			rx_length = ::read(uart_fd, reinterpret_cast<void*>(&c), 1);
		}
		
		if(rx_length < 0){
			// An error occured aka no more bytes.
		}else if(rx_length < count){
			count -= rx_length;
		}else{
			break;
		}
	}
}

vector<uint8_t> UART::read() {
	vector<uint8_t> d(256);
	int rx_length = ::read(uart_fd, reinterpret_cast<void*>(d.data()), 256);
	
	if(rx_length < 0){
		//An error occured (will occur if there are no bytes).
		//TODO Handle it.
	}else if (rx_length == 0){
		//No data waiting.
		d.resize(rx_length);
	}else{
		//Bytes received
		d.resize(rx_length);
	}
	
	return d;
}

uint8_t UART::peek() {
	if(peek_buf_empty){
		read(peek_buf);
		peek_buf_empty = false;
	}
	return peek_buf;
}

uint8_t UART::get() {
	uint8_t c;
	read(c);
	return c;
}
