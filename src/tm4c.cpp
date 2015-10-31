#include <ros/ros.h>
#include "lynxmotion_tm4c/tm4c.h"
#include "unistd.h"

#ifndef DEBUG
#define DEBUG 1
#endif

namespace lynxmotion_tm4c
{

/* Constructor */
TM4C::TM4C() : fd(-1) {
	unsigned int i;
	for(i = 0; i < TM4C::MAX_CHANNELS; i++) {
		first_instruction[i] = 0;
  }
}

/* Destructor */
TM4C::~TM4C() {
	close_port();
}

bool TM4C::open_port(const char *port, int baud)
{
	struct termios options;

	close_port();

	switch(baud)
	{
		case 2400: baud = B2400; break;
		case 9600: baud = B9600; break;
		case 38400: baud = B38400; break;
		case 115200: baud = B115200; break;
		default:
			printf("ERROR: Invalid baud [%d] -- must be 2400, 9600, 38400, or 115200\n", baud);
			return false;
	}

	fd = open(port, O_RDWR | O_NOCTTY);

	if(fd < 0) {
		printf("ERROR: Unable to open device on port %s\n", port);
		return false;
	}

	if(fcntl(fd, F_SETFL, 0) < 0) {
		printf("ERROR: port [%s] is already locked\n", port);
		close_port();
		return false;
	}

	memset(&options, 0, sizeof(options));
	cfsetispeed(&options, baud);
	cfsetospeed(&options, baud);

	options.c_iflag = IGNBRK | IGNPAR;
	options.c_oflag = 0;
	options.c_cflag |= CREAD | CS8 | CLOCAL;
	options.c_lflag = 0;

	if(tcsetattr(fd, TCSANOW, &options) < 0) {
		printf("ERROR: setting termios options\n");
		close_port();
		return false;
	}

	/* Make sure queues are empty */
	tcflush(fd, TCIOFLUSH);
	printf("Successfully opened port %s\n", port);

	return true;
}

bool TM4C::is_connected() {
	return (fd != -1);
}

void TM4C::close_port() {
	unsigned int i;

	if(fd != -1) {
		printf("Closing port\n");
		close(fd);

		fd = -1;
		for(i = 0; i < TM4C::MAX_CHANNELS; i++) {
			first_instruction[i] = 0;
    }
	}
}

bool TM4C::send_message(const char *msg, int size) {
    if(fd != -1) {
      tcflush(fd, TCIOFLUSH);

#if DEBUG
      printf("INFO: [send_message] Sending message: ");
      for(unsigned int i = 0; i < strlen(msg); i++) {
        switch(msg[i]) {
        case '\r': printf("<cr>"); break;
        case 27:   printf("<esc>"); break;
        default:   printf("%c", msg[i]); break;
        }
      }
      printf("\n");
#endif

      int n = 0;
      while (n < size) {
        write(fd, &msg[n++], 1);
        usleep(1000);
      }
    }
    else {
#if DEBUG
      printf("ERROR: [send_message] Device is not open\n");
#endif
      return false;
    }
    return true;
  }

TM4C::move_servo(struct ServoCommand cmd, int time) {
	return move_servo(&cmd, 1, time);
}

bool TM4C::move_servo(struct ServoCommand cmd[], unsigned int n, int time)
{
	char msg[1024] = { 0 };
	char temp[32];
	int time_flag;
	unsigned int i;
	bool result;

	time_flag = 0;

	if(n > TM4C::MAX_CHANNELS) {
#if DEBUG
		printf("ERROR: [move_servo] Invalid number of channels [%u]\n", n);
#endif
		return false;
	}

	for(i = 0; i < n; i++) {
		if(cmd[i].ch > 31) {
#if DEBUG
			printf("ERROR: [move_servo] Invalid channel [%u]\n", cmd[i].ch);
#endif
			return false;
		}

		if(cmd[i].pw < TM4C::MIN_PULSE_WIDTH || cmd[i].pw > TM4C::MAX_PULSE_WIDTH) {
#if DEBUG
			printf("ERROR: [move_servo] Invalid pulse width [%u]\n", cmd[i].pw);
#endif
			return false;
		}

		sprintf(temp, "J %2u P%4u \r", cmd[i].ch, cmd[i].pw);
		strcat(msg, temp);

		if(first_instruction[cmd[i].ch] != 0) {
			if(cmd[i].spd > 0) {
        sprintf(temp, "S%d ", cmd[i].spd);
				strcat(msg, temp);
			}
		}
		else /* this is the first instruction for this channel */
			time_flag++;
	}

	/* If time_flag is 0, then this is not the first instruction */
	/* for any channels to move the servo */
	if(time_flag == 0 && time > 0) {
		sprintf(temp, "T%d ", time);
		strcat(msg, temp);
	}

	strcat(msg, "\r");
  ROS_INFO("HERSHAL: SENDING MESSAGE: %s", msg);
	result = send_message(msg, strlen(msg));

	/* If the command was success, then the channels commanded */
	/* are not on their first instuction anymore. */
	if(result) {
		for(i = 0; i < n; i++) {
			first_instruction[cmd[i].ch] = 1;
    }
  }
	return result;
}

bool TM4C::cancel_command() {
	char msg[4];
	sprintf(msg, "%c \r", 27);
	return send_message(msg, strlen(msg));
}

bool TM4C::pulse_offset(unsigned int ch, int value) {
	return pulse_offset(&ch, &value, 1);
}

bool TM4C::pulse_offset(unsigned int ch[], int value[], unsigned int n) {
	char msg[1024] = { 0 };
	char temp[12];
	unsigned int i;

	if(n > TM4C::MAX_CHANNELS) {
#if DEBUG
		printf("ERROR: [pulse_offset] Invalid number of channels [%u]\n", n);
#endif
		return false;
	}

	for(i = 0; i < n; i++) {
		if(ch[i] > 31) {
#if DEBUG
			printf("ERROR: [pulse_offset] Invalid channel [%u]\n", ch[i]);
#endif
			return false;
		}

		if(value[i] < -100 || value[i] > 100) {
#if DEBUG
			printf("ERROR: [pulse_offset] Invalid offset value [%d]\n", value[i]);
#endif
			return false;
		}

		sprintf(temp, "PO %2u %4d ", ch[i], value[i]);
		strcat(msg, temp);
	}

	strcat(msg, "\r");
	return send_message(msg, strlen(msg));
}

bool TM4C::discrete_output(unsigned int ch, LogicLevel lvl) {
	return discrete_output(&ch, &lvl, 1);
}

bool TM4C::discrete_output(unsigned int ch[], LogicLevel lvl[], unsigned int n) {
	char msg[1024] = { 0 };
	char temp[7];
	unsigned int i;

	if(n > TM4C::MAX_CHANNELS) {
#if DEBUG
		printf("ERROR: [discrete_output] Invalid number of channels [%u]\n", n);
#endif
		return false;
	}

	for(i = 0; i < n; i++) {
		if(ch[i] > 31) {
#if DEBUG
			printf("ERROR: [discrete_output] Invalid servo channel [%u]\n", ch[i]);
#endif
			return false;
		}

		sprintf(temp, "D %2u %c ", ch[i], (lvl[i] == High) ? 'H' : 'L');
		strcat(msg, temp);
	}

	strcat(msg, "\r");
	return send_message(msg, strlen(msg));
}

bool TM4C::byte_output(unsigned int bank, unsigned int value) {
	char msg[10];

	if(bank > 3) {
#if DEBUG
		printf("ERROR: [byte_output] Invalid bank [%u]\n", bank);
#endif
		return false;
	}

	if(value > 255) {
#if DEBUG
		printf("ERROR: [byte_output] Invalid value [%u]\n", value);
#endif
		return false;
	}

	sprintf(msg, "#%d:%d \r", bank, value);
	return send_message(msg, strlen(msg));
}

bool TM4C::query_movement_status()
{
	unsigned char buffer;
	int bytes_read = 0;
	const char *msg = "Q \r";

	if(!send_message(msg, strlen(msg))) {
#if DEBUG
		printf("ERROR: [query_movement_status] Failed to send message\n");
#endif
		return false;
	}

	/* There is a delay of at least 50uS to 5mS, so sleep for 5ms. */
	usleep(10000);

	/* Continue reading from controller until a response is received */
	while(bytes_read != 1) {
		if((bytes_read = read(fd, &buffer, 1)) < 0) {
#if DEBUG
			printf("ERROR: [query_movement_status] Failed to read from the device\n");
#endif
			return false;
		}
	}

	/* Check response value */
  return buffer == '+';
}

int TM4C::query_pulse_width(unsigned int ch)
{
	unsigned char buffer;
	int bytes_read = 0;
	char msg[7];

	// Check if the servo channel is valid
	if(ch > 31) {
#if DEBUG
		printf("ERROR: [query_pulse_width] Invalid servo channel [%u]\n", ch);
#endif
		return false;
	}

	sprintf(msg, "QP %2u \r", ch);

	if(!send_message(msg, strlen(msg))) {
#if DEBUG
    printf("ERROR: [query_pulse_width] Failed to send message\n");
#endif
    return false;
  }

  /* It can take up to 5ms before the controller responds, so sleep for 5ms. */
	usleep(5000);

	while(bytes_read != 1) {
		if((bytes_read = read(fd, &buffer, 1)) < 0) {
#if DEBUG
			printf("ERROR: [query_pulse_width] Failed to read from the device\n");
#endif
			return -1;
		}
	}

	return (10 * (int)buffer);
}

std::string TM4C::get_version() {
	char data[255];
	int bytes_read;
	int total_bytes;
	int i;
	std::string version;
	const char *msg = "VER\r";

	total_bytes = 0;

	if(!send_message(msg, strlen(msg))) {
#if DEBUG
		printf("ERROR: [get_version] Failed to send message\n");
#endif
		return "error";
	}

	usleep(100000);

#if DEBUG
	printf("INFO: [get_version] Reading response\n");
#endif

	while((bytes_read = read(fd, data + total_bytes, 1)) > 0) {
		total_bytes += bytes_read;
  }

#if DEBUG
	printf("INFO: [get_version] Data: ");
	for(i = 0; i < total_bytes; i++) {
		if(data[i] == '\r')
			printf("<cr>");
		else if(data[i] == '\n')
			printf("<nl>");
		else
			printf("%c", data[i]);
	}
	printf("\n");
#endif

	if(bytes_read < 0) {
#if DEBUG
		printf("ERROR: [get_version] Failed to read from device\n");
#endif
	}
	else if(total_bytes > 0) {
#if DEBUG
		printf("Read %d bytes\n", total_bytes);
#endif

		if(data[total_bytes - 1] == '\r') {
      data[total_bytes - 1] = '\0';
    } else {
#if DEBUG
			printf("WARNING: [get_version] Timeout while reading\n");
#endif
			data[total_bytes] = '\0';
		}

		i = total_bytes - 2;

		while(i >= 0 && data[i] != '\r')
			i--;

		version = data + i + 1;
	}
	else {
#if DEBUG
		printf("WARNING: [get_version] Timeout while reading\n");
#endif
	}

	return version;
}

} // namespace
