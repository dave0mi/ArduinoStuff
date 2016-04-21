

#if (!defined(WIN32) && !defined(WIN64)) || defined(CYGWIN)
#include <unistd.h> // read, sleep, etc...
#endif

#include <stdio.h>
#include <stdlib.h>
#include <termios.h> // for setting up serial.
#include <fcntl.h>   // O_RDWR, etc.
#include <sys/ioctl.h> //  FIONREAD
#include <errno.h>
#include <time.h>

static int gVerbose = 0;

void
mssleep(unsigned long ms) {

	struct timespec nsts;
	struct timespec rem;
	nsts.tv_sec  = 0;
	nsts.tv_nsec = 1000000*ms;

	if (nanosleep(&nsts,&rem)== -1)
	{
		fprintf(stderr,"Restart nanosleep %lu", rem.tv_nsec);
		nsts=rem;
		nanosleep(&nsts,&rem);
	}
}

/////////////////////////////
/// return a file descriptor.
/////////////////////////////
int
open_serial_port(const char* pathbuf)
{
	// we'd like to disable flow control, 
	// check out ...
	// http://www.easysw.com/~mike/serial/serial.html#2_5_2
	// there is also an ioctl ...

#if 0 // not on Mac OS.
#include <rtems/libio.h>
rtems_status_code rtems_termios_bufsize (
    int cbufsize,     /* cooked buffer size */
    int raw_input,    /* raw input buffer size */
    int raw_output    /* raw output buffer size */
);
#endif
	int fd = ::open(pathbuf, O_RDWR| O_NOCTTY | O_NDELAY);
	//int fd = ::open(pathbuf, O_RDWR| O_NOCTTY );

	struct termios options;

	if (fd == -1) {
		if (gVerbose) fprintf(stderr,"Can't open '%s'", pathbuf);
		return -1;
	}

	//fcntl(fd, F_SETFL, 0);
	// termios style - also IOCTL version available?

	// modify existing or start from scratch?
	tcgetattr(fd, &options);
	// memset(&options,0, sizeof(options));

	//int baud = B19200;
	int baud = B115200;
	//int baud = B230400;
	//int baud = B460800; // MUST MATCH ARDUINO PROGRAM.

	int speed;

	speed=cfsetispeed(&options, baud); // pick the baud to match arduino.
	fprintf(stderr,"ispeed return %d", speed);

	speed=cfsetospeed(&options, baud);
	fprintf(stderr,"ospeed return %d", speed);

	options.c_cflag |= (CLOCAL | CREAD); //

	// no parity, 8 data bits, 1 stop bit.
	options.c_cflag &= ~PARENB;
	options.c_cflag &= ~CSTOPB;
	options.c_cflag &= ~CSIZE;
	options.c_cflag |= CS8;

	// no hardware flow control.
#ifdef CRTSCTS
	options.c_cflag &= ~CRTSCTS;
#else
	options.c_cflag &= ~CNEW_RTSCTS;
#endif

	// RAW input.
   	options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

	// parity - can ignore if parity if OFF (~PARENB above)
	// options.c_iflag

	// turn off flow control for input
    	options.c_iflag &= ~(IXON | IXOFF | IXANY);

	// turn off flow control for output
  	options.c_oflag &= ~OPOST;

// fprintf(stderr,"VMIN %d, VTIME %d", options.c_cc[VMIN], options.c_cc[VTIME]);

	// set timeouts
	options.c_cc[VMIN]  = 1; // min number of bytes to wait for read to return.
	options.c_cc[VTIME] = 1; // typical time between bytes.

	// update the port with these options.
	tcsetattr(fd, TCSANOW, &options); // don't buffer.

#if 0
// may need to be called after setting port raw above.
// obsolete???
{
	struct serial_struct ss;
	if (ioctl(fd, TIOCGSERIAL, &ss)!=0) {
  		fprintf(stderr,"ioctl TIOCBSERIAL %d",errno);
	} else {
		fprintf(stderr,"ss.foo");
	}
}
#endif

	return fd;
}


#if 0 // don't need for this application.
char 
readByteWithTimeout(int fd)
{
#if 1
  fd_set rset,wset,eset;
  FD_ZERO(&rset); FD_ZERO(&wset); FD_ZERO(&eset);
  FD_SET(fd, &rset);

  struct timeval timeout;

  timeout.tv_sec  = 2;
  timeout.tv_usec = 0; // 2s

  if (select(fd+1, &rset, &wset, &eset, &timeout) == 0)
  {
  	if (gVerbose) fprintf(stderr,"select timed out");
	return 0;
  }
#endif

  char c;
  int ret;
  if ((ret = ::read(fd, &c, 1)) != 1)
  {
  	if (gVerbose) fprintf(stderr,"read byte error %d", ret);
	return 0;
  }

  return c;
}
#endif



#if 0 // don't need for this application.
bool 
readBytesWithTimeout(int fd, char* localbuf, int n)
{
	int haven,startAt=0,need;

	while (ioctl(fd, FIONREAD, &haven) == 0) {

		need = n-startAt;

		if (haven >= need) return (::read(fd, localbuf+startAt, need)==need);

		if (::read(fd, localbuf+startAt, haven)!=haven) return false;

		startAt += haven;

		mssleep(1); // sleep for 1ms about 10 bytes at 11520
	}

	return false;

  	//if (gVerbose) fprintf(stderr,"serial (info): have %d, wanted %d", haven, n);
	// at 11520 we need 86us to transmit a byte.
	// but the real question is why would we ever need to
	// block if the device should have already sent us data?
	// int errs = 0;
	//for (int i = startAt; i < n; i++)
	//	localbuf[i] = readByteWithTimeout(fd);
//
//	return true;
	//return (errs==0);
}
#endif


int
main(int ac, char* av[]) {

	char* portName = (char*)"/dev/ttyACM0";
	int fd,ret,ok;
	int totalSleeps = 0;
	char c;

	if (ac>1) portName = av[1];

	if ((fd = open_serial_port(portName))==-1) {
		fprintf(stderr, "Can't open '%s'\n", portName);
		exit(1);
	}

	for (ok=1; ok;) {
	
		ret=read(fd,&c,1);

		if (ret==1) printf("%c",c);
		else {
			if (ret==-1 && errno==11) {
				mssleep(1);
				totalSleeps++;
			}
		}
	}

	fprintf(stderr, "read()==%d, errno %d\n", ret, errno);
	fprintf(stderr, "totalSleeps==%d\n", totalSleeps);

	close(fd);
}


