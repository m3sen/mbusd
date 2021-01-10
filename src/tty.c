/*
 * OpenMODBUS/TCP to RS-232/485 MODBUS RTU gateway
 *
 * tty.c - terminal I/O related procedures
 *
 * Copyright (c) 2002-2003, 2013, Victor Antonovich (v.antonovich@gmail.com)
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * - Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * - Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: tty.c,v 1.4 2015/02/25 10:33:58 kapyar Exp $
 */

#include "tty.h"

#define IN  0
#define OUT 1

extern cfg_t cfg;

static int tty_break;

static int GPIOExport(int pin);
static int GPIOUnexport(int pin);
static int GPIODirection(int pin, int dir);
static int GPIORead(int pin);
static int GPIOWrite(int pin, int value);

/*
 * Flag signal SIG
 */
void
tty_sighup(void)
{
  tty_break = TRUE;
  return;
}

/*
 * Init serial link parameters
 */
void
tty_init(ttydata_t *mod)
{
  mod->fd = -1;
  mod->port = cfg.ttyport;
  mod->speed = cfg.ttyspeed;
  mod->bpc = DEFAULT_BITS_PER_CHAR;
  if (toupper(cfg.ttymode[1]) != 'N')
  {
    mod->bpc++;
  }
  if (cfg.ttymode[2] == '2')
  {
    mod->bpc++;
  }
#ifdef TRXCTL
  mod->trxcntl = cfg.trxcntl;
  switch (cfg.trxcntl) {
  case TRX_SYSFS_1:
  case TRX_SYSFS_0:
      GPIOExport(cfg.tx_enable_pin);
      GPIODirection(cfg.tx_enable_pin, OUT);
      break;
  case TRX_ADDC:
  case TRX_RTS:
  default:
      break;
  }

#endif
}

#ifdef HAVE_LIBUTIL
char *tty_get_name(char *ttyfullname);

char *
tty_get_name(char *ttyfullname)
{
  static char ttynamebuf[INTBUFSIZE + 1];
  char *ttyname = ttynamebuf, *ttynameptr = ttyname;
  strncpy(ttynamebuf, ttyfullname, INTBUFSIZE);
  for (ttynameptr = strtok(ttynamebuf, "/");
       ttynameptr;
       ttynameptr = strtok(NULL, "/"))
  {
    ttyname = ttynameptr;
  }
  return ttyname;
}
#endif


/*
 * Opening serial link whith parameters in MOD
 */
int
tty_open(ttydata_t *mod)
{
#ifdef HAVE_LIBUTIL
  int buferr, uuerr;
  char *ttyname = tty_get_name(mod->port);
#endif
  if (mod->fd > 0)
    return RC_AOPEN;        /* if already open... */
  tty_break = FALSE;
#ifdef HAVE_LIBUTIL
  if ((uuerr = uu_lock(ttyname)) != UU_LOCK_OK)
  {
    buferr = errno;
#ifdef LOG
    logw(0, "uu_lock(): can't lock tty device %s (%s)",
        ttyname, uu_lockerr(uuerr));
#endif
    errno = buferr;
    return RC_ERR;
  }
#endif
#ifdef LOG
  logw(2, "tty: trying to open %s (speed %d mode %s)", mod->port, mod->speed, cfg.ttymode);
#endif
  mod->fd = open(mod->port, O_RDWR | O_NONBLOCK | O_NOCTTY);
  if (mod->fd < 0)
    return RC_ERR;          /* attempt failed */
  return tty_set_attr(mod);
}

/*
 * Setting up tty device MOD attributes
 */
int
tty_set_attr(ttydata_t *mod)
{
  int flag;
  speed_t tspeed = tty_transpeed(mod->speed);

  memset(&mod->savedtios, 0, sizeof(struct termios));
  if (tcgetattr(mod->fd, &mod->savedtios))
    return RC_ERR;
  memcpy(&mod->tios, &mod->savedtios, sizeof(mod->tios));
#ifdef HAVE_CFMAKERAW
  cfmakeraw(&mod->tios);
#else
  mod->tios.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
  mod->tios.c_oflag &= ~OPOST;
  mod->tios.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
#endif
  mod->tios.c_cflag |= (CREAD | CLOCAL);
  mod->tios.c_cflag &= ~(CSIZE | CSTOPB | PARENB | PARODD | CRTSCTS);
  switch (cfg.ttymode[0])
  {
    case '5':
      mod->tios.c_cflag |= CS5;
      break;
    case '6':
      mod->tios.c_cflag |= CS6;
      break;
    case '7':
      mod->tios.c_cflag |= CS7;
      break;
    case '8':
      mod->tios.c_cflag |= CS8;
      break;
  }
  switch (toupper(cfg.ttymode[1]))
  {
    case 'E':
      mod->tios.c_cflag |= PARENB;
      break;
    case 'O':
      mod->tios.c_cflag |= (PARENB | PARODD);
      break;
  }
  if (cfg.ttymode[2] == '2')
  {
      mod->tios.c_cflag |= CSTOPB;
  }
  mod->tios.c_cc[VTIME] = 0;
  mod->tios.c_cc[VMIN] = 1;
#ifdef HAVE_CFSETSPEED
  cfsetspeed(&mod->tios, tspeed);
#else
  cfsetispeed(&mod->tios, tspeed);
  cfsetospeed(&mod->tios, tspeed);
#endif
  if (tcsetattr(mod->fd, TCSANOW, &mod->tios))
    return RC_ERR;
  tcflush(mod->fd, TCIOFLUSH);
#ifdef  TRXCTL
  tty_clr_rts(mod->fd);
#endif
  flag = fcntl(mod->fd, F_GETFL, 0);
  if (flag < 0)
    return RC_ERR;
  return fcntl(mod->fd, F_SETFL, flag | O_NONBLOCK);
}

/*
 * Translate integer SPEED value to speed_t constant
 */
speed_t
tty_transpeed(int speed)
{
  speed_t tspeed;
  switch (speed)
  {
#if defined(B50)
  case 50:
    tspeed = B50;
    break;
#endif
#if defined(B75)
  case 75:
    tspeed = B75;
    break;
#endif
#if defined(B110)
  case 110:
    tspeed = B110;
    break;
#endif
#if defined(B134)
  case 134:
    tspeed = B134;
    break;
#endif
#if defined(B150)
  case 150:
    tspeed = B150;
    break;
#endif
#if defined(B200)
  case 200:
    tspeed = B200;
    break;
#endif
#if defined(B300)
  case 300:
    tspeed = B300;
    break;
#endif
#if defined(B600)
  case 600:
    tspeed = B600;
    break;
#endif
#if defined(B1200)
  case 1200:
    tspeed = B1200;
    break;
#endif
#if defined(B1800)
  case 1800:
    tspeed = B1800;
    break;
#endif
#if defined(B2400)
  case 2400:
    tspeed = B2400;
    break;
#endif
#if defined(B4800)
  case 4800:
    tspeed = B4800;
    break;
#endif
#if defined(B7200)
  case 7200:
    tspeed = B7200;
    break;
#endif
#if defined(B9600)
  case 9600:
    tspeed = B9600;
    break;
#endif
#if defined(B12000)
  case 12000:
    tspeed = B12000;
    break;
#endif
#if defined(B14400)
  case 14400:
    tspeed = B14400;
    break;
#endif
#if defined(B19200)
  case 19200:
    tspeed = B19200;
    break;
#elif defined(EXTA)
  case 19200:
    tspeed = EXTA;
    break;
#endif
#if defined(B38400)
  case 38400:
    tspeed = B38400;
    break;
#elif defined(EXTB)
  case 38400:
    tspeed = EXTB;
    break;
#endif
#if defined(B57600)
  case 57600:
    tspeed = B57600;
    break;
#endif
#if defined(B115200)
  case 115200:
    tspeed = B115200;
    break;
#endif
#if defined(B230400)
  case 230400:
    tspeed = B230400;
    break;
#endif
#if defined(B460800)
  case 460800:
    tspeed = B460800;
    break;
#endif
#if defined(B500000)
  case 500000:
    tspeed = B500000;
    break;
#endif
#if defined(B576000)
  case 576000:
    tspeed = B576000;
    break;
#endif
#if defined(B921600)
  case 921600:
    tspeed = B921600;
    break;
#endif
#if defined(B1000000)
  case 1000000:
    tspeed = B1000000;
    break;
#endif
#if defined(B1152000)
  case 1152000:
    tspeed = B1152000;
    break;
#endif
#if defined(B1500000)
  case 1500000:
    tspeed = B1500000;
    break;
#endif
#if defined(B2000000)
  case 2000000:
    tspeed = B2000000;
    break;
#endif
#if defined(B2500000)
  case 2500000:
    tspeed = B2500000;
    break;
#endif
#if defined(B3000000)
  case 3000000:
    tspeed = B3000000;
    break;
#endif
#if defined(B3500000)
  case 3500000:
    tspeed = B3500000;
    break;
#endif
#if defined(B4000000)
  case 4000000:
    tspeed = B4000000;
    break;
#endif
  default:
#ifdef LOG
    logw(0, "tty: unsupported speed: %d", speed);
#endif
    exit (-1);
  }
  return tspeed;
}

/*
 * Prepare tty device MOD to closing
 */
int
tty_cooked(ttydata_t *mod)
{
  if (!isatty(mod->fd))
    return RC_ERR;
  if (tcsetattr(mod->fd, TCSAFLUSH, &mod->savedtios))
    return RC_ERR;
  return RC_OK;
}

/*
 * Closing tty device MOD
 */
int
tty_close(ttydata_t *mod)
{
#ifdef HAVE_LIBUTIL
  int buferr;
  char *ttyname = tty_get_name(mod->port);
#endif

#ifdef TRXCTL
  switch (cfg.trxcntl) {
  case TRX_SYSFS_1:
  case TRX_SYSFS_0:
      GPIOUnexport(cfg.tx_enable_pin);
      break;
  case TRX_ADDC:
  case TRX_RTS:
  default:
      break;
  }

#endif

  if (mod->fd < 0)
    return RC_ACLOSE;       /* already closed */
  tty_cooked(mod);
#ifdef HAVE_LIBUTIL
  if (close(mod->fd))
    return RC_ERR;
  if (uu_unlock(ttyname))
  {
    buferr = errno;
#ifdef LOG
    logw(0, "uu_lock(): can't unlock tty device %s",
        ttyname);
#endif
    errno = buferr;
    return RC_ERR;
  }
  return RC_OK;
#else
  return close(mod->fd);
#endif
}

#ifdef  TRXCTL
void sysfs_gpio_set(char *filename, char *value) {
	int fd;

	fd = open(filename, O_WRONLY);
	/* write first byte. Should only be 0 or 1 */
	write(fd, value, 1);
	close(fd);

	logw(9, "tty: sysfs_gpio_set(%s,%s)\n",filename,value);

}

/* Set RTS line to active state */
void
tty_set_rts(int fd)
{
	if ( TRX_RTS == cfg.trxcntl ) {
		int mstat = TIOCM_RTS;
		ioctl(fd, TIOCMBIS, &mstat);
	} else if ( TRX_SYSFS_1 == cfg.trxcntl) {
        GPIOWrite(cfg.tx_enable_pin, 1);
		//sysfs_gpio_set(cfg.trxcntl_file,"1");
	} else if ( TRX_SYSFS_0 == cfg.trxcntl) {
        GPIOWrite(cfg.tx_enable_pin, 0);
		//sysfs_gpio_set(cfg.trxcntl_file,"0");
	}
}

/* Set RTS line to passive state */
void
tty_clr_rts(int fd)
{
	if ( TRX_RTS == cfg.trxcntl ) {
		int mstat = TIOCM_RTS;
		ioctl(fd, TIOCMBIC, &mstat);
	} else if ( TRX_SYSFS_1 == cfg.trxcntl) {
        GPIOWrite(cfg.tx_enable_pin, 0);
		//sysfs_gpio_set(cfg.trxcntl_file,"0");
	} else if ( TRX_SYSFS_0 == cfg.trxcntl) {
        GPIOWrite(cfg.tx_enable_pin, 1);
		//sysfs_gpio_set(cfg.trxcntl_file,"1");
	}
}
#endif

/*
 * Delay for USEC microsecs
 */
void
tty_delay(int usec)
{
  struct timeval tv, ttv;
  long ts;
  gettimeofday(&tv, NULL);
  do
  {
    (void)gettimeofday(&ttv, NULL);
    ts = 1000000l * (ttv.tv_sec - tv.tv_sec) + (ttv.tv_usec - tv.tv_usec);
  } while (ts < usec && !tty_break);
}

static int
GPIOExport(int pin)
{
#define BUFFER_MAX 3
	char buffer[BUFFER_MAX];
	int bytes_written;
	int fd;

	fd = open("/sys/class/gpio/export", O_WRONLY);
	if (-1 == fd) {
		logw(2, "Failed to open export for writing!\n");
		return(-1);
	}

	bytes_written = snprintf(buffer, BUFFER_MAX, "%d", pin);
	write(fd, buffer, bytes_written);
	close(fd);
	return(0);
}

static int
GPIOUnexport(int pin)
{
	char buffer[BUFFER_MAX];
	int bytes_written;
	int fd;

	fd = open("/sys/class/gpio/unexport", O_WRONLY);
	if (-1 == fd) {
		logw(2, "Failed to open unexport for writing!\n");
		return(-1);
	}

	bytes_written = snprintf(buffer, BUFFER_MAX, "%d", pin);
	write(fd, buffer, bytes_written);
	close(fd);
	return(0);
}

static int
GPIODirection(int pin, int dir)
{
	static const char s_directions_str[] = "in\0out";

#define DIRECTION_MAX 35
	char path[DIRECTION_MAX];
	int fd;

	snprintf(path, DIRECTION_MAX, "/sys/class/gpio/gpio%d/direction", pin);
	fd = open(path, O_WRONLY);
	if (-1 == fd) {
		logw(2, "Failed to open gpio direction for writing!\n");
		return(-1);
	}

	if (-1 == write(fd, &s_directions_str[IN == dir ? 0 : 3], IN == dir ? 2 : 3)) {
		logw(2, "Failed to set direction!\n");
		return(-1);
	}

	close(fd);
	return(0);
}

static int
GPIORead(int pin)
{
#define VALUE_MAX 30
	char path[VALUE_MAX];
	char value_str[3];
	int fd;

	snprintf(path, VALUE_MAX, "/sys/class/gpio/gpio%d/value", pin);
	fd = open(path, O_RDONLY);
	if (-1 == fd) {
		logw(2, "Failed to open gpio value for reading!\n");
		return(-1);
	}

	if (-1 == read(fd, value_str, 3)) {
		logw(2, "Failed to read value!\n");
		return(-1);
	}

	close(fd);

	return(atoi(value_str));
}

static int
GPIOWrite(int pin, int value)
{
	static const char s_values_str[] = "01";

	char path[VALUE_MAX];
	int fd;

	snprintf(path, VALUE_MAX, "/sys/class/gpio/gpio%d/value", pin);
	fd = open(path, O_WRONLY);
	if (-1 == fd) {
		logw(2, "Failed to open gpio value for writing!\n");
		return(-1);
	}

	if (1 != write(fd, &s_values_str[LOW == value ? 0 : 1], 1)) {
		logw(2, "Failed to write value!\n");
		return(-1);
	}

	close(fd);
	return(0);
}