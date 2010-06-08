/*
 * sl_drv.c
 *
 *   Windows/Unix serial com driver
 *
 */

#ifdef __WIN32__

#include <stdio.h>
#include "windows.h"
#include "erl_driver.h"

typedef HANDLE  com_t;
#define INVALID INVALID_HANDLE_VALUE

#else

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <unistd.h>
#include <ctype.h>
#include <string.h>
#include <termios.h>
#include <fcntl.h>
#include <errno.h>
#include <signal.h>
#include <sys/param.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/uio.h>

#include "erl_driver.h"

typedef int     com_t;
#define INVALID -1

#endif

#define SL_REPLY_TYPE CBUF_FLAG_PUT_ETF

#ifdef DEBUG
// #define CBUF_DBG(buf,msg) cbuf_print((buf),(msg))
#define CBUF_DBG(buf,msg)
#define DBG(...) sl_emit_error(__FILE__,__LINE__,__VA_ARGS__)
#else
#define CBUF_DBG(buf,msg)
#define DBG(...)
#endif

#include "cbufv2.h"

static struct _rate {
    int baud;
    unsigned int speed;
} rtab[] = 

#ifdef __WIN32__
{
    {0,      0},
    {75,     BAUD_075},
    {110,    BAUD_110},
    {134,    BAUD_134_5},
    {150,    BAUD_150},
    {300,    BAUD_300},
    {600,    BAUD_600},
    {1200,   BAUD_1200},
    {1800,   BAUD_1800},
    {2400,   BAUD_2400},
    {4800,   BAUD_4800},
    {7200,   BAUD_7200},
    {9600,   BAUD_9600},
    {14400,  BAUD_14400},
    {19200,  BAUD_19200},
    {38400,  BAUD_38400},
    {56000,  BAUD_56K},
    {57600,  BAUD_57600},
    {115200, BAUD_115200},
    {128000, BAUD_128K},
    { -1,    0}
};

#else
{
    {0,       B0     },
    {50,      B50    },
    {75,      B75    },
    {110,     B110   },
    {134,     B134   },
    {150,     B150   },
    {200,     B200   },
    {300,     B300   },
    {600,     B600   },
    {1200,    B1200  },
    {1800,    B1800  },
    {2400,    B2400  },
    {4800,    B4800  },
    {9600,    B9600  },
#ifdef B19200
    {19200,   B19200 },
#elif defined(EXTA)
    {19200,   EXTA },
#endif
#ifdef B38400
    {38400,   B38400 },
#elif defined(EXTB)
    {38400,   EXTB },
#endif
#ifdef B57600
    {57600,   B57600 },
#endif
#ifdef B76800
    {76800,   B76800 },
#endif
#ifdef B115200
    {115200,  B115200 },
#endif
#ifdef B153600
    {153600,  B153600 }, 	
#endif
#ifdef B230400
    {230400,  B230400 }, 	
#endif
#ifdef B307200
    {307200,  B307200 }, 	
#endif
#ifdef B460800
    {460800,  B460800 }, 	
#endif
#ifdef B500000
    {500000,  B500000 },
#endif
#ifdef B576000
    {576000,  B576000 },
#endif
#ifdef B921600 
    {921600,  B921600 },
#endif
#ifdef B1000000
    {1000000, B1000000 },
#endif
#ifdef B1152000
    {1152000, B1152000 },
#endif
#ifdef B1500000
    {1500000, B1500000 },
#endif
#ifdef B2000000
    {2000000, B2000000 },
#endif
#ifdef B2500000
    {2500000, B2500000 },
#endif
#ifdef B3000000
    {3000000, B3000000 },
#endif
#ifdef B3500000
    {3500000, B3500000 },
#endif
#ifdef B4000000
    {4000000, B4000000 },
#endif
    { -1, B0 }
};
#endif

#define SL_CONNECT      1
#define SL_DISCONNECT   2
#define SL_OPEN         3
#define SL_CLOSE        4
#define SL_XOFF         5
#define SL_XON          6
#define SL_BREAK        7
#define SL_UPDATE       8
#define SL_GET_RATES    9
#define SL_REVERT       10

#define SL_SET_DEV      20
#define SL_SET_IBAUD    22
#define SL_SET_OBAUD    24
#define SL_SET_CSIZE    26
#define SL_SET_BUFSZ    28
#define SL_SET_BUFTM    30
#define SL_SET_STOPB    32
#define SL_SET_PARITY   34
#define SL_SET_HWFLOW   36
#define SL_SET_SWFLOW   38
#define SL_SET_XONCHAR  40
#define SL_SET_XOFFCHAR 42
#define SL_SET_MODE     46
#define SL_SET_EOLCHAR  48
#define SL_SET_EOL2CHAR 50


#define SL_GET_DEV      21
#define SL_GET_IBAUD    23
#define SL_GET_OBAUD    25
#define SL_GET_CSIZE    27
#define SL_GET_BUFSZ    29
#define SL_GET_BUFTM    31
#define SL_GET_STOPB    33
#define SL_GET_PARITY   35
#define SL_GET_HWFLOW   37
#define SL_GET_SWFLOW   39
#define SL_GET_XONCHAR  41
#define SL_GET_XOFFCHAR 43
#define SL_GET_MODE     47
#define SL_GET_EOLCHAR  49
#define SL_GET_EOL2CHAR 51

#define SL_MODE_RAW    0
#define SL_MODE_LINE   1

#define FL_IBAUD       0x0001
#define FL_OBAUD       0x0002
#define FL_CSIZE       0x0004
#define FL_STOPB       0x0008
#define FL_PARITY      0x0010
#define FL_BUFSZ       0x0020
#define FL_BUFTM       0x0040
#define FL_HWFLOW      0x0080
#define FL_SWFLOW      0x0100
#define FL_XONCHAR     0x0200
#define FL_XOFFCHAR    0x0400
#define FL_EOLCHAR     0x1000
#define FL_EOL2CHAR    0x2000
#define FL_MODE        0x8000  /* mode update */


#define REPLY_OK     1
#define REPLY_ERROR  2
#define REPLY_EVENT  3


typedef struct _sl_t
{
    ErlDrvPort port;
    com_t      com;       /* Connection handle */
    int        ilen;     /* length of input buffer */
    char*      ibuf;     /* Overlapped input buffer */
    int        i_pending; /* Input is pending */
    int        olen;     /* length of output buffer */
    char*      obuf;     /* Overlapped output buffer */
    int        o_pending; /* Output is pending */
    int        cmode;    /* current mode */
    int        ceol;     /* end of line char */
    int        ceol2;    /* end of line char 2 */
#ifdef __WIN32__
    OVERLAPPED in;       /* Overlapped input  */
    OVERLAPPED out;      /* Overlapped output */
    OVERLAPPED stat;     /* Overlapped status */
    DWORD      statm;    /* Status result */
    DCB        tio;      /* Comm parameter block */
#else
    struct termios tio;   /* The termios structure */
#endif
    char*      dev;       /* device name */
    int        flags;     /* flags FL_ */
    int        ibaud;     /* input baud rate  */
    int        obaud;     /* output baud rate  */
    int        csize;     /* 5,6,7,8 */
    int        stopb;     /* 1,2 */
    int        parity;    /* None=0,Odd=1,Even=2 */
    int        hwflow;    /* Hardware flow control */
    int        swflow;    /* Software flow control */
    int        xonchar;   /* XON character (for swflow) */
    int        xoffchar;  /* XOFF character (for swflow) */
    int        eolchar;   /* EOL character */
    int        eol2char;  /* EOL character */
    int        bufsz;     /* Number of bytes buffer */
    int        buftm;     /* Buffer fill timeout */
    int        mode;      /* (pending mode RAW | LINE) */
} sl_t;

static ErlDrvData sl_start(ErlDrvPort port, char *buf);
static void sl_stop(ErlDrvData drv_data);
static void sl_command(ErlDrvData drv_data,  char* buf, int len);
static void sl_ready_input(ErlDrvData drv_data, ErlDrvEvent event); 
static void sl_ready_output(ErlDrvData drv_data, ErlDrvEvent event);

#ifdef DEBUG
static void sl_emit_error(char* file, int line, ...);

static void sl_emit_error(char* file, int line, ...)
{
    va_list ap;
    char* fmt;

    va_start(ap, line);
    fmt = va_arg(ap, char*);

    fprintf(stderr, "%s:%d: ", file, line); 
    vfprintf(stderr, fmt, ap);
    fprintf(stderr, "\r\n");
    va_end(ap);
}
#endif

static unsigned int to_speed(int baud)
{
    int i = 0;
    int speed = 0;

    while((rtab[i].baud != -1) && (baud > rtab[i].baud))
	i++;
    if (rtab[i].baud == -1)
	speed = rtab[i-1].speed;
    else 
	speed = rtab[i].speed;
    return speed;
}

static int from_speed(unsigned int speed)
{
    int i = 0;
    int baud;

    while((rtab[i].baud != -1) && (rtab[i].speed != speed))
	i++;
    baud = rtab[i].baud;
    return baud;
}


static int get_com_state(sl_t* slp)
{
#ifdef __WIN32__
    if (!GetCommState(slp->com, &slp->tio)) {
	DBG("GetCommState: error %d", GetLastError());
	return -1;
    }
    return 0;
#else
    int res;
    if ((res = tcgetattr(slp->com, &slp->tio)) < 0) {
	DBG("tcgetattr: error %s", strerror(errno));
    }
    return res;
#endif
}

static int set_com_state(sl_t* slp)
{
    DBG("set: com_state");
#ifdef __WIN32__
    slp->tio.DCBlength = sizeof(DCB);
    if (!SetCommState(slp->com, &slp->tio)) {
	DBG("SetCommState: error %d", GetLastError());
	return -1;
    }
    return 0;
#else
    int res;
    tcflush(slp->com, TCIFLUSH);
    if ((res = tcsetattr(slp->com, TCSANOW, &slp->tio)) < 0) {
	DBG("tcsetattr: error %s", strerror(errno));
    }
    return res;
#endif
}

static void set_ibaud(sl_t* slp, int baud)
{
    DBG("set: baud %d", baud);
#ifdef __WIN32__
    slp->tio.BaudRate = to_speed(baud);
#else
    cfsetispeed(&slp->tio, to_speed(baud));
#endif
}

static void set_obaud(sl_t* slp, int baud)
{
    DBG("set: baud %d", baud);
#ifdef __WIN32__
    slp->tio.BaudRate = to_speed(baud);
#else
    cfsetospeed(&slp->tio, to_speed(baud));
#endif
}

static int get_ibaud(sl_t* slp)
{
    DBG("get: baud");
#ifdef __WIN32__
    return from_speed(slp->tio.BaudRate);
#else
    return from_speed(cfgetispeed(&slp->tio));
#endif
}

static int get_obaud(sl_t* slp)
{
    DBG("get: baud");
#ifdef __WIN32__
    return from_speed(slp->tio.BaudRate);
#else
    return from_speed(cfgetospeed(&slp->tio));
#endif
}


static void set_parity(sl_t* slp, int parity)
{
    DBG("set: parity %d", parity);
#ifdef __WIN32__
    switch(parity) {
    case 0: 
	slp->tio.fParity = FALSE;
	slp->tio.Parity = NOPARITY; 
	break;
    case 1: 
	slp->tio.fParity = TRUE;
	slp->tio.Parity = ODDPARITY; 
	break;
    case 2: 
	slp->tio.fParity = TRUE;
	slp->tio.Parity = EVENPARITY; 
	break;
    case 3: 
	slp->tio.fParity = TRUE;
	slp->tio.Parity = MARKPARITY; 
	break;
    }
#else
    switch(parity) {
    case 0: /* none */
	slp->tio.c_iflag &= ~PARMRK;
	slp->tio.c_cflag &= ~PARENB;
	break;
    case 1: /* odd */
	slp->tio.c_iflag &= ~PARMRK;
	slp->tio.c_cflag  |= PARODD;
	slp->tio.c_cflag |= PARENB;
	break;
    case 2: /* even */
	slp->tio.c_iflag &= ~PARMRK;
	slp->tio.c_cflag &= ~PARODD;
	slp->tio.c_cflag |= PARENB;
	break;
    case 3:  /* mark (FIXME) */
	slp->tio.c_iflag |= PARMRK;
	slp->tio.c_cflag |= PARENB;
	break;
    default:
	break;
    }
#endif
}


static int get_parity(sl_t* slp)
{
    DBG("get: parity");
#ifdef __WIN32__
    if (slp->tio.fParity) {
	switch (slp->tio.Parity) {
	case NOPARITY: return 0;
	case ODDPARITY: return 1;
	case EVENPARITY: return 2;
	case MARKPARITY: return 3;
	}
    }
    return 0;
#else
    if (slp->tio.c_cflag & PARENB) {
	if (slp->tio.c_iflag & PARMRK)
	    return 3;
	else if (slp->tio.c_cflag & PARODD)
	    return 1;
	else
	    return 2;
    }
    return 0;
#endif
}

static void set_stopb(sl_t* slp, int stopb)
{
    DBG("set: stopb %d", stopb);
#ifdef __WIN32__
    if (stopb == 1)
	slp->tio.StopBits = ONESTOPBIT;
    else if (stopb == 2)
	slp->tio.StopBits = TWOSTOPBITS;
#else
    if (stopb == 1)
	slp->tio.c_cflag &= ~CSTOPB;
    else if (stopb == 2)
	slp->tio.c_cflag |= CSTOPB;
#endif
}

static int get_stopb(sl_t* slp)
{
    DBG("get: stopb");
#ifdef __WIN32__
    if (slp->tio.StopBits == ONESTOPBIT)
	return 1;
    else if (slp->tio.StopBits = TWOSTOPBITS)
	return 2;
    else
	return -1;
#else
    if (slp->tio.c_cflag & CSTOPB)
	return 2;
    else
	return 1;
#endif
}

static int get_csize(sl_t* slp)
{
    DBG("get: csize");
#ifdef __WIN32__
    return slp->tio.ByteSize;
#else
    switch(slp->tio.c_cflag & CSIZE) {
    case CS5: return 5;
    case CS6: return 6;
    case CS7: return 7;
    case CS8: return 8;
    default: return -1;
    }
#endif
}

static void set_csize(sl_t* slp, int csize)
{
    DBG("set: csize %d", csize);
#ifdef __WIN32__
    slp->tio.ByteSize = csize;
#else
    slp->tio.c_cflag &= ~CSIZE;
    switch(csize) {
    case 5: slp->tio.c_cflag |= CS5; break;
    case 6: slp->tio.c_cflag |= CS6; break;
    case 7: slp->tio.c_cflag |= CS7; break;
    case 8: slp->tio.c_cflag |= CS8; break;
    default: break;
    }
#endif
}


/* set minimum buffer size */
static void set_bufsz(sl_t* slp, int bufsz)
{
    DBG("set: bufsz %d", bufsz);
#ifdef __WIN32__
    /* we have to emulate this */
#else
    slp->tio.c_cc[VMIN] = bufsz;
#endif
}    

static int get_bufsz(sl_t* slp)
{
    DBG("get: bufsz");
#ifdef __WIN32__
    return slp->bufsz; /* we have to emulate this */
#else
    return slp->tio.c_cc[VMIN];
#endif
}

/* set read timeout value */
static void set_buftm(sl_t* slp, int buftm)
{
    DBG("set: buftm %d", buftm);
#ifdef __WIN32__
    /* we have to emulate this */
#else
    slp->tio.c_cc[VTIME] = buftm;
#endif
}

/* set read timeout value */
static int get_buftm(sl_t* slp)
{
    DBG("get: buftm");
#ifdef __WIN32__
    return slp->buftm;
#else
    return slp->tio.c_cc[VTIME];
#endif
}

static void set_xonchar(sl_t* slp, int xonchar)
{
    DBG("set: xonchar %c", xonchar);
#ifdef __WIN32__
    slp->tio.XonChar = xonchar;
#else
    slp->tio.c_cc[VSTART] = xonchar;
#endif
}

static int get_xonchar(sl_t* slp)
{
    DBG("get: xonchar");
#ifdef __WIN32__
    return slp->tio.XonChar;
#else
    return slp->tio.c_cc[VSTART];
#endif
}

static void set_xoffchar(sl_t* slp, int xoffchar)
{
    DBG("set: xoffchar %c", xoffchar);
#ifdef __WIN32__
    slp->tio.XoffChar = xoffchar;
#else
    slp->tio.c_cc[VSTOP] = xoffchar;
#endif
}

static int get_xoffchar(sl_t* slp)
{
    DBG("get: xoffchar");
#ifdef __WIN32__
    return slp->tio.XoffChar;
#else
    return slp->tio.c_cc[VSTOP];
#endif
}


static void set_swflow(sl_t* slp, int on)
{
    DBG("set: swflow: %d", on);
#ifdef __WIN32__
    slp->tio.fOutX = slp->tio.fInX = on;
#else
    if (on)
	slp->tio.c_iflag |= (IXON | IXOFF);
    else
	slp->tio.c_iflag &= ~(IXON | IXOFF);
#endif
}

static int get_swflow(sl_t* slp)
{
    DBG("get: swflow");
#ifdef __WIN32__
    return slp->tio.fOutX;
#else
    switch (slp->tio.c_iflag & (IXON|IXOFF)) {
    case 0: return 0;
    case (IXON|IXOFF): return 1;
    default: return -1;
    }
#endif
}

static void set_hwflow(sl_t* slp, int on)
{
    DBG("set: hwflow: %d", on);
#ifdef __WIN32__
    slp->tio.fOutxCtsFlow = on;
#else
    if (on)
	slp->tio.c_cflag |= CRTSCTS;
    else
	slp->tio.c_cflag &= ~CRTSCTS;
#endif
}

static int get_hwflow(sl_t* slp)
{
    DBG("get: hwflow");
#ifdef __WIN32__
    return slp->tio.fOutxCtsFlow;
#else
    return (slp->tio.c_cflag & CRTSCTS) ? 1 : 0;
#endif
}

//
// Send zero bit for 4/10 second (break)
// Blocking? thread?
//
static void send_break(sl_t* slp, int duration)
{
    DBG("send_break:");
    if (slp->com != INVALID) {
#ifdef __WIN32__
    /* FIXME */
#else
	tcsendbreak(slp->com, duration);
#endif
    }
}

static void send_xon(sl_t* slp)
{
    DBG("send_xon:");
    if ((slp->com != INVALID) && (slp->swflow)) {
#ifdef __WIN32__
	/* FIXME */
#else
	tcflow(slp->com, TCION);
#endif
    }
}

static void send_xoff(sl_t* slp)
{
    DBG("send_xoff:");
    if ((slp->com != INVALID) && (slp->swflow)) {
#ifdef __WIN32__
	/* FIXME */
#else
	tcflow(slp->com, TCIOFF);
#endif
    }
}


static void slp_close(sl_t* slp)
{
    DBG("do_close: %d", slp->com);
    if (slp->com != INVALID) {
#ifdef __WIN32__
	driver_select(slp->port, (ErlDrvEvent)slp->in.hEvent,ERL_DRV_READ,0);
	driver_select(slp->port, (ErlDrvEvent)slp->out.hEvent,ERL_DRV_READ,0);
	driver_select(slp->port, (ErlDrvEvent)slp->stat.hEvent,ERL_DRV_READ,0);
	CloseHandle(slp->com);
#else
	driver_select(slp->port, (ErlDrvEvent)slp->com,
		      ERL_DRV_READ|ERL_DRV_WRITE, 0);
	/* NOTE! This will flush data not transmitted, otherwise the
	   driver will hange */
	tcflush(slp->com, TCIOFLUSH); 
	close(slp->com);
#endif
	slp->com = INVALID;
    }
    DBG("do_close: done%s", "");
}

static void slp_update(sl_t* slp)
{
    DBG("slp_update: %04X", slp->flags);

    if ((slp->flags == 0) || (slp->com == INVALID))
	return;

    if (slp->flags & FL_MODE)   slp->cmode = slp->mode;

    if (slp->flags & FL_IBAUD)	set_ibaud(slp, slp->ibaud);

    if (slp->flags & FL_OBAUD)	set_obaud(slp, slp->obaud);

    if (slp->flags & FL_CSIZE)	set_csize(slp, slp->csize);

    if (slp->flags & FL_STOPB)	set_stopb(slp, slp->stopb);

    if (slp->flags & FL_PARITY)	set_parity(slp, slp->parity);

    if (slp->flags & FL_BUFSZ)	set_bufsz(slp, slp->bufsz);

    if (slp->flags & FL_BUFTM)	set_buftm(slp, slp->buftm);

    if (slp->flags & FL_XONCHAR) set_xonchar(slp, slp->xonchar);

    if (slp->flags & FL_XOFFCHAR) set_xoffchar(slp, slp->xoffchar);

    if (slp->flags & FL_EOLCHAR) slp->ceol = slp->eolchar;
    
    if (slp->flags & FL_EOL2CHAR) slp->ceol2 = slp->eol2char;

    if (slp->flags & FL_HWFLOW) set_hwflow(slp, slp->hwflow);

    if (slp->flags & FL_SWFLOW)	set_swflow(slp, slp->swflow);

    set_com_state(slp);

    slp->flags = 0;
}

/* initiate a read operation */
static UNUSED int do_read(sl_t* slp)
{
#ifdef __WIN32__
    DWORD n;
    if ((slp->com == INVALID) || slp->i_pending)
	return -1;

    if ((slp->ibuf == NULL) || (slp->bufsz > slp->ilen)) {
	slp->ibuf = driver_realloc(slp->ibuf, slp->bufsz);
	slp->ilen = slp->bufsz;
    }    

    if (!ReadFile(slp->com, slp->ibuf, slp->bufsz, &n, &slp->in)) {
	if (GetLastError() == ERROR_IO_PENDING) {
	    slp->i_pending = 1;
	    driver_select(slp->port,(ErlDrvEvent)slp->in.hEvent,ERL_DRV_READ,1);
	    return 0;
	}
	return -1;
    }
    driver_output(slp->com, slp->ibuf, n);
    return (int) n;
#else
    int n;
    if ((slp->com == INVALID) || slp->i_pending)
	return -1;

    if ((slp->ibuf == NULL) || (slp->bufsz > slp->ilen)) {
	slp->ibuf = driver_realloc(slp->ibuf, slp->bufsz);
	slp->ilen = slp->bufsz;
    }    

    if ((n = read(slp->com, slp->ibuf, slp->bufsz)) > 0) {
	driver_output(slp->port, slp->ibuf, n);
	return n;
    }
    else if ((n < 0) && (errno == EAGAIN)) {
	driver_select(slp->port, (ErlDrvEvent)slp->com, ERL_DRV_READ, 1);
	return 0;
    }
    return n;
#endif
}


static int do_write_buf(sl_t*slp, char* buf, int len)
{
#ifdef __WIN32__
    DWORD n;
    DWORD nb = 0;

    if (!WriteFile(slp->com, buf, len, &n, &slp->out)) {
	if (GetLastError() == ERROR_IO_PENDING) {
	    slp->o_pending = 1;
	    driver_select(slp->port,(ErlDrvEvent)slp->out.hEvent,ERL_DRV_READ,1);
	    return 0;
	}
	return -1;
    }
#else
    int n;
    int nb = 0;

    if (((n = write(slp->com, buf, len)) < 0) && (errno == EAGAIN)) {
	DBG("do_write_buf: error=EAGAIN");
	nb = len;
	driver_enq(slp->port, buf, nb);
	driver_select(slp->port,(ErlDrvEvent)slp->com,ERL_DRV_WRITE, 1);
    }
    else if ((n >= 0) && (n < len)) {
	nb = len-n;
	driver_enq(slp->port, buf+n, nb);
	driver_select(slp->port,(ErlDrvEvent)slp->com,ERL_DRV_WRITE, 1);
    }
    else if (n < 0) {
	DBG("do_write_buf: error=%s", strerror(errno));
    }
#endif
    DBG("do_write_buf: %d buf=%d", n, nb);
    return n;    
}

static int do_write_more(sl_t* slp)
{
#ifdef __WIN32__
    DWORD n = 0;
    ErlIOVec vec;

    if ((n = driver_peekqv(slp->port, &vec)) > 0) {
	if (slp->olen < n) {
	    slp->obuf = driver_realloc(slp->obuf, n);
	    slp->olen = n;
	}
	driver_vec_to_buf(&vec, slp->obuf, n);
	driver_deq(slp->port, n);
	n = do_write_buf(slp, slp->obuf, n);
    }
#else
    int n = 0;
    SysIOVec* vector;
    int count;

    if ((vector = driver_peekq(slp->port, &count)) != NULL) {
	int n = writev(slp->com, (struct iovec*) vector, count);
	if (n > 0)
	    driver_deq(slp->port, n);
	if (driver_sizeq(slp->port) == 0)
	    driver_select(slp->port,(ErlDrvEvent)slp->com,ERL_DRV_WRITE, 0);
    }
#endif
    DBG("do_write_more: %d", n);
    return n;
}

/* initiate a write operation */
static int do_write_init(sl_t* slp, char* buf, int len)
{
    if (slp->com == INVALID)
	return -1;
    if (len == 0)
	return 0;

    if (slp->o_pending || (driver_sizeq(slp->port) > 0)) {
	driver_enq(slp->port, buf, len);
	return 0;
    }

#ifdef __WIN32__
    if ((slp->obuf == NULL) || (slp->olen > len)) {
	slp->obuf = driver_realloc(slp->obuf, len);
	slp->olen = len;
    }
    memcpy(slp->obuf, buf, len);
    return do_write_buf(slp, slp->obuf, len);
#else
    return do_write_buf(slp, buf, len);
#endif
}


static int do_open(sl_t* slp)
{
    slp_close(slp);

    if (slp->dev == NULL)
	return -1;
    
#ifdef __WIN32__
    slp->com = CreateFile(slp->dev,  
			  GENERIC_READ | GENERIC_WRITE, 
			  0, 
			  0, 
			  OPEN_EXISTING,
			  FILE_FLAG_OVERLAPPED,
			  0);
    if (slp->com == INVALID)
	return -1;
    slp->in.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
    slp->out.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
    slp->stat.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
    SetCommMask(slp->com, EV_RXCHAR);
    WaitCommEvent(slp->com, &slp->statm, &slp->stat);
#else
    DBG("open: dev=%s\n", slp->dev);
    slp->com = open(slp->dev, O_RDWR|O_NDELAY|O_NOCTTY);
    DBG("open: fd=%d\n", slp->com);
    if (slp->com == INVALID)
	return -1;
    tcflush(slp->com, TCOFLUSH);
    tcflush(slp->com, TCIFLUSH);
    // fcntl(slp->com, F_SETFL, fcntl(slp->com, F_GETFL) & ~O_NONBLOCK);

#endif
    if (get_com_state(slp) < 0)
	return -1;

#ifdef __WIN32__
    slp->tio.fBinary = TRUE;
#else
    memset(&slp->tio, 0, sizeof(struct termios));
    cfmakeraw(&slp->tio);
    slp->tio.c_cflag |= (CLOCAL | CREAD);

#endif

    /* setup default state if not set */
    if (!(slp->flags & FL_MODE))
	slp->mode = SL_MODE_RAW;
    if (!(slp->flags & FL_IBAUD)) { slp->ibaud = 9600; slp->flags |= FL_IBAUD; }
    if (!(slp->flags & FL_OBAUD)) { slp->obaud = 9600; slp->flags |= FL_OBAUD; }
    if (!(slp->flags & FL_PARITY)) { slp->parity = 0; slp->flags |= FL_PARITY; }
    if (!(slp->flags & FL_CSIZE))  { slp->csize = 8; slp->flags |= FL_CSIZE;  }
    if (!(slp->flags & FL_HWFLOW)) { slp->hwflow = 0; slp->flags |= FL_HWFLOW; }
    if (!(slp->flags & FL_SWFLOW)) { slp->swflow = 0; slp->flags |= FL_SWFLOW; }
    /* update pending changes */
    slp_update(slp);
    return 0;
}


static int slp_hangup(sl_t* slp)
{
    if (slp->com == INVALID)
	return -1;
#ifdef __WIN32__
    /* FIXME */
    return -1;
#else
    {
	struct termios t = slp->tio;
	cfsetispeed(&t, B0);
	cfsetospeed(&t, B0);
	return tcsetattr(slp->com, TCSAFLUSH, &t);
    }
#endif
}


static ErlDrvData sl_start(ErlDrvPort port, char *buf)
{
    sl_t *slp = (sl_t*) driver_alloc(sizeof (sl_t));
    (void) buf;

    if (!slp) 
	return (ErlDrvData) -1;
    memset(slp, 0, sizeof(sl_t));
    set_port_control_flags(port, PORT_CONTROL_FLAG_BINARY);

    slp->port = port;
    slp->com   = INVALID;
    slp->bufsz = 1;
    slp->buftm = 0;

#ifdef __WIN32__
    memset(&slp->in, 0, sizeof(slp->in));
    memset(&slp->out, 0, sizeof(slp->out));
    memset(&slp->stat, 0, sizeof(slp->stat));
#endif
    return (ErlDrvData) slp;
}


static void sl_stop(ErlDrvData data)
{
    sl_t* slp = (sl_t*) data;

    slp_close(slp);

    if (slp->ibuf != NULL)
	driver_free(slp->ibuf);
    if (slp->obuf != NULL)
	driver_free(slp->obuf);
    driver_free(slp);
}


static void sl_command(ErlDrvData data,  char* buf, int len)
{
    sl_t* slp = (sl_t*) data;

    DBG("sl_command: qsize=%d", driver_sizeq(slp->port));

    do_write_init(slp, buf, len);    
}

static void sl_ready_input(ErlDrvData data, ErlDrvEvent event)
{
    sl_t* slp = (sl_t*) data;
#ifdef __WIN32__
    DWORD n;

    if (event == slp->out.hEvent)
	sl_ready_output(data, event);
    else if (event == slp->stat.hEvent) {
	/* Manually reset the event? */
	if (do_read(slp) > 0)
	    WaitCommEvent(slp->com, &slp->statm, &slp->stat);
    }
    else if (event == slp->in.hEvent) {
	if (!GetOverlappedResult(slp->com, &slp->in, &n, FALSE)) {
	    /* how can we handle this error? */
	    return;
	}
	else {
	    driver_output(slp->port, slp->ibuf, n);
	}
    }
#else
    // FIXME: handle line mode here instead of funky cooked mode
    int n;
    (void) event;
    if ((slp->ibuf == NULL) || (slp->bufsz > slp->ilen)) {
	slp->ibuf = driver_realloc(slp->ibuf, slp->bufsz);
	slp->ilen = slp->bufsz;
    }
    if ((n = read(slp->com, slp->ibuf, slp->bufsz)) > 0) {
	DBG("sl_input_output: n=%d", n);
	driver_output(slp->port, slp->ibuf, n);
    }
#endif
}


static void sl_ready_output(ErlDrvData data, ErlDrvEvent event)
{
    sl_t* slp = (sl_t*) data;
#ifdef __WIN32__
    DWORD n;
    DBG("sl_ready_output: qsize=%d", driver_sizeq(slp->port));
    slp->o_pending = 0;
    driver_select(slp->port, (ErlDrvEvent)slp->out.hEvent,ERL_DRV_READ,0);
    if (!GetOverlappedResult(slp->com, &slp->out, &n, FALSE)) {
	/* Output error */
	return;
    }
    /* Dequeue ? */
#else
    DBG("sl_ready_output: qsize=%d", driver_sizeq(slp->port));
    (void) event;
#endif
    do_write_more(slp);
}

// Write OK
static inline void put_ok(cbuf_t* out)
{
    cbuf_put_tag_ok(out);
}

// Write ERROR,ATOM,String
static inline void put_error(cbuf_t* out, char* err)
{
    cbuf_put_tuple_begin(out, 2);
    cbuf_put_tag_error(out);
    cbuf_put_atom(out, err);
    cbuf_put_tuple_end(out, 2);
}

// Write EVENT,event-ref:32
static inline void put_i32(cbuf_t* out, int32_t v)
{
    cbuf_put_tuple_begin(out, 2);
    cbuf_put_tag_ok(out);
    cbuf_put_int32(out, v);
    cbuf_put_tuple_end(out, 2);
}

#define RETURN_OK()       do { put_ok(&reply); goto done; } while(0)
#define RETURN_ERROR(err) do { put_error(&reply,(err)); goto done; } while(0)
#define RETURN_I32(v)     do { put_i32(&reply,(v)); goto done; } while(0)


static int sl_ctl(ErlDrvData data, unsigned int cmd, char* buf, int len, 
	      char** rbuf, int rsize)
{
    sl_t* slp = (sl_t*) data;
    cbuf_t     arg;
    cbuf_t     reply;

    cbuf_init(&arg, buf, len, 0, 0);
    CBUF_DBG(&arg, "ctl_arg");
    // default data - will relloacte (as binary) on overflow
    cbuf_init(&reply, (unsigned char*) *rbuf, rsize, 0, 
	      CBUF_FLAG_BINARY | SL_REPLY_TYPE);

    cbuf_put_begin(&reply);  // put respose header

    DBG("Cmd: %02X", cmd);    
    switch(cmd) {
    case SL_SET_DEV: /* set device name */
	if (slp->dev != NULL)
	    driver_free(slp->dev);
	slp->dev = NULL;
	if (len > 0) {
	    if ((slp->dev = driver_alloc(len + 1)) == NULL)
		RETURN_ERROR("enomem");
	    memcpy(slp->dev, buf, len);
	    slp->dev[len] = '\0';
	}
	RETURN_OK();

    case SL_GET_DEV: /* get device name */
	if (!cbuf_eob(&arg) || !slp->dev)
	    RETURN_ERROR("badarg");
	else {
	    cbuf_put_tuple_begin(&reply, 2);
	    cbuf_put_tag_ok(&reply);
	    cbuf_put_string(&reply, slp->dev, strlen(slp->dev));
	    cbuf_put_tuple_end(&reply, 2);
	    goto done;
	}

    case SL_SET_IBAUD: /* set input baud rate */
	if (get_int32(&arg, &slp->ibaud) && cbuf_eob(&arg)) {
	    slp->flags |= FL_IBAUD;
	    RETURN_OK();
	}
	break;

    case SL_SET_OBAUD: /* set output baud rate */
	if (get_int32(&arg, &slp->obaud) && cbuf_eob(&arg)) {
	    slp->flags |= FL_OBAUD;
	    RETURN_OK();
	}
	break;

    case SL_GET_IBAUD: /* get baud rate */
	if (!cbuf_eob(&arg))
	    RETURN_ERROR("badarg");
	if (slp->flags & FL_IBAUD) { /* pending */
	    RETURN_I32(slp->ibaud);
	}
	if (slp->com != INVALID)
	    RETURN_I32(get_ibaud(slp));
	break;

    case SL_GET_OBAUD: /* get baud rate */
	if (!cbuf_eob(&arg))
	    RETURN_ERROR("badarg");
	if (slp->flags & FL_OBAUD) { /* pending */
	    RETURN_I32(slp->obaud);
	}
	if (slp->com != INVALID)
	    RETURN_I32(get_obaud(slp));
	break;

    case SL_SET_CSIZE:
	if (get_int32(&arg, &slp->csize) && cbuf_eob(&arg)) {
	    slp->flags |= FL_CSIZE;
	    RETURN_OK();
	}
	break;

    case SL_GET_CSIZE:
	if (!cbuf_eob(&arg))
	    RETURN_ERROR("badarg");
	if (slp->flags & FL_CSIZE) /* pending */
	    RETURN_I32(slp->csize);
	else if (slp->com != INVALID)
	    RETURN_I32(get_csize(slp));
	break;

    case SL_SET_BUFSZ:
	if (get_int32(&arg, &slp->bufsz) && cbuf_eob(&arg)) {
	    slp->flags |= FL_BUFSZ;
	    if (slp->bufsz < 0) slp->bufsz = 1;
	    else if (slp->bufsz > 255) slp->bufsz = 255;
	}
	RETURN_OK();

    case SL_GET_BUFSZ:
	if (!cbuf_eob(&arg))
	    RETURN_ERROR("badarg");
	if (slp->flags & FL_BUFSZ)
	    RETURN_I32(slp->bufsz);
	else if (slp->com != INVALID)
	    RETURN_I32(get_bufsz(slp));
	break;

    case SL_SET_BUFTM:
	if (get_int32(&arg, &slp->buftm) && cbuf_eob(&arg)) {
	    slp->flags |= FL_BUFTM;
	    RETURN_OK();
	}
	break;

    case SL_GET_BUFTM:
	if (!cbuf_eob(&arg))
	    RETURN_ERROR("badarg");
	if (slp->flags & FL_BUFTM)
	    RETURN_I32(slp->buftm);
	else if (slp->com != INVALID)
	    RETURN_I32(get_buftm(slp));
	break;

    case SL_SET_STOPB:
	if (get_int32(&arg, &slp->stopb) && cbuf_eob(&arg)) {
	    slp->flags |= FL_STOPB;
	    RETURN_OK();
	}
	break;

    case SL_GET_STOPB:
	if (!cbuf_eob(&arg))
	    RETURN_ERROR("badarg");
	if (slp->flags & FL_STOPB)
	    RETURN_I32(slp->stopb);
	else if (slp->com != INVALID)
	    RETURN_I32(get_stopb(slp));
	break;

    case SL_SET_PARITY:
	if (get_int32(&arg, &slp->parity) && cbuf_eob(&arg)) {
	    slp->flags |= FL_PARITY;
	    RETURN_OK();
	}
	break;


    case SL_GET_PARITY:
	if (!cbuf_eob(&arg))
	    RETURN_ERROR("badarg");
	if (slp->flags & FL_PARITY)
	    RETURN_I32(slp->parity);
	else if (slp->com != INVALID) 
	    RETURN_I32(get_parity(slp));
	break;

    case SL_SET_HWFLOW:
	if (get_int32(&arg, &slp->hwflow) && cbuf_eob(&arg)) {
	    slp->flags |= FL_HWFLOW;
	    RETURN_OK();
	}
	break;

    case SL_GET_HWFLOW:
	if (!cbuf_eob(&arg))
	    RETURN_ERROR("badarg");
	if (slp->flags & FL_HWFLOW)
	    RETURN_I32(slp->hwflow);
	else if (slp->com != INVALID)
	    RETURN_I32(get_hwflow(slp));
	break;


    case SL_SET_SWFLOW:
	if (get_int32(&arg, &slp->swflow) && cbuf_eob(&arg)) {
	    slp->flags |= FL_SWFLOW;
	    RETURN_OK();
	}
	break;

    case SL_GET_SWFLOW:
	if (!cbuf_eob(&arg))
	    RETURN_ERROR("badarg");
	if (slp->flags & FL_SWFLOW)
	    RETURN_I32(slp->swflow);
	else if (slp->com != INVALID)
	    RETURN_I32(get_swflow(slp));
	break;

    case SL_SET_XONCHAR:
	if (get_int32(&arg, &slp->xonchar) && cbuf_eob(&arg)) {
	    slp->flags |= FL_XONCHAR;
	    RETURN_OK();
	}
	break;


    case SL_GET_XONCHAR:
	if (!cbuf_eob(&arg))
	    RETURN_ERROR("badarg");
	if (slp->flags & FL_XONCHAR)
	    RETURN_I32(slp->xonchar);
	else if (slp->com != INVALID)
	    RETURN_I32(get_xonchar(slp));
	break;
	

    case SL_SET_XOFFCHAR:
	if (get_int32(&arg, &slp->xoffchar) && cbuf_eob(&arg)) {
	    slp->flags |= FL_XOFFCHAR;
	    RETURN_OK();
	}
	break;

    case SL_GET_XOFFCHAR:
	if (!cbuf_eob(&arg)) RETURN_ERROR("badarg");
	if (slp->flags & FL_XOFFCHAR)
	    RETURN_I32(slp->xoffchar);
	else if (slp->com != INVALID)
	    RETURN_I32(get_xoffchar(slp));
	break;

    case SL_SET_EOLCHAR:
	if (get_int32(&arg, &slp->eolchar) && cbuf_eob(&arg)) {
	    slp->flags |= FL_EOLCHAR;
	    RETURN_OK();
	}
	break;

    case SL_GET_EOLCHAR:
	if (!cbuf_eob(&arg)) RETURN_ERROR("badarg");
	if (slp->flags & FL_EOLCHAR)  RETURN_I32(slp->eolchar);
	else if (slp->com != INVALID) RETURN_I32(slp->ceol);
	break;

    case SL_SET_EOL2CHAR:
	if (get_int32(&arg, &slp->eol2char) && cbuf_eob(&arg)) {
	    slp->flags |= FL_EOL2CHAR;
	    RETURN_OK();
	}
	break;

    case SL_GET_EOL2CHAR:
	if (!cbuf_eob(&arg)) RETURN_ERROR("badarg");
	if (slp->flags & FL_EOL2CHAR)
	    RETURN_I32(slp->eol2char);
	else if (slp->com != INVALID)
	    RETURN_I32(slp->ceol2);
	break;

    case SL_SET_MODE: {
	int mode;
	if (get_int32(&arg, &mode) && cbuf_eob(&arg)) {
	    if (mode != slp->mode) {
		if (mode == SL_MODE_RAW)
		    slp->mode = SL_MODE_RAW;
		else if (mode == SL_MODE_LINE)
		    slp->mode = SL_MODE_LINE;
		else
		    RETURN_ERROR("badarg");
		slp->flags |= FL_MODE;
	    }
	    RETURN_OK();
	}
	break;
    }
	
    case SL_GET_MODE:
	if (!cbuf_eob(&arg)) RETURN_ERROR("badarg");
	if (slp->flags & FL_MODE)
	    RETURN_I32(slp->mode);
	else if (slp->com != INVALID)
	    RETURN_I32(slp->cmode);
	break;
	
    case SL_CONNECT:
    case SL_OPEN:
	if (slp->dev == NULL) /* MUST set device first */
	    RETURN_ERROR("badarg");
	if (do_open(slp) < 0)
	    RETURN_ERROR("eaccess");
#ifdef __WIN32__
	driver_select(slp->port,(ErlDrvEvent)slp->stat.hEvent,DO_READ,1);
#else
	driver_select(slp->port,(ErlDrvEvent)slp->com, DO_READ, 1);
#endif
	RETURN_OK();

    case SL_DISCONNECT:
	slp_hangup(slp);
	/* fall through */
    case SL_CLOSE:
	slp_close(slp);
	RETURN_OK();

    case SL_XOFF:
	send_xoff(slp);
	RETURN_OK();

    case SL_XON:
	send_xon(slp);
	RETURN_OK();

    case SL_BREAK:
	send_break(slp, 0);
	RETURN_OK();
	
    case SL_UPDATE:
	slp_update(slp);
	RETURN_OK();

    case SL_REVERT:
	slp->flags = 0;
	RETURN_OK();
	
    case SL_GET_RATES: {
	int i;
	size_t n = 0;

	for (i = 0; rtab[i].baud != -1; i++)
	    n++;
	cbuf_put_tuple_begin(&reply, 2);
	cbuf_put_tag_ok(&reply);
	cbuf_put_list_begin(&reply, n);

	for (i=0; rtab[i].baud != -1; i++)
	    cbuf_put_int32(&reply, rtab[i].baud);
	cbuf_put_list_end(&reply, n);
	cbuf_put_tuple_end(&reply, 2);
	goto done;
    }
    default:
	break;
    }

done:
    cbuf_put_end(&reply);  // put response footer
    CBUF_DBG(&reply, "ctl_reply");
    if (reply.vlen == 1) {
	if (reply.v[0].bp) {
	    cbuf_trim(&reply);
	    *rbuf = (char*) reply.v[0].bp;
	}
	else
	    *rbuf = (char*) reply.v[0].base;
	return cbuf_seg_used(&reply);
    }
    else {
	// FIXME return a event handle and do a send_term
	// example: get_program_info(Program, binaries) 
	fprintf(stderr, "FIXME:vector reply!\r\n");
	return 0;
    }
}

ErlDrvEntry  sl_drv = {
    NULL,                     // sl_drv_init,
    sl_start,
    sl_stop,
    sl_command,
    sl_ready_input,           // ready_input
    sl_ready_output,          // ready_output
    "sl_drv",
    NULL,                     // sl_finish,
    NULL,                     // handle 
    sl_ctl,
    NULL,                     // sl_timeout,
    NULL,                     // sl_commandv,
    NULL,                     // sl_async
    NULL,                     // sl_flush
    NULL,                     // sl_call
    NULL,                     // sl_event
    ERL_DRV_EXTENDED_MARKER,
    ERL_DRV_EXTENDED_MAJOR_VERSION,
    ERL_DRV_EXTENDED_MINOR_VERSION,
    ERL_DRV_FLAG_USE_PORT_LOCKING,  /* We really want NO locking! */
    NULL,                           /* handle2 */
    NULL,                           /* process_exit */
    NULL
};
	
DRIVER_INIT(sl_drv)
{
    return &sl_drv;
}
