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


#include "eapi_drv.h"
#include "sl_drv.h"

typedef int     com_t;
#define INVALID -1

#endif


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

static void sl_ready_input(ErlDrvData drv_data, ErlDrvEvent event); 
static void sl_ready_output(ErlDrvData drv_data, ErlDrvEvent event);

static ErlDrvEntry sl_drv_entry;


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
	EAPI_DRV_DBG("GetCommState: error %d", GetLastError());
	return -1;
    }
    return 0;
#else
    int res;
    if ((res = tcgetattr(slp->com, &slp->tio)) < 0) {
	EAPI_DRV_DBG("tcgetattr: error %s", strerror(errno));
    }
    return res;
#endif
}

static int set_com_state(sl_t* slp)
{
    EAPI_DRV_DBG("set: com_state");
#ifdef __WIN32__
    slp->tio.DCBlength = sizeof(DCB);
    if (!SetCommState(slp->com, &slp->tio)) {
	EAPI_DRV_DBG("SetCommState: error %d", GetLastError());
	return -1;
    }
    return 0;
#else
    int res;
    tcflush(slp->com, TCIFLUSH);
    if ((res = tcsetattr(slp->com, TCSANOW, &slp->tio)) < 0) {
	EAPI_DRV_DBG("tcsetattr: error %s", strerror(errno));
    }
    return res;
#endif
}

static void set_ibaud(sl_t* slp, int baud)
{
    EAPI_DRV_DBG("set: baud %d", baud);
#ifdef __WIN32__
    slp->tio.BaudRate = to_speed(baud);
#else
    cfsetispeed(&slp->tio, to_speed(baud));
#endif
}

static void set_obaud(sl_t* slp, int baud)
{
    EAPI_DRV_DBG("set: baud %d", baud);
#ifdef __WIN32__
    slp->tio.BaudRate = to_speed(baud);
#else
    cfsetospeed(&slp->tio, to_speed(baud));
#endif
}

static int get_ibaud(sl_t* slp)
{
    EAPI_DRV_DBG("get: baud");
#ifdef __WIN32__
    return from_speed(slp->tio.BaudRate);
#else
    return from_speed(cfgetispeed(&slp->tio));
#endif
}

static int get_obaud(sl_t* slp)
{
    EAPI_DRV_DBG("get: baud");
#ifdef __WIN32__
    return from_speed(slp->tio.BaudRate);
#else
    return from_speed(cfgetospeed(&slp->tio));
#endif
}


static void set_parity(sl_t* slp, int parity)
{
    EAPI_DRV_DBG("set: parity %d", parity);
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
    EAPI_DRV_DBG("get: parity");
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
    EAPI_DRV_DBG("set: stopb %d", stopb);
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
    EAPI_DRV_DBG("get: stopb");
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
    EAPI_DRV_DBG("get: csize");
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
    EAPI_DRV_DBG("set: csize %d", csize);
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
    EAPI_DRV_DBG("set: bufsz %d", bufsz);
#ifdef __WIN32__
    /* we have to emulate this */
#else
    slp->tio.c_cc[VMIN] = bufsz;
#endif
}    

static int get_bufsz(sl_t* slp)
{
    EAPI_DRV_DBG("get: bufsz");
#ifdef __WIN32__
    return slp->bufsz; /* we have to emulate this */
#else
    return slp->tio.c_cc[VMIN];
#endif
}

/* set read timeout value */
static void set_buftm(sl_t* slp, int buftm)
{
    EAPI_DRV_DBG("set: buftm %d", buftm);
#ifdef __WIN32__
    /* we have to emulate this */
#else
    slp->tio.c_cc[VTIME] = buftm;
#endif
}

/* set read timeout value */
static int get_buftm(sl_t* slp)
{
    EAPI_DRV_DBG("get: buftm");
#ifdef __WIN32__
    return slp->buftm;
#else
    return slp->tio.c_cc[VTIME];
#endif
}

static void set_xonchar(sl_t* slp, int xonchar)
{
    EAPI_DRV_DBG("set: xonchar %c", xonchar);
#ifdef __WIN32__
    slp->tio.XonChar = xonchar;
#else
    slp->tio.c_cc[VSTART] = xonchar;
#endif
}

static int get_xonchar(sl_t* slp)
{
    EAPI_DRV_DBG("get: xonchar");
#ifdef __WIN32__
    return slp->tio.XonChar;
#else
    return slp->tio.c_cc[VSTART];
#endif
}

static void set_xoffchar(sl_t* slp, int xoffchar)
{
    EAPI_DRV_DBG("set: xoffchar %c", xoffchar);
#ifdef __WIN32__
    slp->tio.XoffChar = xoffchar;
#else
    slp->tio.c_cc[VSTOP] = xoffchar;
#endif
}

static int get_xoffchar(sl_t* slp)
{
    EAPI_DRV_DBG("get: xoffchar");
#ifdef __WIN32__
    return slp->tio.XoffChar;
#else
    return slp->tio.c_cc[VSTOP];
#endif
}


static void set_swflow(sl_t* slp, int on)
{
    EAPI_DRV_DBG("set: swflow: %d", on);
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
    EAPI_DRV_DBG("get: swflow");
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
    EAPI_DRV_DBG("set: hwflow: %d", on);
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
    EAPI_DRV_DBG("get: hwflow");
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
    EAPI_DRV_DBG("send_break:");
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
    EAPI_DRV_DBG("send_xon:");
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
    EAPI_DRV_DBG("send_xoff:");
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
    EAPI_DRV_DBG("do_close: %d", slp->com);
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
    EAPI_DRV_DBG("do_close: done%s", "");
}

static void slp_update(sl_t* slp)
{
    EAPI_DRV_DBG("slp_update: %04X", slp->flags);

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
	EAPI_DRV_DBG("do_write_buf: error=EAGAIN");
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
	EAPI_DRV_DBG("do_write_buf: error=%s", strerror(errno));
    }
#endif
    EAPI_DRV_DBG("do_write_buf: %d buf=%d", n, nb);
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
    EAPI_DRV_DBG("do_write_more: %d", n);
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
    EAPI_DRV_DBG("open: dev=%s\n", slp->dev);
    slp->com = open(slp->dev, O_RDWR|O_NDELAY|O_NOCTTY);
    EAPI_DRV_DBG("open: fd=%d\n", slp->com);
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

// FIMXE: return int status!
static void sl_init(ErlDrvData d)
{
    eapi_ctx_t* ctx = (eapi_ctx_t*) d;
    sl_t* slp;
    
    slp = (sl_t*) driver_alloc(sizeof (sl_t));
    memset(slp, 0, sizeof(sl_t));

    slp->com   = INVALID;
    slp->bufsz = 1;
    slp->buftm = 0;

#ifdef __WIN32__
    memset(&slp->in, 0, sizeof(slp->in));
    memset(&slp->out, 0, sizeof(slp->out));
    memset(&slp->stat, 0, sizeof(slp->stat));
#endif
    ctx->user_data = slp;
}


static void sl_finish(ErlDrvData d)
{
    eapi_ctx_t* ctx = (eapi_ctx_t*) d;
    sl_t* slp = (sl_t*) ctx->user_data;

    slp_close(slp);

    if (slp->ibuf != NULL)
	driver_free(slp->ibuf);
    if (slp->obuf != NULL)
	driver_free(slp->obuf);
    driver_free(slp);
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
	EAPI_DRV_DBG("sl_input_output: n=%d", n);
	driver_output(slp->port, slp->ibuf, n);
    }
#endif
}


static void sl_ready_output(ErlDrvData data, ErlDrvEvent event)
{
    sl_t* slp = (sl_t*) data;
#ifdef __WIN32__
    DWORD n;
    EAPI_DRV_DBG("sl_ready_output: qsize=%d", driver_sizeq(slp->port));
    slp->o_pending = 0;
    driver_select(slp->port, (ErlDrvEvent)slp->out.hEvent,ERL_DRV_READ,0);
    if (!GetOverlappedResult(slp->com, &slp->out, &n, FALSE)) {
	/* Output error */
	return;
    }
    /* Dequeue ? */
#else
    EAPI_DRV_DBG("sl_ready_output: qsize=%d", driver_sizeq(slp->port));
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

void sl_drv_impl_open(eapi_ctx_t* ctx,cbuf_t* c_out)
{
    sl_t* slp = (sl_t*) ctx->user_data;
    if (slp->dev == NULL) /* MUST set device first */
	put_error(c_out, "badarg");
    else if (do_open(slp) < 0)
	put_error(c_out, "eaccess");
    else {
#ifdef __WIN32__
	driver_select(slp->port,(ErlDrvEvent)slp->stat.hEvent,DO_READ,1);
#else
	driver_select(slp->port,(ErlDrvEvent)slp->com, DO_READ, 1);
#endif
	put_ok(c_out);
    }
}

void sl_drv_impl_sendchar(eapi_ctx_t* ctx, cbuf_t* c_out, uint8_t c)
{
    sl_t* slp = (sl_t*) ctx->user_data;
    char buf[1];

    EAPI_DRV_DBG("sendchar: qsize=%d", driver_sizeq(slp->port));

    buf[0] = (char) c;
    do_write_init(slp, buf, 1);
    put_ok(c_out);
}


void sl_drv_impl_send(eapi_ctx_t* ctx, cbuf_t* c_out, eapi_binary_t* bp)
{
    sl_t* slp = (sl_t*) ctx->user_data;

    EAPI_DRV_DBG("send: qsize=%d", driver_sizeq(slp->port));

    do_write_init(slp, bp->bin->orig_bytes + bp->offset, bp->len);
    put_ok(c_out);
}


void sl_drv_impl_connect(eapi_ctx_t* ctx,cbuf_t* c_out)
{
    sl_drv_impl_open(ctx, c_out);
}

void sl_drv_impl_close(eapi_ctx_t* ctx,cbuf_t* c_out)
{
    sl_t* slp = (sl_t*) ctx->user_data;
    slp_close(slp);
    put_ok(c_out);
}

void sl_drv_impl_disconnect(eapi_ctx_t* ctx,cbuf_t* c_out)
{
    sl_t* slp = (sl_t*) ctx->user_data;
    slp_hangup(slp);
    slp_close(slp);
    put_ok(c_out);
}

void sl_drv_impl_xon(eapi_ctx_t* ctx,cbuf_t* c_out)
{
    sl_t* slp = (sl_t*) ctx->user_data;
    send_xon(slp);
    put_ok(c_out);    
}

void sl_drv_impl_xoff(eapi_ctx_t* ctx,cbuf_t* c_out)
{
    sl_t* slp = (sl_t*) ctx->user_data;
    send_xoff(slp);
    put_ok(c_out);
}

void sl_drv_impl_break(eapi_ctx_t* ctx,cbuf_t* c_out)
{
    sl_t* slp = (sl_t*) ctx->user_data;
    send_break(slp, 0);
    put_ok(c_out);    
}

void sl_drv_impl_update(eapi_ctx_t* ctx,cbuf_t* c_out)
{
    sl_t* slp = (sl_t*) ctx->user_data;
    slp_update(slp);
    put_ok(c_out);
}

void sl_drv_impl_revert(eapi_ctx_t* ctx,cbuf_t* c_out)
{
    sl_t* slp = (sl_t*) ctx->user_data;
    slp->flags = 0;
    put_ok(c_out);    
}

void sl_drv_impl_get_baud_rates(eapi_ctx_t* ctx,cbuf_t* c_out)
{
    (void) ctx;
    int i;
    size_t n = 0;

    for (i = 0; rtab[i].baud != -1; i++)
	n++;
    cbuf_put_tuple_begin(c_out, 2);
    cbuf_put_tag_ok(c_out);
    cbuf_put_list_begin(c_out, n);

    for (i=0; rtab[i].baud != -1; i++)
	cbuf_put_int32(c_out, rtab[i].baud);
    cbuf_put_list_end(c_out, n);
    cbuf_put_tuple_end(c_out, 2);
}

void sl_drv_impl_set_device(eapi_ctx_t* ctx,cbuf_t* c_out,eapi_string_t* name)
{
    sl_t* slp = (sl_t*) ctx->user_data;
    if (slp->dev != NULL) {
	driver_free(slp->dev);
	slp->dev = NULL;
    }
    if (name->len > 0) {
	if ((slp->dev = driver_alloc(name->len + 1)) == NULL) {
	    put_error(c_out, "enomem");
	    return;
	}
	memcpy(slp->dev, name->buf, name->len);
	slp->dev[name->len] = '\0';
    }
    put_ok(c_out);
}

void sl_drv_impl_set_ibaud(eapi_ctx_t* ctx,cbuf_t* c_out,int32_t baud)
{
    sl_t* slp = (sl_t*) ctx->user_data;
    // FIXME: range check
    slp->ibaud = baud;
    slp->flags |= FL_IBAUD;
    put_ok(c_out);    
}

void sl_drv_impl_set_obaud(eapi_ctx_t* ctx,cbuf_t* c_out,int32_t baud)
{
    sl_t* slp = (sl_t*) ctx->user_data;
    // FIXME: range check
    slp->obaud = baud;
    slp->flags |= FL_OBAUD;
    put_ok(c_out);
}

void sl_drv_impl_set_csize(eapi_ctx_t* ctx,cbuf_t* c_out,int32_t csize)
{
    sl_t* slp = (sl_t*) ctx->user_data;    
    slp->csize = csize;
    slp->flags |= FL_CSIZE;
    put_ok(c_out);
}

void sl_drv_impl_set_bufsize(eapi_ctx_t* ctx,cbuf_t* c_out,int32_t bufsize)
{
    sl_t* slp = (sl_t*) ctx->user_data;
    slp->bufsz = bufsize;
    slp->flags |= FL_BUFSZ;
    if (slp->bufsz < 0) slp->bufsz = 1;
    else if (slp->bufsz > 255) slp->bufsz = 255;
    put_ok(c_out);    
}

void sl_drv_impl_set_buftm(eapi_ctx_t* ctx,cbuf_t* c_out,int32_t buftm)
{
    sl_t* slp = (sl_t*) ctx->user_data;

    slp->buftm = buftm;
    slp->flags |= FL_BUFTM;
    put_ok(c_out);
}

void sl_drv_impl_set_stopb(eapi_ctx_t* ctx,cbuf_t* c_out,int32_t stopb)
{
    sl_t* slp = (sl_t*) ctx->user_data;
    slp->stopb = stopb;
    slp->flags |= FL_STOPB;
    put_ok(c_out);    
}

void sl_drv_impl_set_parity(eapi_ctx_t* ctx,cbuf_t* c_out,int32_t parity)
{
    sl_t* slp = (sl_t*) ctx->user_data;
    slp->parity = parity;
    slp->flags |= FL_PARITY;
    put_ok(c_out);
}


void sl_drv_impl_set_hwflow(eapi_ctx_t* ctx,cbuf_t* c_out,uint8_t enable)
{
    sl_t* slp = (sl_t*) ctx->user_data;
    slp->hwflow = enable;
    slp->flags |= FL_HWFLOW;
    put_ok(c_out);
}


void sl_drv_impl_set_swflow(eapi_ctx_t* ctx,cbuf_t* c_out,uint8_t enable)
{
    sl_t* slp = (sl_t*) ctx->user_data;
    slp->swflow = enable;
    slp->flags |= FL_SWFLOW;
    put_ok(c_out);
}

void sl_drv_impl_set_xoffchar(eapi_ctx_t* ctx,cbuf_t* c_out,int32_t c)
{
    sl_t* slp = (sl_t*) ctx->user_data;
    slp->xoffchar = c;
    slp->flags |= FL_XOFFCHAR;
    put_ok(c_out);
}

void sl_drv_impl_set_xonchar(eapi_ctx_t* ctx,cbuf_t* c_out,int32_t c)
{
    sl_t* slp = (sl_t*) ctx->user_data;
    slp->xonchar = c;
    slp->flags |= FL_XONCHAR;
    put_ok(c_out);
}

void sl_drv_impl_set_eolchar(eapi_ctx_t* ctx,cbuf_t* c_out,int32_t c)
{
    sl_t* slp = (sl_t*) ctx->user_data;
    slp->eolchar = c;
    slp->flags |= FL_EOLCHAR;
    put_ok(c_out);
}

void sl_drv_impl_set_eol2char(eapi_ctx_t* ctx,cbuf_t* c_out,int32_t c)
{
    sl_t* slp = (sl_t*) ctx->user_data;
    slp->eol2char = c;
    slp->flags |= FL_EOL2CHAR;
    put_ok(c_out);
}

void sl_drv_impl_set_mode(eapi_ctx_t* ctx,cbuf_t* c_out,int32_t mode)
{
    sl_t* slp = (sl_t*) ctx->user_data;
    if (mode != slp->mode) {
	if (mode == SL_MODE_RAW)
	    slp->mode = SL_MODE_RAW;
	else if (mode == SL_MODE_LINE)
	    slp->mode = SL_MODE_LINE;
	else {
	    put_error(c_out, "badarg");
	    return;
	}
	slp->flags |= FL_MODE;
    }
    put_ok(c_out);
}

void sl_drv_impl_get_device(eapi_ctx_t* ctx,cbuf_t* c_out)
{
    sl_t* slp = (sl_t*) ctx->user_data;
    if (!slp->dev)
	put_error(c_out, "badarg");
    else {
	cbuf_put_tuple_begin(c_out, 2);
	cbuf_put_tag_ok(c_out);
	cbuf_put_string(c_out, slp->dev, strlen(slp->dev));
	cbuf_put_tuple_end(c_out, 2);
    }
}


void sl_drv_impl_get_ibaud(eapi_ctx_t* ctx,cbuf_t* c_out)
{
    sl_t* slp = (sl_t*) ctx->user_data;

    if (slp->flags & FL_IBAUD)
	put_i32(c_out, slp->ibaud);
    else if (slp->com != INVALID)
	put_i32(c_out, get_ibaud(slp));
    else
	put_error(c_out, "ebadf");
}

void sl_drv_impl_get_obaud(eapi_ctx_t* ctx,cbuf_t* c_out)
{
    sl_t* slp = (sl_t*) ctx->user_data;

    if (slp->flags & FL_OBAUD)
	put_i32(c_out, slp->obaud);
    else if (slp->com != INVALID)
	put_i32(c_out, get_obaud(slp));
    else
	put_error(c_out, "ebadf");
}

void sl_drv_impl_get_csize(eapi_ctx_t* ctx,cbuf_t* c_out)
{
    sl_t* slp = (sl_t*) ctx->user_data;

    if (slp->flags & FL_CSIZE)
	put_i32(c_out, slp->csize);
    else if (slp->com != INVALID)
	put_i32(c_out, get_csize(slp));
    else
	put_error(c_out, "ebadf");
}

void sl_drv_impl_get_bufsize(eapi_ctx_t* ctx,cbuf_t* c_out)
{
    sl_t* slp = (sl_t*) ctx->user_data;

    if (slp->flags & FL_BUFSZ)
	put_i32(c_out, slp->bufsz);
    else if (slp->com != INVALID)
	put_i32(c_out, get_bufsz(slp));
    else
	put_error(c_out, "ebadf");
}

void sl_drv_impl_get_buftm(eapi_ctx_t* ctx,cbuf_t* c_out)
{
    sl_t* slp = (sl_t*) ctx->user_data;

    if (slp->flags & FL_BUFTM)
	put_i32(c_out, slp->buftm);
    else if (slp->com != INVALID)
	put_i32(c_out, get_buftm(slp));
    else
	put_error(c_out, "ebadf");
}

void sl_drv_impl_get_stopb(eapi_ctx_t* ctx,cbuf_t* c_out)
{
    sl_t* slp = (sl_t*) ctx->user_data;

    if (slp->flags & FL_STOPB)
	put_i32(c_out, slp->stopb);
    else if (slp->com != INVALID)
	put_i32(c_out, get_stopb(slp));
    else
	put_error(c_out, "ebadf");
}

void sl_drv_impl_get_parity(eapi_ctx_t* ctx,cbuf_t* c_out)
{
    sl_t* slp = (sl_t*) ctx->user_data;

    if (slp->flags & FL_PARITY)
	put_i32(c_out, slp->parity);
    else if (slp->com != INVALID)
	put_i32(c_out, get_parity(slp));
    else
	put_error(c_out, "ebadf");
}

void sl_drv_impl_get_hwflow(eapi_ctx_t* ctx,cbuf_t* c_out)
{
    sl_t* slp = (sl_t*) ctx->user_data;

    if (slp->flags & FL_HWFLOW)
	put_i32(c_out, slp->hwflow);
    else if (slp->com != INVALID)
	put_i32(c_out, get_hwflow(slp));
    else
	put_error(c_out, "ebadf");
}

void sl_drv_impl_get_swflow(eapi_ctx_t* ctx,cbuf_t* c_out)
{
    sl_t* slp = (sl_t*) ctx->user_data;
    if (slp->flags & FL_SWFLOW)   put_i32(c_out, slp->swflow);
    else if (slp->com != INVALID) put_i32(c_out, get_swflow(slp));
    else put_error(c_out, "ebadf");
}

void sl_drv_impl_get_xoffchar(eapi_ctx_t* ctx,cbuf_t* c_out)
{
    sl_t* slp = (sl_t*) ctx->user_data;
    if (slp->flags & FL_XOFFCHAR)   put_i32(c_out, slp->xoffchar);
    else if (slp->com != INVALID) put_i32(c_out, get_xoffchar(slp));
    else put_error(c_out, "ebadf");
}

void sl_drv_impl_get_xonchar(eapi_ctx_t* ctx,cbuf_t* c_out)
{
    sl_t* slp = (sl_t*) ctx->user_data;
    if (slp->flags & FL_XONCHAR)   put_i32(c_out, slp->xonchar);
    else if (slp->com != INVALID) put_i32(c_out, get_xonchar(slp));
    else put_error(c_out, "ebadf");
}

void sl_drv_impl_get_eolchar(eapi_ctx_t* ctx,cbuf_t* c_out)
{
    sl_t* slp = (sl_t*) ctx->user_data;
    if (slp->flags & FL_EOLCHAR)   put_i32(c_out, slp->eolchar);
    else if (slp->com != INVALID) put_i32(c_out, slp->ceol);
    else put_error(c_out, "ebadf");
}

void sl_drv_impl_get_eol2char(eapi_ctx_t* ctx,cbuf_t* c_out)
{
    sl_t* slp = (sl_t*) ctx->user_data;
    if (slp->flags & FL_EOLCHAR)   put_i32(c_out, slp->eol2char);
    else if (slp->com != INVALID) put_i32(c_out, slp->ceol2);
    else put_error(c_out, "ebadf");
}

void sl_drv_impl_get_mode(eapi_ctx_t* ctx,cbuf_t* c_out)
{
    sl_t* slp = (sl_t*) ctx->user_data;
    if (slp->flags & FL_MODE)   put_i32(c_out, slp->mode);
    else if (slp->com != INVALID) put_i32(c_out, slp->cmode);
    else put_error(c_out, "ebadf");
}

DRIVER_INIT(sl_drv)
{
    eapi_driver_init(&sl_drv_entry,
		     sl_init,
		     sl_finish);
    sl_drv_entry.driver_name = "sl_drv";
    sl_drv_entry.ready_input = sl_ready_input;
    sl_drv_entry.ready_output = sl_ready_output;
    return (ErlDrvEntry*) &sl_drv_entry;
}
