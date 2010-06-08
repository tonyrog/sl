%%
%% SL internal mappings
%%
-ifndef(__SL_INT_HRL__).
-define(__SL_INT_HRL__, true).

-define(SL_SERVER, sl_serv).
-define(SL_PORT,   sl_port).
-define(SL_REG,    sl_reg).

-define(OK,             1).
-define(ERROR,          2).
-define(EVENT,          3).
-define(INT8,           4).
-define(UINT8,          5).
-define(INT16,          6).
-define(UINT16,         7).
-define(INT32,          8).
-define(UINT32,         9).
-define(INT64,          10).
-define(UINT64,         11).
-define(BOOLEAN,        12).
-define(FLOAT32,        13).
-define(FLOAT64,        14).
-define(STRING1,        15).
-define(STRING4,        16).
-define(ATOM,           17).
-define(BINARY,         18).
-define(LIST,           19).
-define(LIST_END,       20).
-define(TUPLE,          21).
-define(TUPLE_END,      22).
-define(ENUM,           23).
-define(BITFIELD,       24).
-define(HANDLE,         25).


-define(SL_CONNECT,    1).
-define(SL_DISCONNECT, 2).
-define(SL_OPEN,       3).
-define(SL_CLOSE,      4).
-define(SL_XOFF,       5).
-define(SL_XON,        6).
-define(SL_BREAK,      7).
-define(SL_UPDATE,     8).
-define(SL_GET_RATES,  9).
-define(SL_REVERT,     10).

-define(SL_SET_DEV,    20).
-define(SL_SET_IBAUD,  22).
-define(SL_SET_OBAUD,  24).
-define(SL_SET_CSIZE,  26).
-define(SL_SET_BUFSZ,  28).
-define(SL_SET_BUFTM,  30).
-define(SL_SET_STOPB,  32).
-define(SL_SET_PARITY, 34).
-define(SL_SET_HWFLOW, 36).
-define(SL_SET_SWFLOW, 38).
-define(SL_SET_XONCHAR,    40).
-define(SL_SET_XOFFCHAR,   42).
-define(SL_SET_MODE,   46).
-define(SL_SET_EOLCHAR, 48).
-define(SL_SET_EOL2CHAR, 50).


-define(SL_GET_DEV,    21).
-define(SL_GET_IBAUD,  23).
-define(SL_GET_OBAUD,  25).
-define(SL_GET_CSIZE,  27).
-define(SL_GET_BUFSZ,  29).
-define(SL_GET_BUFTM,  31).
-define(SL_GET_STOPB,  33).
-define(SL_GET_PARITY, 35).
-define(SL_GET_HWFLOW, 37).
-define(SL_GET_SWFLOW, 39).
-define(SL_GET_XONCHAR,    41).
-define(SL_GET_XOFFCHAR,   43).
-define(SL_GET_MODE,   47).
-define(SL_GET_EOLCHAR, 49).
-define(SL_GET_EOL2CHAR, 51).

-define(SL_MODE_RAW,    0).
-define(SL_MODE_LINE,   1).


-endif.
