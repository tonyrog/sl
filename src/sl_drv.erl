%%%---- BEGIN COPYRIGHT --------------------------------------------------------
%%%
%%% Copyright (C) 2007 - 2012, Rogvall Invest AB, <tony@rogvall.se>
%%%
%%% This software is licensed as described in the file COPYRIGHT, which
%%% you should have received as part of this distribution. The terms
%%% are also available at http://www.rogvall.se/docs/copyright.txt.
%%%
%%% You may opt to use, copy, modify, merge, publish, distribute and/or sell
%%% copies of the Software, and permit persons to whom the Software is
%%% furnished to do so, under the terms of the COPYRIGHT file.
%%%
%%% This software is distributed on an "AS IS" basis, WITHOUT WARRANTY OF ANY
%%% KIND, either express or implied.
%%%
%%%---- END COPYRIGHT ----------------------------------------------------------
%% Generated by eapi (DO NOT EDIT)
-module(sl_drv).

-export([open/1]).
-export([xon/1]).
-export([set_buftm/2]).
-export([get_buftm/1]).
-export([break/1]).
-export([set_parity/2]).
-export([get_parity/1]).
-export([set_hwflow/2]).
-export([set_swflow/2]).
-export([get_hwflow/1]).
-export([get_swflow/1]).
-export([xoff/1]).
-export([close/1]).
-export([update/1]).
-export([set_device/2]).
-export([set_csize/2]).
-export([set_bufsize/2]).
-export([set_mode/2]).
-export([get_device/1]).
-export([get_csize/1]).
-export([get_bufsize/1]).
-export([get_mode/1]).
-export([connect/1]).
-export([disconnect/1]).
-export([revert/1]).
-export([send/2]).
-export([set_ibaud/2]).
-export([set_obaud/2]).
-export([get_ibaud/1]).
-export([get_obaud/1]).
-export([get_baud_rates/1]).
-export([sendchar/2]).
-export([set_stopb/2]).
-export([set_xoffchar/2]).
-export([set_xonchar/2]).
-export([set_eolchar/2]).
-export([set_eol2char/2]).
-export([get_stopb/1]).
-export([get_xoffchar/1]).
-export([get_xonchar/1]).
-export([get_eolchar/1]).
-export([get_eol2char/1]).
-include_lib("eapi/include/cbuf.hrl").
-include("sl_drv.hrl").

open(EApiPort) -> 
  eapi_drv:control(EApiPort,?SL_CMD_OPEN, <<>>).
connect(EApiPort) -> 
  eapi_drv:control(EApiPort,?SL_CMD_CONNECT, <<>>).
disconnect(EApiPort) -> 
  eapi_drv:control(EApiPort,?SL_CMD_DISCONNECT, <<>>).
close(EApiPort) -> 
  eapi_drv:control(EApiPort,?SL_CMD_CLOSE, <<>>).
xon(EApiPort) -> 
  eapi_drv:control(EApiPort,?SL_CMD_XON, <<>>).
xoff(EApiPort) -> 
  eapi_drv:control(EApiPort,?SL_CMD_XOFF, <<>>).
break(EApiPort) -> 
  eapi_drv:control(EApiPort,?SL_CMD_BREAK, <<>>).
update(EApiPort) -> 
  eapi_drv:control(EApiPort,?SL_CMD_UPDATE, <<>>).
revert(EApiPort) -> 
  eapi_drv:control(EApiPort,?SL_CMD_REVERT, <<>>).
sendchar(EApiPort,C) -> 
  eapi_drv:control(EApiPort,?SL_CMD_SENDCHAR, <<?uint8_t((C))>>).
send(EApiPort,Data) -> 
  Data__size = byte_size(Data),
 eapi_drv:command(EApiPort,?SL_CMD_SEND, [<<?uint32_t(Data__size),(Data)/binary>>]).
get_baud_rates(EApiPort) -> 
  eapi_drv:control(EApiPort,?SL_CMD_GET_BAUD_RATES, <<>>).
set_device(EApiPort,Name) -> 
  Name__bin = list_to_binary([Name]),Name__size = byte_size(Name__bin),
  eapi_drv:control(EApiPort,?SL_CMD_SET_DEVICE, <<?uint32_t(Name__size),Name__bin/binary>>).
set_ibaud(EApiPort,Baud) -> 
  eapi_drv:control(EApiPort,?SL_CMD_SET_IBAUD, <<?int32_t((Baud))>>).
set_obaud(EApiPort,Baud) -> 
  eapi_drv:control(EApiPort,?SL_CMD_SET_OBAUD, <<?int32_t((Baud))>>).
set_csize(EApiPort,Csize) -> 
  eapi_drv:control(EApiPort,?SL_CMD_SET_CSIZE, <<?int32_t((Csize))>>).
set_bufsize(EApiPort,Bufsize) -> 
  eapi_drv:control(EApiPort,?SL_CMD_SET_BUFSIZE, <<?int32_t((Bufsize))>>).
set_buftm(EApiPort,Buftm) -> 
  eapi_drv:control(EApiPort,?SL_CMD_SET_BUFTM, <<?int32_t((Buftm))>>).
set_stopb(EApiPort,Stopb) -> 
  eapi_drv:control(EApiPort,?SL_CMD_SET_STOPB, <<?int32_t((Stopb))>>).
set_parity(EApiPort,Parity) -> 
  eapi_drv:control(EApiPort,?SL_CMD_SET_PARITY, <<?int32_t((Parity))>>).
set_hwflow(EApiPort,Enable) -> 
  Enable__bool=if (Enable) -> 1; true -> 0 end,
  eapi_drv:control(EApiPort,?SL_CMD_SET_HWFLOW, <<?uint8_t(Enable__bool)>>).
set_swflow(EApiPort,Enable) -> 
  Enable__bool=if (Enable) -> 1; true -> 0 end,
  eapi_drv:control(EApiPort,?SL_CMD_SET_SWFLOW, <<?uint8_t(Enable__bool)>>).
set_xoffchar(EApiPort,C) -> 
  eapi_drv:control(EApiPort,?SL_CMD_SET_XOFFCHAR, <<?int32_t((C))>>).
set_xonchar(EApiPort,C) -> 
  eapi_drv:control(EApiPort,?SL_CMD_SET_XONCHAR, <<?int32_t((C))>>).
set_eolchar(EApiPort,C) -> 
  eapi_drv:control(EApiPort,?SL_CMD_SET_EOLCHAR, <<?int32_t((C))>>).
set_eol2char(EApiPort,C) -> 
  eapi_drv:control(EApiPort,?SL_CMD_SET_EOL2CHAR, <<?int32_t((C))>>).
set_mode(EApiPort,Mode) -> 
  eapi_drv:control(EApiPort,?SL_CMD_SET_MODE, <<?int32_t((Mode))>>).
get_device(EApiPort) -> 
  eapi_drv:control(EApiPort,?SL_CMD_GET_DEVICE, <<>>).
get_ibaud(EApiPort) -> 
  eapi_drv:control(EApiPort,?SL_CMD_GET_IBAUD, <<>>).
get_obaud(EApiPort) -> 
  eapi_drv:control(EApiPort,?SL_CMD_GET_OBAUD, <<>>).
get_csize(EApiPort) -> 
  eapi_drv:control(EApiPort,?SL_CMD_GET_CSIZE, <<>>).
get_bufsize(EApiPort) -> 
  eapi_drv:control(EApiPort,?SL_CMD_GET_BUFSIZE, <<>>).
get_buftm(EApiPort) -> 
  eapi_drv:control(EApiPort,?SL_CMD_GET_BUFTM, <<>>).
get_stopb(EApiPort) -> 
  eapi_drv:control(EApiPort,?SL_CMD_GET_STOPB, <<>>).
get_parity(EApiPort) -> 
  eapi_drv:control(EApiPort,?SL_CMD_GET_PARITY, <<>>).
get_hwflow(EApiPort) -> 
  eapi_drv:control(EApiPort,?SL_CMD_GET_HWFLOW, <<>>).
get_swflow(EApiPort) -> 
  eapi_drv:control(EApiPort,?SL_CMD_GET_SWFLOW, <<>>).
get_xoffchar(EApiPort) -> 
  eapi_drv:control(EApiPort,?SL_CMD_GET_XOFFCHAR, <<>>).
get_xonchar(EApiPort) -> 
  eapi_drv:control(EApiPort,?SL_CMD_GET_XONCHAR, <<>>).
get_eolchar(EApiPort) -> 
  eapi_drv:control(EApiPort,?SL_CMD_GET_EOLCHAR, <<>>).
get_eol2char(EApiPort) -> 
  eapi_drv:control(EApiPort,?SL_CMD_GET_EOL2CHAR, <<>>).
get_mode(EApiPort) -> 
  eapi_drv:control(EApiPort,?SL_CMD_GET_MODE, <<>>).
