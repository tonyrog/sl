%%% File    : sl.erl
%%% Author  : Tony Rogvall <tony@a55.hemma.se>
%%% Description : Serial line control
%%% Created :  7 Oct 2002 by Tony Rogvall <tony@a55.hemma.se>

-module(sl).

-export([open/2, close/1, send/2, break/1, xon/1, xoff/1]).
-export([options/0]).
-export([setopts/2, getopts/2]).

-compile(export_all).

-define(SL_MODE_RAW,  0).
-define(SL_MODE_LINE, 1).

options() ->
    [dev, ibaud, obaud, baud, csize, bufsz, buftm, stopb, parity,
     hwflow, swflow, xonchar, xoffchar, eolchar, eol2char, 
     mode].

getopts(P, [Opt|Opts]) ->
    case getopt(P, Opt) of
	{ok, Val} -> [{Opt,Val} | getopts(P, Opts)];
	_Error -> getopts(P, Opts)
    end;
getopts(_P, []) -> [].

getopt(SL, Opt) ->
    case Opt of
	rates  -> sl_drv:get_baud_rates(SL);
	dev    -> sl_drv:get_device(SL);
	ibaud  -> sl_drv:get_ibaud(SL);
	obaud  -> sl_drv:get_obaud(SL);
	csize  -> sl_drv:get_csize(SL);
	bufsz  -> sl_drv:get_bufsize(SL);
	buftm  -> sl_drv:get_buftm(SL);
	stopb  -> sl_drv:get_stopb(SL);
	parity -> sl_drv:get_parity(SL);
	hwflow -> sl_drv:get_hwflow(SL);
	swflow -> sl_drv:get_swflow(SL);
	xonchar    -> sl_drv:get_xonchar(SL);
	xoffchar   -> sl_drv:get_xoffchar(SL);
	eolchar    -> sl_drv:get_eolchar(SL);
	eol2char   -> sl_drv:get_eol2char(SL);
	mode  ->
	    case sl_drv:get_mode(SL) of
		{ok,?SL_MODE_RAW}  -> {ok, raw};
		{ok,?SL_MODE_LINE} -> {ok, line};
		Other -> Other
	    end;
	baud  -> sl_drv:get_ibaud(SL);
	_ -> {error, {bad_opt, Opt}}
    end.
    
setopt(SL, Opt, Arg) ->
    case Opt of
	dev    -> sl_drv:set_device(SL, Arg);
	ibaud  -> sl_drv:set_ibaud(SL, Arg);
	obaud  -> sl_drv:set_obaud(SL, Arg);
	csize  -> sl_drv:set_csize(SL, Arg);
	bufsz  -> sl_drv:set_bufsize(SL, Arg);
	buftm  -> sl_drv:set_buftm(SL, Arg);
	stopb  -> sl_drv:set_stopb(SL, Arg);
	parity -> sl_drv:set_parity(SL, Arg);
	hwflow -> sl_drv:set_hwflow(SL, Arg);
	swflow -> sl_drv:set_swflow(SL, Arg);
	xoffchar -> sl_drv:set_xoffchar(SL, Arg);
	xonchar  -> sl_drv:set_xonchar(SL, Arg);
	eolchar -> sl_drv:set_eolchar(SL, Arg);
	eol2char -> sl_drv:set_eol2char(SL, Arg);
	binary -> ok;
	debug -> ok;
	mode   ->
	    if is_integer(Arg) ->
		    sl_drv:set_mode(SL, Arg);
		Arg=:=raw -> 
		    sl_drv:set_mode(SL, ?SL_MODE_RAW);
		Arg=:=line ->
		    sl_drv:set_mode(SL, ?SL_MODE_LINE)
	    end;
	baud ->
	    case sl_drv:set_ibaud(SL, Arg) of
		ok ->
		    sl_drv:set_obaud(SL, Arg);
		Err ->
		    Err
	    end;
	_ -> {error, {badarg,Opt}}
    end.


setopts(_SL, []) -> ok;
setopts(SL, Opts) ->
    try setopts0(SL, Opts) of
	ok -> 
	    sl_drv:update(SL)
    catch
	error:Error ->
	    sl_drv:revert(SL),
	    {error,Error}
    end.

setopts0(P, [{Opt,Arg}|Opts]) ->
    case setopt(P, Opt, Arg) of
	ok -> setopts0(P, Opts);
	Error -> Error
    end;
setopts0(P, [binary|Opts]) ->
    setopts0(P, Opts);
setopts0(P, [debug|Opts]) ->
    setopts0(P, Opts);
setopts0(_P, []) -> 
    ok.

	    
open(Dev, Opts) ->
    case eapi_drv:open([{driver_name, "sl_drv"},
			{app, sl} | Opts]) of
	{ok,SL} ->
	    case sl_drv:set_device(SL, Dev) of
		ok ->
		    case setopts0(SL, Opts) of
			ok ->
			    case sl_drv:connect(SL) of
				ok ->
				    {ok,SL};
				Error ->
				    erlang:port_close(SL),
				    Error
			    end;
			Error ->
			    erlang:port_close(SL),
			    Error
		    end;
		Error ->
		    erlang:port_close(SL),
		    Error
	    end;
	Error ->
	    Error
    end.

close(SL) ->
    sl_drv:close(SL),
    erlang:port_close(SL).

sl_revert(SL) ->
    sl_drv:reverse(SL).

break(SL) ->
    sl_drv:break(SL).

xon(SL) ->
    sl_drv:xon(SL).

xoff(SL) ->
    sl_drv:xoff(SL).

send(SL, [C]) when is_integer(C), C >= 0, C =< 255 ->
    sl_drv:sendchar(SL, C);
send(SL, <<C>>) ->
    sl_drv:sendchar(SL, C);
send(SL, Data) when is_list(Data) ->
    sl_drv:send(SL, list_to_binary(Data));
send(SL, Data) when is_binary(Data) ->
    sl_drv:send(SL, Data).

