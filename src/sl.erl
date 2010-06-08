%%% File    : sl.erl
%%% Author  : Tony Rogvall <tony@a55.hemma.se>
%%% Description : Serial line control
%%% Created :  7 Oct 2002 by Tony Rogvall <tony@a55.hemma.se>

-module(sl).

-export([open/2, close/1, send/2, break/1, xon/1, xoff/1]).
-export([options/0]).
-export([setopts/2, getopts/2]).

-compile(export_all).

-include("sl_int.hrl").
-include("sl.hrl").

-ifdef(debug).
-define(dbg(F,A), io:format((F)++"\n",(A))).
-define(dbg_hard(F,A), ok).
-define(drv_path, "debug").
-else.
-define(dbg(F,A), ok).
-define(dbg_hard(F,A), ok).
-define(drv_path, "release").
-endif.

-define(ASYNC_TIMEOUT, 1000). 

sl_create(OpenOpts) ->
    erl_ddll:load_driver(code:priv_dir(sl), "sl_drv"),
    open_port({spawn, "sl_drv"}, OpenOpts).

sl_delete(P) when is_port(P) ->
    erlang:port_close(P).

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


getopt(P, Opt) ->
    case Opt of
	rates  -> call(P, ?SL_GET_RATES, []);
	dev    -> call(P, ?SL_GET_DEV, []);
	ibaud  -> call(P, ?SL_GET_IBAUD, []);
	obaud  -> call(P, ?SL_GET_OBAUD, []);
	csize  -> call(P, ?SL_GET_CSIZE, []);
	bufsz  -> call(P, ?SL_GET_BUFSZ, []);
	buftm  -> call(P, ?SL_GET_BUFTM, []);
	stopb  -> call(P, ?SL_GET_STOPB, []);
	parity -> call(P, ?SL_GET_PARITY, []);
	hwflow -> call(P, ?SL_GET_HWFLOW, []);
	swflow -> call(P, ?SL_GET_SWFLOW, []);
	xonchar    -> call(P, ?SL_GET_XONCHAR, []);
	xoffchar   -> call(P, ?SL_GET_XOFFCHAR, []);
	eolchar    -> call(P, ?SL_GET_EOLCHAR, []);
	eol2char   -> call(P, ?SL_GET_EOL2CHAR, []);
	mode  ->
	    case call(P, ?SL_GET_MODE, []) of
		{ok,?SL_MODE_RAW}  -> {ok, raw};
		{ok,?SL_MODE_LINE} -> {ok, line};
		Other -> Other
	    end;
	baud  -> call(P, ?SL_GET_IBAUD, []);
	_ -> {error, {bad_opt, Opt}}
    end.
    
setopt(P, Opt, Arg) ->
    %% io:format("setopt: ~s value=~p\n", [Opt, Arg]),
    case Opt of
	dev    -> call(P, ?SL_SET_DEV, Arg);
	ibaud  -> call(P, ?SL_SET_IBAUD, <<?int32_t(Arg)>>);
	obaud  -> call(P, ?SL_SET_OBAUD, <<?int32_t(Arg)>>);
	csize  -> call(P, ?SL_SET_CSIZE, <<?int32_t(Arg)>>);
	bufsz  -> call(P, ?SL_SET_BUFSZ, <<?int32_t(Arg)>>);
	buftm  -> call(P, ?SL_SET_BUFTM, <<?int32_t(Arg)>>);
	stopb  -> call(P, ?SL_SET_STOPB, <<?int32_t(Arg)>>);
	parity -> call(P, ?SL_SET_PARITY, <<?int32_t(Arg)>>);
	hwflow -> call(P, ?SL_SET_HWFLOW, bool(Arg));
	swflow -> call(P, ?SL_SET_SWFLOW, bool(Arg));
	xoffchar -> call(P, ?SL_SET_XOFFCHAR, <<?int32_t(Arg)>>);
	xonchar  -> call(P, ?SL_SET_XONCHAR, <<?int32_t(Arg)>>);
	eolchar -> call(P,?SL_SET_EOLCHAR, <<?int32_t(Arg)>>);
	eol2char -> call(P,?SL_SET_EOL2CHAR, <<?int32_t(Arg)>>);
	binary -> ok;
	mode   ->
	    if is_integer(Arg) ->
		    call(P, ?SL_SET_MODE,<<?int32_t(Arg)>>);
		Arg==raw -> 
		    call(P, ?SL_SET_MODE, <<?int32_t(?SL_MODE_RAW)>>);
		Arg==line ->
		    call(P, ?SL_SET_MODE, <<?int32_t(?SL_MODE_LINE)>>)
            end;
	baud ->
	    case call(P, ?SL_SET_IBAUD, <<?int32_t(Arg)>>) of
		ok ->
		    call(P, ?SL_SET_OBAUD, <<?int32_t(Arg)>>);
		Err ->
		    Err
	    end;
	_ -> {error, {badarg,Opt}}
    end.

bool(true) ->  <<?int32_t(1)>>;
bool(false) -> <<?int32_t(0)>>.

setopts(_P, []) -> ok;
setopts(P, Opts) ->
    try setopts0(P, Opts) of
	ok -> sl_update(P)
    catch
	error:Error ->
	    sl_revert(P),
	    {error,Error}
    end.

setopts0(P, [{Opt,Arg}|Opts]) ->
    case setopt(P, Opt, Arg) of
	ok -> setopts0(P, Opts);
	Error -> Error
    end;
setopts0(P, [binary|Opts]) ->
    setopts0(P, Opts);
setopts0(_P, []) -> 
    ok.

	    
open(Dev, Opts) ->
    PortOpts = case proplists:get_value(binary, Opts, false) of
		   false -> [];
		   true -> [binary]
	       end,
    P = sl_create(PortOpts),
    case setopt(P, dev, Dev) of
	ok ->
	    case setopts(P, Opts) of
		ok -> 
		    case sl_connect(P) of
			ok -> 
			    {ok,P};
			Error -> 
			    sl_delete(P), 
			    Error
		    end;
		Error ->
		    sl_delete(P),
		    Error
	    end;
	Error ->
	    sl_delete(P),
	    Error
    end.

close(P) ->
    sl_close(P),
    sl_delete(P).


sl_open(P) ->
    call(P, ?SL_OPEN, "").

sl_close(P) ->
    call(P, ?SL_CLOSE, "").

sl_connect(P) ->
    call(P, ?SL_CONNECT, "").

sl_disconnect(P) ->
    call(P, ?SL_DISCONNECT, "").

sl_revert(P) ->
    call(P, ?SL_REVERT, "").

sl_update(P) ->
    call(P, ?SL_UPDATE, "").

break(P) ->
    call(P, ?SL_BREAK, "").    

xon(P) ->
    call(P, ?SL_XON, "").

xoff(P) ->
    call(P, ?SL_XOFF, "").

send(P, Data) ->
    erlang:port_command(P, Data).

call(P, Code, Args) ->
    Reply = erlang:port_control(P, Code, Args),
    case decode(Reply) of
	{event,Ref} ->
	    wait_reply(Ref);
	Result ->
	    Result
    end.

%% async call to command(v) interface
async_call(P, Code, Args) ->
    CmdRef = random:uniform(16#ffffffff),
    Header = <<?u_int8_t(Code),?u_int32_t(CmdRef)>>,
    erlang:port_command(P,[Header,Args]),
    wait_reply(CmdRef).

wait_reply(CmdRef) ->
    receive
	{sl_reply,CmdRef,Reply} when is_binary(Reply) ->
	    %% This is a reply from a port command
	    %% (passing binaries with zero copy)
	    decode(Reply);
	{sl_reply,CmdRef,Reply} ->
	    %% This is a reply from async calls (running in driver thread)
	    Reply
    after ?ASYNC_TIMEOUT ->
	    {error, no_reply}
    end.

decode(Data = <<131,_/binary>>) ->
    binary_to_term(Data);
decode(Data) -> 
    ?dbg_hard("deocde: data = ~p\n", [Data]),
    decode(Data, []).


decode(<<>>, [Hd]) -> 
    ?dbg_hard("deocde = ~p\n", [Hd]),
    Hd;
decode(Data, Stack) ->
    case Data of
	<<?OK, Rest/binary>> -> 
            ?dbg_hard("OK",[]),
            decode(Rest, [ok|Stack]);
	<<?ERROR, Rest/binary>> -> 
            ?dbg_hard("ERROR",[]),
            decode(Rest, [error|Stack]);
	<<?EVENT, Rest/binary>> -> 
            ?dbg_hard("EVENT",[]),
            decode(Rest, [event|Stack]);
        <<?LIST, Rest/binary>> -> 
            ?dbg_hard("LIST",[]),
            decode(Rest, [list|Stack]);
        <<?TUPLE, Rest/binary>> ->
            ?dbg_hard("TUPLE",[]),
            decode(Rest, [tuple|Stack]);
        <<?BOOLEAN, B, Rest/binary>> -> 
            ?dbg_hard("BOOLEAN:~w",[B]),
            decode(Rest, [B =/= 0 | Stack]);
        <<?UINT8, ?u_int8_t(I), Rest/binary>> -> 
            ?dbg_hard("UINT8:~w",[I]),
            decode(Rest, [I|Stack]);
        <<?UINT16, ?u_int16_t(I), Rest/binary>> ->
            ?dbg_hard("UINT16:~w",[I]),
            decode(Rest, [I|Stack]);
        <<?UINT32, ?u_int32_t(I), Rest/binary>> ->
            ?dbg_hard("UINT32:~w",[I]),
            decode(Rest, [I|Stack]);
        <<?UINT64, ?u_int64_t(I), Rest/binary>> ->
            ?dbg_hard("UINT64:~w",[I]),
            decode(Rest, [I|Stack]);
        <<?INT8, ?int8_t(I), Rest/binary>> ->
            ?dbg_hard("INT8:~w",[I]),
            decode(Rest, [I|Stack]);
        <<?INT16, ?int16_t(I), Rest/binary>> ->
            ?dbg_hard("INT16:~w",[I]),
            decode(Rest, [I|Stack]);
        <<?INT32, ?int32_t(I), Rest/binary>> -> 
            ?dbg_hard("INT32:~w",[I]),
            decode(Rest, [I|Stack]);
        <<?INT64, ?int64_t(I), Rest/binary>> ->
            ?dbg_hard("INT64:~w",[I]),
            decode(Rest, [I|Stack]);
        <<?FLOAT32, ?float_t(F), Rest/binary>> ->
            ?dbg_hard("FLOAT32:~w",[F]),
            decode(Rest, [F|Stack]);
        <<?FLOAT64, ?double_t(F), Rest/binary>> -> 
            ?dbg_hard("FLOAT64:~w",[F]),
            decode(Rest, [F|Stack]);
        <<?STRING1, ?u_int8_t(N), String:N/binary, Rest/binary>> -> 
            ?dbg_hard("STRING1: len=~w, ~w",[N,String]),
            decode(Rest, [binary_to_list(String)  | Stack]);
        <<?STRING4, ?u_int32_t(N), String:N/binary, Rest/binary>> ->
            ?dbg_hard("STRING4: len=~w, ~w",[N,String]),
            decode(Rest, [binary_to_list(String)  | Stack]);
        <<?BINARY, ?u_int32_t(N), Bin:N/binary, Rest/binary>> -> 
            ?dbg_hard("BINARY: len=~w, ~w",[N,Bin]),
            decode(Rest, [Bin | Stack]);
	<<?ATOM, ?u_int8_t(N), Atom:N/binary, Rest/binary>> -> 
            ?dbg_hard("ATOM: len=~w, ~w",[N,Atom]),
            decode(Rest, [list_to_atom(binary_to_list(Atom)) | Stack]);
        <<?LIST_END, Rest/binary>> ->
            ?dbg_hard("LIST_END",[]),
            {L,[_|Stack1]} = lists:splitwith(fun(X) -> X =/= list end, Stack),
            decode(Rest, [lists:reverse(L) | Stack1]);
        <<?TUPLE_END, Rest/binary>> ->
            ?dbg_hard("TUPLE_END",[]),
            {L,[_|Stack1]}=lists:splitwith(fun(X) -> X =/= tuple end, Stack),
            decode(Rest, [list_to_tuple(lists:reverse(L)) | Stack1])
    end.



%% Encode/Decode reply data from driver
%% encode into Data format
encode(Term) ->
    list_to_binary([enc(Term)]).

enc(true) ->   
    <<?BOOLEAN, 1>>;
enc(false) ->  
    <<?BOOLEAN, 0>>;
enc(X) when is_atom(X) ->
    Val = atom_to_list(X),
    [<<?ATOM, (length(Val)):8>>, Val];
enc(X) when is_integer(X) ->
    if X >= 0, X =< 16#ffffffff -> <<?UINT32, ?u_int32_t(X)>> ;
       X < 0, X >= -16#8000000  -> <<?INT32, ?int32_t(X)>> ;
       X > 0 -> <<?UINT64, ?u_int64_t(X)>>;
       true -> (<<?INT64, ?int64_t(X)>>)
    end;
enc(X) when is_float(X) -> 
    <<?FLOAT64, ?double_t(X)>>;
enc(X) when is_list(X) ->
    case is_string(X) of
        true ->
            Len = length(X),
            if Len =< 255 ->
                    [<<?STRING1, ?u_int8_t(Len)>>, X];
               true ->
                    [<<?STRING4, ?u_int32_t(Len)>>, X]
            end;
        false ->
            [?LIST, lists:map(fun(E) -> enc(E) end, X), ?LIST_END]
    end;
enc(X) when is_tuple(X) ->
    [?TUPLE, lists:map(fun(E) -> enc(E) end, tuple_to_list(X)), ?TUPLE_END];
enc(X) when is_binary(X) ->
    Sz = byte_size(X),
    [<<?BINARY,?u_int32_t(Sz)>>, X].
     
is_string([X|Xs]) when X >= 0, X =< 255 -> is_string(Xs);
is_string([]) -> true;
is_string(_) -> false.

