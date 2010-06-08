%%
%% Definitions used here and there
%%
-ifndef(__SL_HRL__).
-define(__SL_HRL__, true).

%% FIXME: split encoder/decoder part that use this definition
-define(POINTER_SIZE, 32).
-define(SIZE_SIZE,    32).

%% transport types
-define(u_int8_t(X),    X:8/native-unsigned-integer).
-define(u_int16_t(X),   X:16/native-unsigned-integer).
-define(u_int32_t(X),   X:32/native-unsigned-integer).
-define(u_int64_t(X),   X:64/native-unsigned-integer).
-define(int8_t(X),      X:8/native-signed-integer).
-define(int16_t(X),     X:16/native-signed-integer).
-define(int32_t(X),     X:32/native-signed-integer).
-define(int64_t(X),     X:64/native-signed-integer).
-define(float_t(X),     X:32/native-float).
-define(double_t(X),    X:64/native-float).
-define(pointer_t(X),   X:?POINTER_SIZE/native-unsigned-integer).
-define(size_t(X),      X:?SIZE_SIZE/native-unsigned-integer).

-endif.


