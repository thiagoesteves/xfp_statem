%%%-------------------------------------------------------------------
%%% Created : 24 Feb 2020 by Thiago Esteves <thiagocalori@gmail.com>
%%%
%%% @doc
%%% This is the XFP implementation as a gen_statem. All the high-level
%%% functions are located here and the low level functions (i2c read 
%%% and gpio read) are accessed by PORT.
%%%
%%% This gen-server supports multiple devices connected and can be
%%% dynamically created/removed by the xfp_sup.
%%% @end
%%%-------------------------------------------------------------------

-module(xfp).

-behaviour(gen_statem).

-author('Thiago Esteves').

-include("xfp.hrl").

%% gen_server exports
-export([init/1,
         start_link/1,
         terminate/2,
         callback_mode/0,
         code_change/3]).

%% Public API export
-export([get_state/1,
         get_temperature/1,
         get_tx_bias/1,
         get_tx_power_mw/1,
         get_tx_power/1,
         get_rx_power_mw/1,
         get_rx_power/1,
         get_rx_los/1,
         get_laser_state/1,
         get_xfp_not_ready/1,
         set_laser_on/1,
         set_laser_off/1,
         set_reset/1]).

%% Public API states
-export([absent/3,
         inserted/3]).

%%%===================================================================
%%% Global Defines
%%%===================================================================

-define(SERVER, ?MODULE).
-define(XFP_CHECK_PRESENCE_INTERVAL, 1000).

%%%===================================================================
%%% Gen Statem Defines
%%%===================================================================

-define(HANDLE_COMMON, 
  ?FUNCTION_NAME(T, C, D) -> handle_common(T, C, ?FUNCTION_NAME, D)).

-define(HANDLE_PRESENCE,
  ?FUNCTION_NAME(info, check_presence, D) -> handle_presence(?FUNCTION_NAME, D)).

%%%===================================================================
%%% PIN Defines - See xfp_driver.c enumeration
%%%===================================================================

-define(XFP_PIN_MOD_DESEL, 0).
-define(XFP_PIN_TX_DIS   , 1).
-define(XFP_PIN_RESENCE  , 2).
-define(XFP_PIN_NOT_READY, 3).
-define(XFP_PIN_RX_LOS   , 4).
-define(XFP_PIN_RESET    , 5).
-define(XFP_PIN_POWERDOWN, 6).
-define(XFP_MAX_PIN      , 7).

-define(XFP_PRESENT      , 0).
-define(XFP_LASER_ON     , 0).
-define(XFP_LASER_OFF    , 1).

%%%===================================================================
%%% Register Defines
%%%===================================================================
%% Lower Memory Map
-define(XFP_REG_TEMPERATURE   , 96).
-define(XFP_REG_TX_BIAS       , 100).
-define(XFP_REG_TX_POWER      , 102).
-define(XFP_REG_RX_POWER      , 104).
-define(XFP_REG_SELECT_PAGE   , 127).
%% Table 01
-define(XFP_REG_IDENTIFIER    , 128).
-define(XFP_REG_VENDOR_NAME   , 148).
-define(XFP_REG_CDR_SUP       , 164).
-define(XFP_REG_VENDOR_OUI    , 165).
-define(XFP_REG_PART_NUMBER   , 168).
-define(XFP_REG_REVISION      , 184).
-define(XFP_REG_WAVELENGTH    , 186).
-define(XFP_REG_VENDOR_SERIAL , 196).
-define(XFP_REG_DATE_CODE     , 212).
-define(XFP_REG_DIAGNOSTIC    , 220).
-define(XFP_REG_ENHANCED      , 221).
-define(XFP_REG_AUX_MONITORING, 222).

%% Define registers size for the values bigger than 1
-define(XFP_REG_TEMPERATURE_SIZE   , 2).
-define(XFP_REG_TX_BIAS_SIZE       , 2).
-define(XFP_REG_TX_POWER_SIZE      , 2).
-define(XFP_REG_RX_POWER_SIZE      , 2).
-define(XFP_REG_VENDOR_NAME_SIZE   , 16).
-define(XFP_REG_CDR_SUP_SIZE       , 1).
-define(XFP_REG_VENDOR_OUI_SIZE    , 3).
-define(XFP_REG_PART_NUMBER_SIZE   , 16).
-define(XFP_REG_REVISION_SIZE      , 2).
-define(XFP_REG_WAVELENGTH_SIZE    , 2).
-define(XFP_REG_VENDOR_SERIAL_SIZE , 16).
-define(XFP_REG_DATE_CODE_SIZE     , 8).

%% Generic definition
-define(XFP_REG_SIZE_BITS    , 16).
-define(XFP_REG_OUI_SIZE_BITS, 24).

%% Minimum Dbm power measure
-define(XFP_DBM_MIN, -40.0).

%% Reset delay im ms
-define(XFP_RESET_DELAY, 100).

%%%===================================================================
%%% API
%%%===================================================================

%%--------------------------------------------------------------------
%% @private
%%--------------------------------------------------------------------
start_link([XfpName, Instance]) ->
  gen_statem:start_link({local, XfpName}, ?MODULE, [Instance], []).

%%%===================================================================
%%% gen_statem callbacks
%%%===================================================================

%%--------------------------------------------------------------------
%% @private
%%--------------------------------------------------------------------
init([Instance]) ->
  %% Register Gproc name
  gproc:reg({p, l, {?MODULE, Instance}}),
  %% comment this line to stop trapping exits
  process_flag(trap_exit, true),
  %% Start the check presence interval for the device with check_presence msg
  erlang:send(self(), check_presence),
  {ok, absent, #xfp_data { instance = Instance } }.

%%--------------------------------------------------------------------
%% @private
%%--------------------------------------------------------------------

callback_mode() ->
  [state_functions, state_enter].

terminate(_, _LD) ->
  gproc:goodbye().

%%--------------------------------------------------------------------
%% @private
%%--------------------------------------------------------------------
handle_presence(State, GenServerData) ->
  { ok, CurrPresence } = get_presence_priv(GenServerData),
  NewState = update_xfp_presence(State, CurrPresence),
  %% Send message to the next round
  erlang:send_after(?XFP_CHECK_PRESENCE_INTERVAL, self(), check_presence),
  {next_state, NewState, GenServerData}.

%%--------------------------------------------------------------------
%% STATE FUNCTIONS
%%--------------------------------------------------------------------
absent(enter, _MSG, GenServerData) ->
  {next_state, absent, #xfp_data { 
                         instance = GenServerData#xfp_data.instance} };
?HANDLE_PRESENCE;
?HANDLE_COMMON.

inserted(enter, _MSG, GenServerData) ->
  {next_state, inserted, upload_xfp_static_information(GenServerData)};
?HANDLE_PRESENCE;
?HANDLE_COMMON.

handle_common({call,From}, {get, state}, _, GenServerData) ->
  {keep_state_and_data, [{reply,From,GenServerData}]};

handle_common({call,From}, _, absent, _) ->
  {keep_state_and_data, [{reply,From,{ok, not_present}}]};

handle_common({call,From}, {get, Operation}, _, GenServerData) ->
  Res = read_priv(Operation, GenServerData#xfp_data.instance),
  {keep_state_and_data, [{reply,From,Res}]};

handle_common({call,From}, {set, Operation}, _, GenServerData) ->
  Res = write_priv(Operation, GenServerData#xfp_data.instance),
  {keep_state_and_data, [{reply,From,Res}]}.

%%--------------------------------------------------------------------
%% @private
%%--------------------------------------------------------------------
code_change(_OldVsn, State, _Extra) ->
  {ok, State}.

%%%===================================================================
%%% Exported XFP functions
%%%===================================================================

-spec get_state(xfpInstance()) -> { ok | error , #xfp_data{} }.
get_state(Instance) ->
  gproc_call(Instance, {get, state}).

-spec get_temperature(xfpInstance()) -> { ok | error , float()}.
get_temperature(Instance) ->
  gproc_call(Instance, {get, temperature}).

-spec get_tx_bias(xfpInstance()) -> { ok | error , float()}.
get_tx_bias(Instance) ->
  gproc_call(Instance, {get, tx_bias}).

-spec get_tx_power_mw(xfpInstance()) -> { ok | error , float()}.
get_tx_power_mw(Instance) ->
  gproc_call(Instance, {get, tx_power_mw}).

-spec get_tx_power(xfpInstance()) -> { ok | error , float()}.
get_tx_power(Instance) ->
  gproc_call(Instance, {get, tx_power_dbm}).

-spec get_rx_power_mw(xfpInstance()) -> { ok | error , float()}.
get_rx_power_mw(Instance) ->
  gproc_call(Instance, {get, rx_power_mw}).

-spec get_rx_power(xfpInstance()) -> { ok | error , float()}.
get_rx_power(Instance) ->
  gproc_call(Instance, {get, rx_power_dbm}).

-spec get_rx_los(xfpInstance()) -> { ok | error , integer()}.
get_rx_los(Instance) ->
  gproc_call(Instance, {get, rx_los}).

-spec get_laser_state(xfpInstance()) -> { ok | error , integer()}.
get_laser_state(Instance) ->
  gproc_call(Instance, {get, laser_state}).

-spec get_xfp_not_ready(xfpInstance()) -> { ok | error , integer()}.
get_xfp_not_ready(Instance) ->
  gproc_call(Instance, {get, xfp_not_ready}).

-spec set_laser_on(xfpInstance()) -> { ok | error , integer()}.
set_laser_on(Instance) ->
  gproc_call(Instance, {set, laser_on}).

-spec set_laser_off(xfpInstance()) -> { ok | error , integer()}.
set_laser_off(Instance) ->
  gproc_call(Instance, {set, laser_off}).

-spec set_reset(xfpInstance()) -> { ok | error , integer()}.
set_reset(Instance) ->
  gproc_call(Instance, {set, reset}).

%%%===================================================================
%%% Internal functions
%%%===================================================================

%%--------------------------------------------------------------------
%% @private Get the current presence state of the device
%%--------------------------------------------------------------------
get_presence_priv(S) ->
  {ok, Presence} = xfp_driver:read_pin(S#xfp_data.instance, ?XFP_PIN_RESENCE),
  pin_presence_analyse(Presence).

%%--------------------------------------------------------------------
%% @private This function translates io state to true/false
%%--------------------------------------------------------------------
pin_presence_analyse(?XFP_PRESENT) -> {ok, true};
pin_presence_analyse(_) -> {ok, false}.

%%--------------------------------------------------------------------
%% @private Update xfp data if needed
%%--------------------------------------------------------------------
%% Update the xfp data from not present to present
update_xfp_presence(absent, true) ->
  inserted;
%% Update the xfp data from present to not present
update_xfp_presence(inserted, false) ->
  absent;
%% No change of state
update_xfp_presence(State, _) ->
  State.

%%--------------------------------------------------------------------
%% @private Get the current state of the device
%%--------------------------------------------------------------------
upload_xfp_static_information(S) ->
  %% 1 byte information
  {ok, Id} = xfp_driver:read_register(S#xfp_data.instance, ?XFP_REG_IDENTIFIER),
  {ok, Diagnostic} = xfp_driver:read_register(S#xfp_data.instance, ?XFP_REG_DIAGNOSTIC),
  {ok, Enhanced} = xfp_driver:read_register(S#xfp_data.instance, ?XFP_REG_ENHANCED),
  {ok, Aux_mon} = xfp_driver:read_register(S#xfp_data.instance, ?XFP_REG_AUX_MONITORING),
  {ok, Cdr_sup} = xfp_driver:read_register(S#xfp_data.instance, ?XFP_REG_CDR_SUP),
  %% >1 byte information
  VendorName = read_xfp_string(S#xfp_data.instance,
                               ?XFP_REG_VENDOR_NAME,
                               ?XFP_REG_VENDOR_NAME_SIZE),
  VendorPart = read_xfp_string(S#xfp_data.instance,
                               ?XFP_REG_PART_NUMBER,
                               ?XFP_REG_PART_NUMBER_SIZE),
  VendorSerial = read_xfp_string(S#xfp_data.instance,
                                 ?XFP_REG_VENDOR_SERIAL,
                                 ?XFP_REG_VENDOR_SERIAL_SIZE),
  VendorDateCode = read_xfp_string(S#xfp_data.instance,
                                   ?XFP_REG_DATE_CODE,
                                   ?XFP_REG_DATE_CODE_SIZE),
  Revision = read_xfp_string(S#xfp_data.instance,
                             ?XFP_REG_REVISION,
                             ?XFP_REG_REVISION_SIZE),
  %% For Vendor OUI, we capture the list and convert
  VendorOuiList = read_xfp_string(S#xfp_data.instance,
                                  ?XFP_REG_VENDOR_OUI,
                                  ?XFP_REG_VENDOR_OUI_SIZE),
  VendorOui = convert_list_to(oui, VendorOuiList),
  %% For Wavelength, we capture the list and convert
  WaveList = read_xfp_string(S#xfp_data.instance,
                             ?XFP_REG_WAVELENGTH,
                             ?XFP_REG_WAVELENGTH_SIZE),
  Wavelength = convert_list_to(wavelenth, WaveList),
  %% Update all information but instance
  S#xfp_data {
    identifier = Id,
    vendor_name = VendorName,
    cdr_sup = Cdr_sup,
    vendor_oui = VendorOui,
    part_number = VendorPart,
    revision = Revision,
    wavelength = Wavelength,
    vendor_serial = VendorSerial,
    data_code = VendorDateCode,
    diagnostic = Diagnostic,
    enhanced = Enhanced,
    aux_monitoring = Aux_mon
  }.

%%--------------------------------------------------------------------
%% @private Read XFP information
%%--------------------------------------------------------------------
read_priv(temperature, Instance) ->
  TempList = read_xfp_string(Instance, ?XFP_REG_TEMPERATURE,
                                       ?XFP_REG_TEMPERATURE_SIZE),
  Temp = convert_list_to(temperature, TempList),
  {ok, Temp};

read_priv(tx_bias, Instance) ->
  TxBiasList = read_xfp_string(Instance, ?XFP_REG_TX_BIAS,
                                         ?XFP_REG_TX_BIAS_SIZE),
  TxBias = convert_list_to(tx_bias, TxBiasList),
  {ok, TxBias};

read_priv(tx_power_mw, Instance) ->
  TxList = read_xfp_string(Instance, ?XFP_REG_TX_POWER,
                                     ?XFP_REG_TX_POWER_SIZE),
  TxPower = convert_list_to(power_mw, TxList),
  {ok, TxPower};

read_priv(tx_power_dbm, Instance) ->
  TxList = read_xfp_string(Instance, ?XFP_REG_TX_POWER,
                                     ?XFP_REG_TX_POWER_SIZE),
  TxPower = convert_list_to(power_dbm, TxList),
  {ok, TxPower};

read_priv(rx_power_mw, Instance) ->
  RxList = read_xfp_string(Instance, ?XFP_REG_RX_POWER,
                                     ?XFP_REG_RX_POWER_SIZE),
  RxPower = convert_list_to(power_mw, RxList),
  {ok, RxPower};

read_priv(rx_power_dbm, Instance) ->
  RxList = read_xfp_string(Instance, ?XFP_REG_RX_POWER,
                                     ?XFP_REG_RX_POWER_SIZE),
  RxPower = convert_list_to(power_dbm, RxList),
  {ok, RxPower};

read_priv(rx_los, Instance) ->
  xfp_driver:read_pin(Instance, ?XFP_PIN_RX_LOS);

read_priv(laser_state, Instance) ->
  {ok, Tx_Dis} = xfp_driver:read_pin(Instance, ?XFP_PIN_TX_DIS),
  {ok, invert_data(Tx_Dis)};

read_priv(xfp_not_ready, Instance) ->
  xfp_driver:read_pin(Instance, ?XFP_PIN_NOT_READY).

%%--------------------------------------------------------------------
%% @private Write XFP information
%%--------------------------------------------------------------------
write_priv(laser_on, Instance) ->
  xfp_driver:write_pin(Instance, ?XFP_PIN_TX_DIS, 0);

write_priv(laser_off, Instance) ->
  xfp_driver:write_pin(Instance, ?XFP_PIN_TX_DIS, 1);

write_priv(reset, Instance) ->
  xfp_driver:write_pin(Instance, ?XFP_PIN_RESET, 0),
  timer:sleep(?XFP_RESET_DELAY),
  xfp_driver:write_pin(Instance, ?XFP_PIN_RESET, 1).

%%--------------------------------------------------------------------
%% @private Read string data and compose a list
%%--------------------------------------------------------------------
read_xfp_string(Instance, Register, Size) ->
  lists:foldr( fun (Elem, Acc) ->
                  {ok, Value} = xfp_driver:read_register(Instance, Register+Elem),
                  [Value | Acc]
               end,
               [],
               lists:seq(0,Size - 1)).

%%--------------------------------------------------------------------
%% @private Convert a List to a unsigned number
%%--------------------------------------------------------------------
list_to_unumber(List,Bits) ->
  <<Num:Bits/unsigned>> = erlang:list_to_binary(List),
  Num.

%%--------------------------------------------------------------------
%% @private Convert a List to a signed number
%%--------------------------------------------------------------------
list_to_snumber(List,Bits) ->
  <<Num:Bits/signed>> = erlang:list_to_binary(List),
  Num.

%%--------------------------------------------------------------------
%% @private Convert a List to a specific value
%%--------------------------------------------------------------------
%% in Celsius
convert_list_to(temperature, L) ->
  list_to_snumber(L,?XFP_REG_SIZE_BITS) / 256;
%% in mA
convert_list_to(tx_bias, L) ->
  list_to_unumber(L,?XFP_REG_SIZE_BITS) / 500;
%% in mw
convert_list_to(power_mw, L) ->
  list_to_unumber(L,?XFP_REG_SIZE_BITS) / 10000;
%% in Dbm 
convert_list_to(power_dbm, L) ->
  case convert_list_to(power_mw, L) of
    X when X > 0 -> 10 * math:log10(X);
    _ -> ?XFP_DBM_MIN
   end;
%% in nm
convert_list_to(wavelenth, L) ->
  list_to_unumber(L,?XFP_REG_SIZE_BITS) / 20;
%% generic number
convert_list_to(oui, L) ->
  list_to_unumber(L,?XFP_REG_OUI_SIZE_BITS).

%%--------------------------------------------------------------------
%% @private Invert the data value
%%--------------------------------------------------------------------
invert_data(0) ->
  1;
invert_data(1) ->
  0.

%%--------------------------------------------------------------------
%% @private Send a gen_statem:call message if the PID is found
%%--------------------------------------------------------------------
gproc_call(Instance, Msg) ->
  Key = {?MODULE, Instance},
  case gproc:lookup_pids({p, l, Key}) of
    [Pid] -> gen_statem:call(Pid, Msg);
    _ -> {error, invalid_xfp}
  end.
















