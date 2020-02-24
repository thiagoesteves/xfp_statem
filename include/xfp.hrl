%%%=============================================================================
%%% Created : 24 Feb 2020 by Thiago Esteves <thiagocalori@gmail.com>
%%%=============================================================================

-ifndef(xfp).
-define(xfp, true).

%% Xfp Defines
-type xfpInstance() :: 0..20.

%% static information records
-record(xfp_data, {
  instance,
  identifier,
  vendor_name,
  cdr_sup,
  vendor_oui,
  part_number,
  revision,
  wavelength,
  vendor_serial,
  data_code,
  diagnostic,
  enhanced,
  aux_monitoring
}).


-endif. %% xfp