# SPDX-License-Identifier: Apache-2.0

zephyr_sources_ifdef(
  CONFIG_BT_ACS
  acs_seg.c
  acs_persist.c
  acs_crypto.c
  acs_kex.c
  acs_cp_domain.c
  acs_runtime.c
  acs_service.c
  acs_channel_rx.c
  acs_protected_resource_router.c
  acs_procedure_engine.c
  acs_data_out_channel.c
  acs_service_adapter.c
)
