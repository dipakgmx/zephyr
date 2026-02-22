# SPDX-License-Identifier: Apache-2.0

# Core — always compiled when BT_ACS is enabled.
zephyr_sources_ifdef(
  CONFIG_BT_ACS
  acs_seg.c
  acs.c
  acs_conn.c
  acs_runtime.c
  acs_reply.c
  acs_data_out.c
  acs_cp.c
  acs_cp_feature.c
  acs_rhandle.c
)

# Authentication — key exchange, protected Data In/Out, data-protection crypto,
# and persistent parent-key storage all come together with authentication support.
zephyr_sources_ifdef(
  CONFIG_BT_ACS_FEAT_AUTHENTICATION
  acs_isc.c
  acs_sec_mgmt.c
  acs_request.c
  acs_data_in.c
  acs_key_exchange.c
  acs_key_desc.c
  acs_cp_kex.c
  acs_crypto.c
  acs_crypto_alg.c
  acs_key_store.c
)

# Authorization — restriction maps, the descriptor layer, and the GATT
# authorization callback that enforces the map on plain ATT access.
zephyr_sources_ifdef(
  CONFIG_BT_ACS_FEAT_AUTHORIZATION
  acs_policy.c
  acs_rmap.c
  acs_descs.c
)
