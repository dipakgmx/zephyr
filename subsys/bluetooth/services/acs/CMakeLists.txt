# SPDX-License-Identifier: Apache-2.0

# Core — always compiled when BT_ACS is enabled.
zephyr_sources_ifdef(
  CONFIG_BT_ACS
  acs_seg.c
  acs_channel_rx.c
  acs.c
  acs_conn.c
  acs_session.c
  acs_runtime.c
  acs_reply_seq.c
  acs_cp.c
  acs_cp_feature.c
  acs_cp_descriptors.c
  acs_rhandle.c
)

# Security/authentication base — needed when any crypto protection is active.
zephyr_sources_ifdef(
  CONFIG_BT_ACS_FEAT_AUTHENTICATION
  acs_isc.c
  acs_sec_mgmt.c
  acs_request.c
  acs_data_in.c
  acs_data_out.c
)

# Key exchange — compiled when any KEx method is enabled.
zephyr_sources_ifdef(
  CONFIG_BT_ACS_ANY_KEY_EXCHANGE
  acs_key_exchange.c
  acs_key_desc.c
  acs_cp_kex.c
)

# Data protection crypto — compiled when any algorithm is enabled.
zephyr_sources_ifdef(
  CONFIG_BT_ACS_ANY_DATA_PROTECTION
  acs_crypto.c
  acs_crypto_keys.c
  acs_crypto_alg.c
)

# GATT authorization callback — only when auto-gating is enabled.
zephyr_sources_ifdef(CONFIG_BT_ACS_GATT_AUTHORIZATION acs_policy.c)

# Restriction map data and CP handlers — only when authorization is compiled in.
zephyr_sources_ifdef(
  CONFIG_BT_ACS_FEAT_AUTHORIZATION
  acs_rmap.c
  acs_cp_rmap.c
  acs_cp_all_active.c
)
