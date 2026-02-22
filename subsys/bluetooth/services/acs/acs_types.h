/*
 * Copyright (c) 2025 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef BT_GATT_ACS_TYPES_H_
#define BT_GATT_ACS_TYPES_H_

#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/math_extras.h>
#include <zephyr/sys/slist.h>
#include <zephyr/net_buf.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/services/acs.h>
#include <psa/crypto.h>

#include "acs_seg.h"
#include "acs_cp_operands.h"
#include "acs_key_desc.h"
#include "acs_wire_constants.h"

#ifdef __cplusplus
extern "C" {
#endif

struct acs_reply;
struct bt_acs_conn;
struct bt_acs_rmap_runtime;

/* Source characteristic of an inbound frame. */
enum acs_source_channel {
	ACS_SRC_CP = 0,      /* ACS Control Point write */
	ACS_SRC_DATA_IN = 1, /* ACS Data In write */
};

/* Inbound request and routing information. */
struct acs_frame {
	struct bt_acs_key_desc_runtime *key_runtime; /* Protection key, NULL for plain CP */
	const uint8_t *payload;                      /* Payload in the RX buffer */
	uint16_t payload_len;                        /* Payload length */
	uint16_t resource_handle;                    /* Resource Handle, 0 for plain CP */
	uint16_t isc_id;                             /* ISC_ID, 0 for plain CP */
	enum acs_source_channel source_channel;      /* Source characteristic */
};

/* Plain ACS Control Point frame from a reassembled request. */
static inline struct acs_frame acs_frame_from_cp_rx(struct net_buf *rx_buf)
{
	return (struct acs_frame){
		.resource_handle = 0,
		.isc_id = 0,
		.key_runtime = NULL,
		.payload = rx_buf->data,
		.payload_len = rx_buf->len,
		.source_channel = ACS_SRC_CP,
	};
}

/* Route a protected payload takes once decrypted. */
enum acs_route_kind {
	ACS_ROUTE_PROTECTED_ACS_CP = 0,  /* ACS Control Point procedure */
	ACS_ROUTE_PROTECTED_EXTERNAL_CP, /* Control Point of another service */
	ACS_ROUTE_PROTECTED_READ,        /* Characteristic read */
	ACS_ROUTE_PROTECTED_WRITE,       /* Characteristic write */
};

/* Output channel of an ACS response. */
enum acs_reply_channel {
	ACS_REPLY_CP = 0,  /* ACS Control Point, plaintext indication */
	ACS_REPLY_DON = 1, /* Data Out Notify, encrypted notification */
	ACS_REPLY_DOI = 2, /* Data Out Indicate, encrypted indication */
};

/* Continuation of an ACS Control Point response sequence (Section 4.4.3). */
enum acs_reply_step {
	ACS_REPLY_DONE = 0,     /* Response sequence complete */
	ACS_REPLY_KEX_OK,       /* Send the Key Exchange Response */
	ACS_REPLY_KEX_COMPLETE, /* Make the exchanged keys usable */
	ACS_REPLY_DESCS_ISC,    /* Send the ISC Descriptor Response */
	ACS_REPLY_DESCS_KEY,    /* Send the Key Descriptor Response */
	ACS_REPLY_DESCS_RC,     /* Send the closing Response Code */
	ACS_REPLY_INVALIDATE,   /* Remove a key after sending the response */
};

/* Per-connection runtime state for a key descriptor record. */
struct bt_acs_key_desc_runtime {
	uint64_t tx_nonce_counter; /* Next TX sequence number */
	uint64_t rx_nonce_counter; /* Lowest RX sequence number still accepted */
	const struct bt_acs_key_desc_record *key_desc; /* Bound descriptor record */
	psa_key_id_t psa_key_id;                       /* Operational PSA key ID, 0 if absent */
	psa_key_id_t derive_key_id; /* Derivation key ID for exchange slots, 0 if absent */
	psa_algorithm_t psa_alg;    /* Algorithm for record slots, 0 for exchange slots */
	uint16_t current_key_id;    /* Parent Key_ID for record slots, 0 for exchange slots */
	uint8_t server_nonce_fixed[ACS_MAX_NONCE_PREFIX_SIZE]; /* Server prefix, MSO order */
	uint8_t client_nonce_fixed[ACS_MAX_NONCE_PREFIX_SIZE]; /* Client prefix, MSO order */
	bool server_nonce_set;                                 /* Server prefix available */
	bool client_nonce_set;                                 /* Client prefix available */
};

/* Key_ID bound to a runtime slot, or 0 if the slot is unbound. */
static inline uint16_t
acs_key_desc_runtime_key_id(const struct bt_acs_key_desc_runtime *key_desc_runtime)
{
	return (key_desc_runtime != NULL && key_desc_runtime->key_desc != NULL)
		       ? key_desc_runtime->key_desc->key_id
		       : 0U;
}

/* Number of runtime slots reserved for enabled algorithm records. */
#define ACS_ALGO_RECORD_COUNT                                                                      \
	(IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_GCM) +                                       \
	 IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_CCM) +                                       \
	 IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_GMAC) +                                      \
	 IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_CMAC))

/* Number of runtime slots reserved for exchange and algorithm records. */
#define ACS_KEY_RUNTIME_COUNT (ACS_KEY_ID_COUNT + ACS_ALGO_RECORD_COUNT)

/* Per-connection key-descriptor runtime slots. */
struct bt_acs_crypto_state {
	struct bt_acs_key_desc_runtime key_runtimes[ACS_KEY_RUNTIME_COUNT]; /* Runtime slots */
};

/* Plain ECDH public-key fields (Tables 4.66 and 4.69). */
struct acs_ecdh_pubkey {
	uint16_t key_id;                /* Key_ID */
	uint8_t x_size;                 /* X_Coordinate_Size */
	uint8_t x[ACS_ECDH_COORD_SIZE]; /* X_Coordinate */
	uint8_t y_size;                 /* Y_Coordinate_Size */
	uint8_t y[ACS_ECDH_COORD_SIZE]; /* Y_Coordinate */
} __packed;

/* Minimum operand length with nonempty X and Y coordinates (Table 4.66). */
#define ACS_ECDH_PUBKEY_MIN_OPERAND (offsetof(struct acs_ecdh_pubkey, x) + 3U)

/* Key Exchange KDF Response parameters (Table 4.76). */
struct bt_acs_kdf_params {
	uint8_t salt_size;               /* KDF_Salt_Size */
	uint8_t salt[ACS_KDF_SALT_SIZE]; /* KDF_Salt */
	uint8_t info_size;               /* KDF_Info_Size */
	uint8_t info[ACS_KDF_INFO_SIZE]; /* KDF_Info */
};

/* Key-exchange request accepted in the current exchange state. */
enum acs_kex_state {
	ACS_KEX_AWAIT_PUBKEY = 0,   /* Key Exchange ECDH */
	ACS_KEX_AWAIT_KDF,          /* Key Exchange KDF */
	ACS_KEX_AWAIT_CONFIRM_CODE, /* ECDH Confirmation Code */
	ACS_KEX_AWAIT_CONFIRM_RAND, /* ECDH Confirmation Random Number */
};

/* State of an in-progress key exchange. */
struct bt_acs_kex_ctx {
	enum acs_kex_state state; /* Expected request */
#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHENTICATION)
	struct bt_acs_kdf_params kdf; /* KDF response parameters */
#endif
	struct acs_cp_start_key_exchange_req start_kex; /* Start Key Exchange operand */
	uint8_t auth_value[ACS_CONFIRM_VALUE_SIZE];     /* Authentication value */
#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHENTICATION)
	struct acs_ecdh_pubkey server_pubkey;           /* Server ephemeral public key */
	struct acs_ecdh_pubkey client_pubkey;           /* Client public key */
	psa_key_id_t ecdh_key_id;                       /* Server ephemeral private-key ID */
	psa_key_id_t derived_key_id;                    /* ECDH shared-secret key ID */
	uint8_t server_random[ACS_CONFIRM_VALUE_SIZE];  /* Server random nonce */
	uint8_t client_confirm[ACS_CONFIRM_VALUE_SIZE]; /* Client confirmation code */
#endif
};

/* Access performed for a protected request. */
enum acs_req_access {
	ACS_REQ_ACCESS_UNKNOWN = 0, /* Not a protected characteristic request */
	ACS_REQ_ACCESS_READ,        /* Characteristic read, answered on DON */
	ACS_REQ_ACCESS_WRITE,       /* Characteristic write, answered on DON */
	ACS_REQ_ACCESS_CP_WRITE,    /* External service Control Point write, no reply */
};

/* General reply slots plus one slot reserved for Abort (§4.4.5). */
#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHENTICATION)
#define ACS_REPLY_SLOTS (1 + CONFIG_BT_ACS_MAX_INFLIGHT_REQ_PER_CONN)
#else
#define ACS_REPLY_SLOTS 1
#endif
#define ACS_REPLY_ABORT_SLOT  ACS_REPLY_SLOTS
#define ACS_REPLY_TOTAL_SLOTS (ACS_REPLY_SLOTS + 1)

/* Outbound response released by acs_reply_free(). */
struct acs_reply {
	sys_snode_t node;                            /* Queue linkage */
	struct bt_acs_conn *conn;                    /* Owning connection */
	struct net_buf *request;                     /* Request, NULL for server output */
	struct net_buf *response;                    /* Response being built */
	enum acs_req_access access;                  /* Protected request access */
	enum acs_reply_channel channel;              /* Characteristic the reply goes out on */
	enum acs_reply_step step;                    /* Continuation after this transfer */
	bool aborted;                                /* Procedure aborted mid-transfer */
	bool holds_cp_lock;                          /* Reply holds the CP lock */
	uint16_t resource_handle;                    /* Resource Handle, 0 for plain CP */
	uint16_t attr_handle;                        /* Backing GATT Attribute Handle */
	const struct bt_gatt_attr *value_attr;       /* Resolved GATT value attribute */
	uint8_t value_props;                         /* Cached characteristic properties */
	uint16_t isc_id;                             /* ISC_ID, 0 for plain CP */
	uint16_t invalidate_key_id;                  /* Key_ID to remove after the response */
	const struct bt_acs_rmap_runtime *rmap;      /* Map used for this request */
	struct bt_acs_key_desc_runtime *key_runtime; /* Response protection key */
	bt_acs_output_func_t output_func;            /* Server output completion callback */
	void *output_user_data;                      /* Completion callback data */
};

#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHENTICATION)
/*
 * Data Out Notify or Data Out Indicate transmission channel.
 *
 * active is accessed only on the ACS workqueue.
 */
struct acs_tx_channel {
	struct k_fifo fifo;                 /* Replies awaiting transmission */
	struct k_work drain_work;           /* Queue processing work */
	struct acs_seg_tx_ctx tx;           /* Segmented transmission state */
	struct acs_reply *active;           /* Reply currently being transmitted */
	const struct bt_gatt_attr *attr;    /* Data Out characteristic */
	struct bt_acs_conn *conn;           /* Owning connection */
	acs_seg_tx_completion_cb_t done_cb; /* Channel completion handler */
	const char *name;                   /* Channel name used for logging */
};
#endif /* CONFIG_BT_ACS_FEAT_AUTHENTICATION */

/* Inbound frame reconstructed from a queued reply. */
static inline struct acs_frame acs_frame_from_reply(const struct acs_reply *reply)
{
	return (struct acs_frame){
		.resource_handle = reply->resource_handle,
		.isc_id = reply->isc_id,
		.key_runtime = reply->key_runtime,
		.payload = reply->request->data,
		.payload_len = reply->request->len,
		.source_channel = (reply->channel == ACS_REPLY_CP) ? ACS_SRC_CP : ACS_SRC_DATA_IN,
	};
}

/*
 * Internal bit positions in bt_acs_conn::state. acs_status_fill() translates
 * the security-established bit to Status_Flags (Table 4.7).
 */
enum acs_conn_state_bit {
	ACS_STATE_SECURITY_ESTABLISHED = 0, /* Security established by key exchange */
	ACS_STATE_CP_LOCKED,                /* ACS Control Point locked */
	ACS_STATE_CP_SERVER_TX,             /* Lock held by a server indication */
	ACS_STATE_INVALIDATE_PENDING,       /* Key removal waiting for the response */
	ACS_STATE_ABORT_REQUESTED,          /* Abort requested */
	ACS_STATE_ABORT_HAD_WORK,           /* Abort found an active procedure */
	ACS_STATE_STATUS_PENDING,           /* Status indication pending */
	ACS_STATE_STATUS_BUSY,              /* Status indication in flight */
};

/* ACS runtime state for one connection. */
struct bt_acs_conn {
	struct bt_conn *conn; /* Connection assigned to this slot, NULL when inactive */
	atomic_t state;       /* Connection-state bits */
	struct bt_acs_crypto_state crypto;    /* Key runtime state */
	struct bt_acs_kex_ctx *kex;           /* Active key exchange, NULL when idle */
	uint8_t status_data[ACS_STATUS_SIZE]; /* Status indication value */
	struct bt_gatt_indicate_params status_indicate_params; /* Status indication parameters */
	struct k_work_delayable status_work;                /* Status indication and retry work */
	struct k_work abort_work;                           /* Abort evaluation work */
	struct acs_reply replies[ACS_REPLY_TOTAL_SLOTS];    /* Reply pool */
	ATOMIC_DEFINE(reply_in_use, ACS_REPLY_TOTAL_SLOTS); /* Allocated reply slots */
	struct acs_seg_tx_ctx cp_tx;                        /* Plain Control Point TX state */
	struct acs_seg_rx_ctx cp_rx;                        /* Control Point RX state */
	struct acs_seg_rx_ctx data_rx;                      /* Data In RX state */
	struct k_fifo cp_exec_fifo;   /* Queued ACS Control Point procedures */
	struct k_work cp_exec_work;   /* ACS Control Point execution work */
	struct acs_reply *cp_pending; /* Response awaiting ACS Control Point TX */
#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHENTICATION)
	struct k_fifo request_fifo;         /* Queued protected requests */
	struct k_work request_work;         /* Protected request execution work */
	struct k_work invalidate_work;      /* Key removal work */
	uint16_t remove_after_reply_key_id; /* Key_ID to remove after the response */
	struct acs_tx_channel doi;          /* Data Out Indicate TX channel */
	struct acs_tx_channel don;          /* Data Out Notify TX channel */
#endif
};

#ifdef __cplusplus
}
#endif

#endif /* BT_GATT_ACS_TYPES_H_ */
