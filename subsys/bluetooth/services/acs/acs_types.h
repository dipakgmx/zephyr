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

/* Inbound request and routing information. */
struct acs_frame {
	struct acs_sec_alg *alg;  /* Security algorithm, NULL for plain CP */
	const uint8_t *payload;   /* Payload in the RX buffer */
	uint16_t payload_len;     /* Payload length */
	uint16_t resource_handle; /* Resource Handle, 0 for plain CP */
	uint16_t isc_id;          /* ISC_ID, 0 for plain CP */
};

/* Output channel of an ACS response. */
enum acs_reply_channel {
	ACS_REPLY_CP = 0,  /* ACS Control Point, plaintext indication */
	ACS_REPLY_DON = 1, /* Data Out Notify, encrypted notification */
	ACS_REPLY_DOI = 2, /* Data Out Indicate, encrypted indication */
};

/*
 * Per-connection state of a security algorithm record (Table 4.45): the parent
 * key under this algorithm's PSA policy, and the algorithm's nonce state.
 */
struct acs_sec_alg {
	uint64_t tx_nonce_counter;                             /* Next TX sequence number */
	uint64_t rx_nonce_counter;                             /* Lowest RX sequence accepted */
	const struct bt_acs_key_desc_record *key_desc;         /* Security algorithm record */
	psa_key_id_t psa_key_id;                               /* PSA key ID, 0 if absent */
	psa_algorithm_t psa_alg;                               /* PSA algorithm of the key */
	uint8_t server_nonce_fixed[ACS_MAX_NONCE_PREFIX_SIZE]; /* Server prefix, MSO order */
	uint8_t client_nonce_fixed[ACS_MAX_NONCE_PREFIX_SIZE]; /* Client prefix, MSO order */
	bool server_nonce_set;                                 /* Server prefix available */
	bool client_nonce_set;                                 /* Client prefix available */
	bool client_nonce_restored;                            /* Client prefix from storage */
};

/* Key_ID of the security algorithm record. */
static inline uint16_t acs_sec_alg_id(const struct acs_sec_alg *alg)
{
	return alg->key_desc->key_id;
}

/* Return true when alg is present and holds key material. */
static inline bool acs_sec_alg_ready(const struct acs_sec_alg *alg)
{
	return alg != NULL && alg->psa_key_id != PSA_KEY_ID_NULL;
}

/* Number of security algorithm records, one per enabled algorithm. */
#define ACS_SEC_ALG_COUNT                                                                          \
	(IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_GCM) +                                       \
	 IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_CCM) +                                       \
	 IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_GMAC) +                                      \
	 IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_CMAC))

/* Number of Key_IDs a connection can hold: ECDH, KDF and the algorithm records. */
#define ACS_KEY_COUNT (ACS_KEY_ID_COUNT + ACS_SEC_ALG_COUNT)

/*
 * Keys of one connection. The ECDH key is the root and the KDF exchange derives
 * the key of every security algorithm from it, so removing a key removes those
 * below it.
 */
struct acs_keys {
	psa_key_id_t ecdh;                          /* ECDH exchange key, 0 if absent */
	bool kdf;                                   /* KDF exchange completed */
	struct acs_sec_alg algs[ACS_SEC_ALG_COUNT]; /* Security algorithms */
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

/* next_opcode of a key exchange whose last step succeeded; an AC Client never sends it. */
#define ACS_KEX_COMPLETE BT_ACS_CP_OPCODE_RESPONSE_CODE

/*
 * In-progress key exchange (§4.4.3.17). The AC Client sends these requests in
 * order, and next_opcode is the one it must send next:
 *   ECDH: Start Key Exchange -> Key Exchange ECDH -> Key Exchange KDF
 *         -> ECDH Confirmation Code -> ECDH Confirmation Random Number
 *   KDF:  Start Key Exchange -> Key Exchange KDF
 * After the last step next_opcode is ACS_KEX_COMPLETE, and the Key Exchange
 * Response goes out after that step's response.
 */
struct bt_acs_kex_ctx {
	uint8_t next_opcode;                        /* Request the AC Client sends next */
	uint8_t confirmation_method;                /* Selected_Confirmation_Method (Table 4.50) */
	uint16_t key_id;                            /* Key_ID of the Start Key Exchange */
	uint8_t auth_value[ACS_CONFIRM_VALUE_SIZE]; /* Authentication value */
#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHENTICATION)
	struct acs_ecdh_pubkey server_pubkey;           /* Server ephemeral public key */
	struct acs_ecdh_pubkey client_pubkey;           /* Client public key */
	psa_key_id_t ecdh_key_id;                       /* Server ephemeral private-key ID */
	psa_key_id_t derived_key_id;                    /* ECDH shared-secret key ID */
	uint8_t server_random[ACS_CONFIRM_VALUE_SIZE];  /* Server random nonce */
	uint8_t client_confirm[ACS_CONFIRM_VALUE_SIZE]; /* Client confirmation code */
#endif
};

/* Outbound response released by acs_reply_free(). */
struct acs_reply {
	sys_snode_t node;                      /* Queue linkage */
	struct bt_acs_conn *acs_conn;          /* Owning connection */
	struct net_buf *request;               /* Request, NULL for server output */
	struct net_buf *response;              /* Messages to send, one fragment each */
	struct net_buf *tx_msg;                /* Message being sent */
	enum acs_reply_channel channel;        /* Characteristic the reply goes out on */
	bool aborted;                          /* Procedure aborted mid-transfer */
	bool holds_cp_lock;                    /* Reply holds the CP lock */
	uint16_t resource_handle;              /* Resource Handle, 0 for plain CP */
	const struct bt_gatt_attr *value_attr; /* Resolved GATT value attribute */
	uint8_t value_props;                   /* Cached characteristic properties */
	uint16_t isc_id;                       /* ISC_ID, selects the algorithm; 0 for plain CP */
	uint16_t invalidate_key_id;            /* Key_ID to remove once delivered */
	struct bt_gatt_indicate_params *ind;   /* Protected indication, else NULL */
	bt_gatt_complete_func_t ntf_func;      /* Protected notification callback, else NULL */
	void *ntf_user_data;                   /* Protected notification callback data */
};

#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHENTICATION)
/*
 * Data Out Notify or Data Out Indicate transmission channel.
 *
 * active is accessed only on the ACS workqueue.
 */
struct acs_tx_channel {
	struct k_fifo fifo;       /* Replies awaiting transmission */
	struct k_work drain_work; /* Queue processing work */
	struct acs_seg_tx_ctx tx; /* Segmented transmission state */
	struct acs_reply *active; /* Reply currently being transmitted */
};
#endif /* CONFIG_BT_ACS_FEAT_AUTHENTICATION */

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
	ACS_STATE_DISCONNECTED,             /* Link gone, teardown work queued */
};

/* ACS runtime state for one connection. */
struct bt_acs_conn {
	struct bt_conn *conn;       /* Referenced connection of this slot, NULL when inactive */
	atomic_t state;             /* Connection-state bits */
	struct acs_keys keys;       /* Keys of this connection */
	struct bt_acs_kex_ctx *kex; /* Active key exchange, NULL when idle */
	uint8_t status_data[ACS_STATUS_SIZE];                  /* Status indication value */
	struct bt_gatt_indicate_params status_indicate_params; /* Status indication parameters */
	struct k_work_delayable status_work; /* Status indication and retry work */
	struct k_work abort_work;            /* Abort evaluation work */
	struct k_work teardown_work;         /* Disconnect teardown work */
	/* Each reply holds at least one buffer, so buffers limit the replies in use. */
	struct acs_reply replies[ACS_BUF_PER_CONN];    /* Reply pool */
	ATOMIC_DEFINE(reply_in_use, ACS_BUF_PER_CONN); /* Allocated reply slots */
	struct acs_seg_tx_ctx cp_tx;                   /* Plain Control Point TX state */
	struct acs_seg_rx_ctx cp_rx;                   /* Control Point RX state */
	struct acs_seg_rx_ctx data_rx;                 /* Data In RX state */
	struct k_fifo cp_exec_fifo;                    /* Queued ACS Control Point procedures */
	struct k_work cp_exec_work;                    /* ACS Control Point execution work */
	struct acs_reply *cp_pending;                  /* Response awaiting ACS Control Point TX */
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
