/*
 * Copyright (c) 2025 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file acs_types.h
 * @brief ACS internal data structures, enumerations, and forward declarations.
 *
 * All core ACS types live here to keep the circular acs_reply <-> bt_acs_conn
 * dependency resolved in one place. Included transitively via acs_internal.h.
 */

#ifndef BT_GATT_ACS_TYPES_H_
#define BT_GATT_ACS_TYPES_H_

#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/slist.h>
#include <zephyr/net_buf.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/services/acs.h>
#include <psa/crypto.h>

#include "acs_seg.h"
#include "acs_cp.h"
#include "acs_key_desc.h"

#ifdef __cplusplus
extern "C" {
#endif

struct acs_reply;   /**< Defined below; forward-declared for bt_acs_conn */
struct bt_acs_conn; /**< Defined below; forward-declared for struct acs_reply */

/**
 * @brief Source GATT characteristic that produced an inbound frame.
 *
 * Set by the GATT entry layer and consumed by the runtime to decide whether
 * the frame requires unwrap (Data In) or can be classified directly (plain CP).
 */
enum acs_source_channel {
	ACS_SRC_CP = 0,      /**< ACS Control Point write */
	ACS_SRC_DATA_IN = 1, /**< ACS Data In write (carries encrypted transport payload) */
};

/**
 * @brief Normalized inbound request - handoff object from transport decode into runtime dispatch.
 *
 * Built once acs_channel_rx_feed() reports COMPLETE. @c payload points into
 * the reassembled channel-RX buffer, whose lifetime the runtime layer owns
 * through @c cp_rx / @c data_rx. For Data In, the unwrap helper trims the
 * buffer to the inner plaintext and fills @c resource_handle / @c isc_id
 * before the frame is built.
 */
struct acs_frame {
	uint16_t resource_handle;               /**< Inner resource handle (0 for plain CP) */
	uint16_t isc_id;                        /**< ISC_ID (0 for plain CP) */
	const uint8_t *payload;                 /**< Pointer to current logical payload */
	uint16_t payload_len;                   /**< Length of @p payload */
	enum acs_source_channel source_channel; /**< Originating ACS characteristic */
};

/** @brief Build a plain-CP frame from a reassembled segmentation buffer. */
static inline struct acs_frame acs_frame_from_cp_rx(struct net_buf *rx_buf)
{
	return (struct acs_frame){
		.resource_handle = 0,
		.isc_id = 0,
		.payload = rx_buf->data,
		.payload_len = rx_buf->len,
		.source_channel = ACS_SRC_CP,
	};
}

/**
 * @brief Result kind from the frame classifier.
 *
 * The classifier answers "what kind of work is this?" - it does not allocate
 * state and does not dispatch.
 */
enum acs_route_kind {
	ACS_ROUTE_ACS_CP = 0, /**< Plain ACS Control Point opcode */
	ACS_ROUTE_PROTECTED_CHAR =
		1, /**< Protected characteristic request (auto-respond / handler) */
	ACS_ROUTE_PROTECTED_SERVICE_CP = 2, /**< Protected service Control Point request */
};

/**
 * @brief Outbound transport channel for an @ref acs_reply.
 *
 * Distinguishes the three send paths a reply can take:
 *  - @ref ACS_REPLY_CP   - plain ACS Control Point indication (segmented).
 *  - @ref ACS_REPLY_DON  - Data Out Notify (encrypted notification).
 *  - @ref ACS_REPLY_DOI  - Data Out Indicate (encrypted, segmented, confirmed).
 */
enum acs_reply_channel {
	ACS_REPLY_CP = 0,
	ACS_REPLY_DON = 1,
	ACS_REPLY_DOI = 2,
};

/**
 * @brief Continuation step for multi-indication reply sequences.
 *
 * Each value identifies the next action after the current indication confirms.
 * ACS_REPLY_DONE means the reply is complete and can be freed.
 * Maps 1:1 to the spec's multi-indication sequences (§4.4.3).
 */
enum acs_reply_step {
	ACS_REPLY_DONE = 0,
	ACS_REPLY_KEX_OK,       /**< Send Key Exchange Rsp (success) */
	ACS_REPLY_KEX_COMPLETE, /**< Finalize: commit keys, set SECURITY_ESTABLISHED */
	ACS_REPLY_KEX_FAIL,     /**< Send Key Exchange Rsp (failure) */
	ACS_REPLY_KEX_CLEANUP,  /**< Abort KEX context */
	ACS_REPLY_DESCS_ISC,    /**< Send ISC Descriptor Rsp */
	ACS_REPLY_DESCS_KEY,    /**< Send Key Descriptor Rsp */
	ACS_REPLY_DESCS_RC,     /**< Send final Response Code */
	ACS_REPLY_INVALIDATE,   /**< Clear security-established after confirm */
};

/**
 * @brief Unified runtime state for one key descriptor on a connection.
 *
 * Covers both key-exchange records (ECDH, KDF) and algorithm records
 * (GCM, CCM, GMAC, CMAC).  Exchange-key entries use only key_desc and
 * psa_key_id; the nonce fields are unused for them.
 */
struct bt_acs_key_desc_runtime {
	/** Static key descriptor bound to this runtime slot. */
	const struct bt_acs_key_desc_record *key_desc;
	/** Resolved parent exchange-key Key_ID (0 for exchange-key entries). */
	uint16_t current_key_id;
	/** Imported PSA key handle. 0 means no key installed. */
	psa_key_id_t psa_key_id;
	/** Derivation-capable twin for exchange-key slots. */
	psa_key_id_t derive_key_id;
	/** Cached PSA algorithm for this record's encrypt/decrypt ops (0 for exchange keys). */
	psa_algorithm_t psa_alg;
	/** AC Server nonce fixed value for this Key_ID (runtime order for nonce build). */
	uint8_t server_nonce_fixed[ACS_MAX_NONCE_FIXED_SIZE];
	/** AC Client nonce fixed value for this Key_ID (runtime order for nonce build). */
	uint8_t client_nonce_fixed[ACS_MAX_NONCE_FIXED_SIZE];
	/** True once the client has set or restored AC_Client_Nonce_Fixed_Value. */
	bool client_nonce_set;
	/** Server TX nonce-variable counter for this Key_ID. */
	uint64_t tx_nonce_counter;
	/** Client RX nonce-variable counter for this Key_ID. */
	uint64_t rx_nonce_counter;
};

static inline uint16_t
acs_key_desc_runtime_key_id(const struct bt_acs_key_desc_runtime *key_desc_runtime)
{
	return (key_desc_runtime && key_desc_runtime->key_desc) ? key_desc_runtime->key_desc->key_id
								: 0U;
}

/** Number of algorithm-record runtime slots derived from enabled algorithms. */
#define ACS_ALGO_RECORD_COUNT                                                                      \
	(IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_GCM) +                                       \
	 IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_CCM) +                                       \
	 IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_GMAC) +                                      \
	 IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_CMAC))

/** Total unified key-runtime slots: exchange keys + algorithm records. */
#define ACS_KEY_RUNTIME_COUNT (ACS_KEY_ID_COUNT + ACS_ALGO_RECORD_COUNT)

/**
 * @brief Persistent per-connection ACS crypto state.
 */
struct bt_acs_crypto_session {
	/** Unified per-Key_ID runtime state for all key descriptors.
	 *
	 * Exchange-key slots (ECDH, KDF) are bound first, followed by
	 * algorithm-record slots (GCM, CCM, GMAC).  Looked up uniformly via
	 * acs_crypto_key_runtime_lookup().
	 */
	struct bt_acs_key_desc_runtime key_runtimes[ACS_KEY_RUNTIME_COUNT];
};

/**
 * @brief Wire format of an ACS ECDH public key (Tables 4.68-4.69).
 */
struct acs_ecdh_pubkey {
	uint16_t key_id;                          /**< Key identifier */
	uint8_t x_size;                           /**< Size of X coordinate */
	uint8_t x[CONFIG_BT_ACS_ECDH_COORD_SIZE]; /**< X coordinate */
	uint8_t y_size;                           /**< Size of Y coordinate (0 for Curve25519) */
#if CONFIG_BT_ACS_ECDH_HAS_Y
	uint8_t y[CONFIG_BT_ACS_ECDH_COORD_SIZE]; /**< Y coordinate (if present) */
#endif
} __packed;

/**
 * @brief KDF parameters (Table 4.76 - Key Exchange KDF Response operand).
 */
struct bt_acs_kdf_params {
	uint8_t salt_size;                             /**< KDF_Salt_Size */
	uint8_t salt[CONFIG_BT_ACS_KDF_SALT_MAX_SIZE]; /**< KDF_Salt */
	uint8_t info_size;                             /**< KDF_Info_Size */
	uint8_t info[CONFIG_BT_ACS_KDF_INFO_MAX_SIZE]; /**< KDF_Info */
};

/**
 * @brief Key-exchange state machine states.
 *
 * AWAIT_* states name the inbound CP opcode the exchange accepts next.
 * SEND_RESULT and FINALIZE drive the outbound completion chain: each is
 * entered when the previous indication confirms (via acs_kex_continue()).
 */
enum acs_kex_state {
	ACS_KEX_AWAIT_PUBKEY = 0,   /**< Expect Key Exchange ECDH */
	ACS_KEX_AWAIT_KDF,          /**< Expect Key Exchange KDF (in-chain or standalone) */
	ACS_KEX_AWAIT_CONFIRM_CODE, /**< Expect ECDH Confirmation Code */
	ACS_KEX_AWAIT_CONFIRM_RAND, /**< Expect ECDH Confirmation Random Number */
	ACS_KEX_SEND_RESULT,        /**< On confirm: send Key Exchange Response */
	ACS_KEX_FINALIZE,           /**< On confirm: commit keys, or tear down on failure */
};

/**
 * @brief Key-exchange transaction state.
 *
 * Pool-allocated when Start Key Exchange is accepted; holds the transcript
 * for exactly one exchange targeting one Key_ID and is zeroized and released
 * on completion or abort. Connections reference it via bt_acs_conn::kex
 * (NULL when no exchange is in progress).
 */
struct bt_acs_kex_ctx {
	enum acs_kex_state state; /**< Next step the state machine will accept/run */
	bool failed;              /**< Completion chain reports failure instead of success */
	/** Key material buffer — raw ECDH shared secret initially, overwritten
	 *  in-place by HKDF when the KDF step runs.  key_mat_len tracks the
	 *  valid length after each write.
	 */
	uint8_t key_material[CONFIG_BT_ACS_SHARED_SECRET_MAX_SIZE];
	uint16_t key_mat_len; /**< Valid length of key_material */
	bool kdf_applied;     /**< True once the in-exchange KDF step has run */
#if IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_ECDH) || IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_KDF)
	struct bt_acs_kdf_params kdf; /**< KDF parameters (Table 4.76) */
#endif
	struct acs_cp_start_key_exchange_req start_kex; /**< Cached START_KEY_EXCHANGE operand */
#if IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_ECDH)
	struct acs_ecdh_pubkey server_pubkey; /**< Server ephemeral public key */
	struct acs_ecdh_pubkey client_pubkey; /**< Client public key */
	psa_key_id_t ecdh_key_id;    /**< PSA key identifier for the server ephemeral private key */
	psa_key_id_t derived_key_id; /**< PSA key for shared secret / ECDHKey (never in app RAM) */
	uint8_t auth_value[ACS_CONFIRM_VALUE_SIZE];     /**< AuthValue (confirmation number) */
	uint8_t server_random[ACS_CONFIRM_VALUE_SIZE];  /**< Server random nonce */
	uint8_t client_confirm[ACS_CONFIRM_VALUE_SIZE]; /**< Client confirmation code */
	uint8_t confirm_pubkey_concat[CONFIG_BT_ACS_ECDH_COORD_SIZE * 4];
	uint8_t confirm_ecdh_auth[CONFIG_BT_ACS_ECDH_COORD_SIZE + ACS_CONFIRM_VALUE_SIZE];
	uint8_t confirm_salt[ACS_CONFIRM_VALUE_SIZE];
	uint8_t confirm_key[ACS_CONFIRM_VALUE_SIZE];
#endif
};

/** Resolved request access kind for a protected characteristic request. */
enum acs_req_access {
	ACS_REQ_ACCESS_UNKNOWN = 0,
	ACS_REQ_ACCESS_READ,
	ACS_REQ_ACCESS_WRITE,
};

#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHENTICATION)
#define ACS_REPLY_SLOTS (1 + CONFIG_BT_ACS_MAX_INFLIGHT_REQ_PER_CONN)
#else
#define ACS_REPLY_SLOTS 1
#endif

/**
 * @brief Outbound reply context — ownership-transfer model, no refcount.
 *
 * At any moment exactly one entity owns the reply: the handler building it,
 * the TX queue, or the BLE stack (via completion callback user_data).
 * Freed by acs_reply_free() through a single cleanup path.
 */
struct acs_reply {
	sys_snode_t node;               /**< TX queue linkage */
	struct bt_acs_conn *conn;       /**< Owning connection (always valid while allocated) */
	struct net_buf *request;        /**< Decrypted input (NULL for plain CP) */
	struct net_buf *response;       /**< Staged reply buffer */
	enum acs_req_access access;     /**< Read/write for protected char requests */
	enum acs_reply_channel channel; /**< ACS_REPLY_CP / DOI / DON */
	enum acs_reply_step step;       /**< What to do after current indication confirms */
	bool aborted;                   /**< Set by abort_all, checked by completion cb */
	uint16_t resource_handle;       /**< 0 for plain CP */
	uint16_t isc_id;                /**< 0 for plain CP */
};

/**
 * @brief Per-connection ACS state.
 */
struct bt_acs_conn {
	struct bt_conn *conn;                /**< Connection pointer */
	uint8_t status_flags;                /**< Status flags */
	uint16_t restriction_map_id;         /**< Restriction map ID */
	struct bt_acs_crypto_session crypto; /**< Persistent crypto session */
	struct bt_acs_kex_ctx *kex;          /**< In-progress key exchange, NULL when idle */
	uint8_t status_data[3];              /**< Embedded status indication payload */
	struct bt_gatt_indicate_params status_indicate_params; /**< Status indication params */
	atomic_t cp_locked;    /**< Plain-CP busy gate: 1 while a procedure is active */
	bool cp_abort_pending; /**< Plain-CP deferred-abort flag (indication in flight) */
	struct acs_reply replies[ACS_REPLY_SLOTS]; /**< Per-connection reply pool */
	atomic_t reply_in_use[ACS_REPLY_SLOTS];    /**< Allocation flags for reply pool */
	atomic_ptr_t pending_continue;             /**< Reply awaiting continuation */
	struct k_work reply_continue_work;         /**< Deferred continuation handler */
	struct acs_seg_tx_ctx indicate_tx; /**< Shared indication context (plain CP + DOI) */
	struct acs_seg_rx_ctx cp_rx;       /**< CP path RX reassembly context */
	struct acs_seg_rx_ctx data_rx;     /**< Data In path RX reassembly context */
	struct k_fifo request_fifo;        /**< Pending protected request FIFO */
	struct k_work request_work;        /**< Protected request dispatch worker */
#if IS_ENABLED(CONFIG_BT_ACS_PROTECTED_RESOURCE_INDICATION)
	struct k_fifo tx_indicate_fifo; /**< Pending Data Out Indicate response FIFO */
	struct k_work doi_drain_work;   /**< DOI drain worker */
	atomic_ptr_t active_indication; /**< Currently in-flight DOI response slot */
#endif
#if IS_ENABLED(CONFIG_BT_ACS_PROTECTED_RESOURCE_NOTIFICATION)
	struct acs_seg_tx_ctx notify_tx;  /**< DON notification segmentation context */
	struct k_fifo tx_notify_fifo;     /**< Pending Data Out Notify response FIFO */
	struct k_work don_drain_work;     /**< DON drain worker */
	atomic_ptr_t active_notification; /**< Currently in-flight DON response slot */
#endif
};

#ifdef __cplusplus
}
#endif

#endif /* BT_GATT_ACS_TYPES_H_ */
