/*
 * Copyright (c) 2025 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file acs_types.h
 * @brief ACS internal data structures, enumerations, and forward declarations.
 *
 * All core ACS types live here to keep the circular acs_procedure <-> bt_acs_conn
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

struct acs_procedure; /**< Defined below; forward-declared for bt_acs_conn */
struct bt_acs_conn;   /**< Defined below; forward-declared for struct acs_procedure */

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
 * @brief Normalized inbound request — handoff object from transport decode into runtime dispatch.
 *
 * Lifecycle:
 * - Built by the GATT entry layer once acs_channel_rx_feed() reports COMPLETE.
 *   At this point @c payload points at the reassembled bytes in @c backing_buf,
 *   @c source_channel is set, @c encrypted is true for Data In and false for CP,
 *   and @c resource_handle / @c isc_id are 0.
 * - For Data In, the runtime then calls the unwrap helper which sets
 *   @c resource_handle, @c isc_id, clears @c encrypted, and replaces @c payload /
 *   @c payload_len with the inner plaintext.
 * - The runtime owns @c backing_buf and is responsible for releasing it on
 *   every terminal path.
 */
struct acs_frame {
	struct bt_conn *conn;     /**< Connection that produced the frame */
	uint16_t resource_handle; /**< Inner resource handle (0 until unwrap for Data In) */
	uint16_t isc_id;          /**< ISC_ID (0 for plain CP) */
	const uint8_t *payload;   /**< Pointer to current logical payload */
	uint16_t payload_len;     /**< Length of @p payload */
	enum acs_source_channel source_channel; /**< Originating ACS characteristic */
	bool encrypted;                         /**< True between channel RX and unwrap */
	struct net_buf *backing_buf;            /**< Owns request storage; runtime releases */
};

/**
 * @brief Build a plain-CP frame from a reassembled segmentation buffer.
 *
 * The frame carries @p rx_buf in @c backing_buf — the runtime layer is
 * responsible for releasing the buffer (or transferring ownership) on every
 * terminal path. The Data In equivalent is built post-decrypt inside the
 * unwrap helper and intentionally has @c backing_buf == NULL because the
 * runtime dispatcher transfers ownership directly out of @c data_rx.
 */
static inline struct acs_frame acs_frame_from_cp_rx(struct bt_conn *conn, struct net_buf *rx_buf)
{
	return (struct acs_frame){
		.conn = conn,
		.resource_handle = 0,
		.isc_id = 0,
		.payload = rx_buf->data,
		.payload_len = rx_buf->len,
		.source_channel = ACS_SRC_CP,
		.encrypted = false,
		.backing_buf = rx_buf,
	};
}

/**
 * @brief Result kind from @ref acs_classify_frame.
 *
 * The classifier answers "what kind of work is this?" — it does not allocate
 * state and does not dispatch.
 */
enum acs_route_kind {
	ACS_ROUTE_ACS_CP = 0, /**< Plain ACS Control Point opcode */
	ACS_ROUTE_PROTECTED_CHAR =
		1, /**< Protected characteristic request (auto-respond / handler) */
	ACS_ROUTE_PROTECTED_SERVICE_CP = 2, /**< Protected service Control Point request */
};

/**
 * @brief Classification result for an inbound frame.
 *
 * Built by @ref acs_classify_frame. @c encrypted carries forward whether the
 * eventual reply must be wrapped on the way out.
 */
struct acs_route {
	enum acs_route_kind kind;
	uint16_t resource_handle;
	uint16_t isc_id;
	bool encrypted;
};

/**
 * @brief Outbound transport channel for an @ref acs_reply.
 *
 * Distinguishes the three send paths a reply can take:
 *  - @ref ACS_REPLY_CP   — plain ACS Control Point indication (segmented).
 *  - @ref ACS_REPLY_DON  — Data Out Notify (encrypted notification).
 *  - @ref ACS_REPLY_DOI  — Data Out Indicate (encrypted, segmented, confirmed).
 */
enum acs_reply_channel {
	ACS_REPLY_CP = 0,
	ACS_REPLY_DON = 1,
	ACS_REPLY_DOI = 2,
};

/**
 * @brief Canonical transport defaults for a logical ACS reply.
 *
 * Derived once from the dispatch path, then reused by handlers when staging
 * and sending replies so call sites do not have to re-encode channel /
 * encryption / confirm policy inline.
 */
struct acs_reply_mode {
	enum acs_reply_channel channel;
	bool encrypted;
};

/**
 * @brief Logical outbound message produced by a domain handler.
 *
 * Built by the CP-domain or service-adapter handlers and consumed by the
 * data-out channel layer. The data-out layer decides how to transport
 * @c plaintext based on @c channel.
 *
 * @c plaintext lifetime is owned by the caller (typically the in-flight
 * procedure / request context); the data-out layer only borrows it for the
 * send path.
 */
struct acs_reply {
	enum acs_reply_channel channel;
	struct net_buf *plaintext;
};

/**
 * @brief State of a multi-indication reply sequence (OTS-style enum dispatch).
 *
 * Each value identifies the next step to execute when the current CP indication
 * is confirmed.  ACS_CP_SEQ_IDLE means no sequence is active.
 */
enum acs_cp_seq_state {
	ACS_CP_SEQ_IDLE = 0,
	ACS_CP_SEQ_KEX_SUCCESS_RSP,
	ACS_CP_SEQ_KEX_SUCCESS_STATUS,
	ACS_CP_SEQ_KEX_FAIL_RSP,
	ACS_CP_SEQ_KEX_FAIL_CLEANUP,
	ACS_CP_SEQ_ALL_ACTIVE_ISC,
	ACS_CP_SEQ_ALL_ACTIVE_KEY,
	ACS_CP_SEQ_ALL_ACTIVE_RC,
	ACS_CP_SEQ_INVALIDATE_SELF,
};

struct bt_acs_kex_ctx;

/**
 * @brief Discriminator for which caller holds a reference on a request context.
 */
enum acs_procedure_ref_who {
	ACS_PROCEDURE_REF_ALLOC = 0, /**< Owner reference (dispatch / work-handler / reply seq) */
	ACS_PROCEDURE_REF_TX = 1,    /**< TX path reference (queued/in-flight) */
};

/**
 * @brief Which flavour of ACS procedure this struct instance represents.
 */
enum acs_proc_kind {
	ACS_PROC_KIND_PLAIN_CP = 0,      /**< Embedded per-connection plain Control Point proc */
	ACS_PROC_KIND_PROTECTED_REQ = 1, /**< Slab-allocated protected request proc */
};

/**
 * @brief Runtime state for one live ACS key on a connection.
 */
struct bt_acs_runtime_key_state {
	/** Static key descriptor that this runtime state is bound to. */
	const struct bt_acs_key_desc_record *key_desc;
	/** Imported PSA key handle. 0 means this current key is not installed/usable. */
	psa_key_id_t psa_key_id;
	/** Key material retained in RAM for restore/invalidate/derive flows. */
	uint8_t key[CONFIG_BT_ACS_SESSION_KEY_SIZE];
};

static inline uint16_t acs_runtime_key_id(const struct bt_acs_runtime_key_state *key_state)
{
	return (key_state && key_state->key_desc) ? key_state->key_desc->key_id : 0U;
}

/**
 * @brief Runtime nonce + algorithm state for one AES key descriptor record.
 */
struct bt_acs_key_desc_runtime {
	/** Static AES-with-nonce key descriptor bound to this runtime slot. */
	const struct bt_acs_key_desc_record *key_desc;
	/** Resolved current exchange-key Key_ID for this record's parent chain. */
	uint16_t current_key_id;
	/** Imported PSA key handle for the algorithm record. */
	psa_key_id_t psa_key_id;
	/** Key material copied from the current parent/child exchange key. */
	uint8_t key[CONFIG_BT_ACS_SESSION_KEY_SIZE];
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

/**
 * @brief Persistent per-connection ACS crypto state.
 */
struct bt_acs_crypto_session {
	/** Runtime live-key states for this connection.
	 *
	 * Only key-exchange records become current keys:
	 * ECDH, KDF, and OOB. Algorithm records remain compile-time descriptors
	 * that resolve to one of these current keys via their parent Key_ID
	 * relation.
	 */
	struct bt_acs_runtime_key_state current_keys[ACS_KEY_ID_COUNT];
	/** Transient key-exchange scratch state, if a handshake is active. */
	struct bt_acs_kex_ctx *kex;
	/** Per-Key_ID nonce/counter state for AES records that use nonce material. */
	struct bt_acs_key_desc_runtime key_desc_runtimes[CONFIG_BT_ACS_MAX_NONCE_RECORDS];
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
 * @brief Transient key-exchange context.
 */
struct bt_acs_kex_ctx {
	bool in_use; /**< Pool-allocation flag */
	/** Next legal inbound KEX opcode, or 0 when no further inbound step is accepted. */
	uint8_t next_expected_opcode;
	/**
	 * Key material buffer - holds either the raw ECDH shared secret
	 * (before KDF) or the HKDF-derived ECDH key (after KDF).  Without
	 * KDF, shared_secret is used directly for confirmation; with KDF,
	 * ecdh_key replaces it.  The two are mutually exclusive at any
	 * given point in the key exchange state machine.
	 */
	union {
		uint8_t shared_secret[CONFIG_BT_ACS_SHARED_SECRET_MAX_SIZE];
		uint8_t ecdh_key[CONFIG_BT_ACS_SHARED_SECRET_MAX_SIZE];
	};
	uint16_t key_mat_len; /**< Length of the active key material (shared_secret or ecdh_key) */
	bool kdf_applied;     /**< True after KDF overwrites shared_secret with ecdh_key */
#if IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_ECDH) || IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_KDF)
	struct bt_acs_kdf_params kdf; /**< KDF parameters (Table 4.76) */
#endif
	struct acs_ecdh_pubkey server_pubkey; /**< Server ephemeral public key */
	struct acs_ecdh_pubkey client_pubkey; /**< Client public key */
	psa_key_id_t ecdh_key_id; /**< PSA key identifier for the server ephemeral private key */
	struct acs_cp_start_key_exchange_req start_kex; /**< Cached START_KEY_EXCHANGE operand */
	uint8_t auth_value[ACS_CONFIRM_VALUE_SIZE];     /**< AuthValue (OOB number / static key) */
	uint8_t server_random[ACS_CONFIRM_VALUE_SIZE];  /**< Server random nonce */
	uint8_t client_random[ACS_CONFIRM_VALUE_SIZE];  /**< Client random nonce */
	uint8_t server_confirm[ACS_CONFIRM_VALUE_SIZE]; /**< Server confirmation code */
	uint8_t client_confirm[ACS_CONFIRM_VALUE_SIZE]; /**< Client confirmation code */
};

/**
 * @brief Buffers owned by an in-flight procedure.
 */
struct acs_proc_buffers {
	struct net_buf *request_buf;  /**< Reference-counted decrypted request buffer */
	struct net_buf *response_buf; /**< Pool buffer for plaintext response staging */
};

struct acs_proc_route {
	uint16_t resource_handle; /**< Protected resource handle (0 for plain CP singleton) */
	uint16_t isc_id;          /**< ISC_ID associated with this request (0 for plain CP) */
};

struct acs_proc_lifetime {
	atomic_t ref_count;          /**< Request lifetime refcount (unused for plain CP) */
	ATOMIC_DEFINE(ref_flags, 2); /**< Per-caller bitmask (debug: catch double-release) */
};

struct acs_proc_registration {
	/** Published slot in acs_conn->pending_reqs[] (NULL for plain CP singleton). */
	atomic_ptr_t *pending_slot;
};

struct acs_proc_plain_cp_state {
	atomic_t locked;    /**< Plain-CP busy gate: 1 while a procedure is active */
	bool abort_pending; /**< Plain-CP deferred-abort flag (indication in flight) */
};

struct acs_procedure {
	sys_snode_t node;                  /**< k_fifo linkage for send queue (DOI) */
	struct bt_acs_conn *acs_conn;      /**< Owning ACS connection; NULL after disconnect */
	enum acs_proc_kind kind;           /**< Plain CP singleton vs protected request */
	enum acs_cp_seq_state seq_state;   /**< Multi-indication reply-sequence state */
	struct acs_proc_buffers buffers;   /**< Request/response buffer ownership */
	struct acs_proc_route route;       /**< Resource routing metadata */
	struct acs_proc_lifetime lifetime; /**< Refcount bookkeeping */
	struct acs_proc_registration registration; /**< Connection slot registration */
	struct acs_proc_plain_cp_state plain_cp;   /**< Plain-CP-only busy/abort interlock */
};

/**
 * @brief Per-connection ACS state.
 */
struct bt_acs_conn {
	struct bt_conn *conn; /**< Connection pointer */
	/* Cached GATT attrs — populated once in acs_conn_alloc */
	const struct bt_gatt_attr *attr_cp;
	const struct bt_gatt_attr *attr_status;
#if IS_ENABLED(CONFIG_BT_ACS_PROTECTED_RESOURCE_NOTIFICATION)
	const struct bt_gatt_attr *attr_don;
#endif
#if IS_ENABLED(CONFIG_BT_ACS_PROTECTED_RESOURCE_INDICATION)
	const struct bt_gatt_attr *attr_doi;
#endif
	uint8_t status_flags;                /**< Status flags */
	uint16_t restriction_map_id;         /**< Restriction map ID */
	struct bt_acs_crypto_session crypto; /**< Persistent crypto session */
	uint8_t status_data[3];              /**< Embedded status indication payload */
	struct bt_gatt_indicate_params status_indicate_params; /**< Status indication params */
	struct acs_procedure plain_cp_proc;                    /**< Singleton plain-CP procedure
								*  (kind=ACS_PROC_KIND_PLAIN_CP; not slab-managed)
								*/
	struct acs_seg_tx_ctx cp_tx; /**< Plain CP indication transport context */
#if IS_ENABLED(CONFIG_BT_ACS_PROTECTED_RESOURCE_NOTIFICATION)
	struct acs_seg_notify_ctx notify_tx; /**< DON notification segmentation context */
#endif
#if IS_ENABLED(CONFIG_BT_ACS_PROTECTED_RESOURCE_INDICATION)
	struct acs_seg_tx_ctx indicate_tx; /**< DOI indication segmentation context */
#endif
	struct acs_seg_rx_ctx cp_rx;   /**< CP path RX reassembly context */
	struct acs_seg_rx_ctx data_rx; /**< Data In path RX reassembly context */
	struct k_fifo request_fifo;    /**< Pending protected request FIFO */
	struct k_work request_work;    /**< Protected request dispatch worker */
	/** In-flight request slots, one per concurrent protected request. */
	atomic_ptr_t inflight_reqs[CONFIG_BT_ACS_MAX_INFLIGHT_REQ_PER_CONN]; /**< Protected request
										dispatch worker */
#if IS_ENABLED(CONFIG_BT_ACS_PROTECTED_RESOURCE_INDICATION)
	struct k_fifo indicate_fifo;       /**< Pending Data Out Indicate response FIFO */
	struct k_work doi_drain_work;      /**< DOI drain / continuation worker */
	atomic_ptr_t active_indication;    /**< Currently in-flight DOI response slot */
	atomic_ptr_t pending_seq_continue; /**< Reply sequence waiting to continue on workq */
#endif
#if IS_ENABLED(CONFIG_BT_ACS_PROTECTED_RESOURCE_NOTIFICATION)
	struct k_fifo notify_fifo;        /**< Pending Data Out Notify response FIFO */
	struct k_work don_drain_work;     /**< DON drain worker */
	atomic_ptr_t active_notification; /**< Currently in-flight DON response slot */
#endif
};

/**
 * @brief Derive the canonical reply transport mode for a procedure.
 *
 * Plain CP replies travel as unencrypted CP indications; protected CP replies
 * travel as encrypted DOI indications. Both are confirmed paths.
 */
static inline struct acs_reply_mode acs_proc_reply_mode(const struct acs_procedure *proc)
{
	__ASSERT_NO_MSG(proc != NULL);

	return (struct acs_reply_mode){
		.channel = (proc->kind == ACS_PROC_KIND_PLAIN_CP) ? ACS_REPLY_CP : ACS_REPLY_DOI,
		.encrypted = (proc->kind == ACS_PROC_KIND_PROTECTED_REQ),
	};
}

#ifdef __cplusplus
}
#endif

#endif /* BT_GATT_ACS_TYPES_H_ */
