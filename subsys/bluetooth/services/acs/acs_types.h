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

#include <zephyr/types.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/slist.h>
#include <zephyr/net_buf.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/services/acs.h>
#include <psa/crypto.h>

#include "acs_wire_constants.h"
#include "acs_seg.h"
#include "acs_cp.h"

#ifdef __cplusplus
extern "C" {
#endif

struct acs_procedure; /**< Defined below; forward-declared for bt_acs_conn */
struct bt_acs_conn;  /**< Defined below; forward-declared for struct acs_procedure */
struct acs_seq_desc; /**< Defined below; forward-declared for acs_reply_seq_state */

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
 * - Built by the GATT entry layer once @ref acs_channel_rx_feed reports COMPLETE.
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
	struct bt_conn *conn;             /**< Connection that produced the frame */
	uint16_t resource_handle;         /**< Inner resource handle (0 until unwrap for Data In) */
	uint16_t isc_id;                  /**< ISC_ID (0 for plain CP) */
	const uint8_t *payload;           /**< Pointer to current logical payload */
	uint16_t payload_len;             /**< Length of @p payload */
	enum acs_source_channel source_channel; /**< Originating ACS characteristic */
	bool encrypted;                   /**< True between channel RX and unwrap */
	struct net_buf *backing_buf;      /**< Owns request storage; runtime releases */
};

/**
 * @brief Result kind from @ref acs_classify_frame.
 *
 * The classifier answers "what kind of work is this?" — it does not allocate
 * state and does not dispatch.
 */
enum acs_route_kind {
	ACS_ROUTE_ACS_CP = 0,            /**< Plain ACS Control Point opcode */
	ACS_ROUTE_PROTECTED_CHAR = 1,    /**< Protected characteristic request (auto-respond / handler) */
	ACS_ROUTE_PROTECTED_SERVICE_CP = 2, /**< Protected service Control Point request */
};

/**
 * @brief Classification result for an inbound frame.
 *
 * Built by @ref acs_classify_frame. @c encrypted carries forward whether the
 * eventual reply must be wrapped on the way out (drives @ref acs_reply::encrypted
 * once that lands).
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
	bool needs_confirm;
};

/**
 * @brief Logical outbound message produced by a domain handler.
 *
 * Built by the CP-domain or service-adapter handlers and consumed by the
 * data-out channel layer. The data-out layer decides how to transport @c
 * plaintext (it may encrypt in place when @c encrypted is true) and whether
 * to wait for an indication confirmation.
 *
 * @c plaintext lifetime is owned by the caller (typically the in-flight
 * procedure / request context); the data-out layer only borrows it for the
 * send path.
 */
struct acs_reply {
	enum acs_reply_channel channel;
	struct net_buf *plaintext;
	bool encrypted;
	bool needs_confirm;
};

/**
 * @brief Signature for one step in a multi-indication reply sequence.
 *
 * Return 0 on success (framework waits for confirm, then calls next step).
 * Return negative errno to abort the sequence.
 */
typedef int (*acs_seq_step_fn)(struct acs_procedure *proc);

/**
 * @brief Internal request-aware ACS handler callback type.
 */
typedef void (*bt_acs_prot_resource_handler_t)(struct acs_procedure *req);

/**
 * @brief ACS key exchange states.
 *
 * Tracks progress through the ECDH or KDF handshake on a per-connection basis.
 */
enum bt_acs_key_exchange_state {
	BT_ACS_KEY_EXCHANGE_IDLE,    /**< No key exchange in progress */
	BT_ACS_KEY_EXCHANGE_STARTED, /**< START_KEY_EXCHANGE received; awaiting public key */
	BT_ACS_KEY_EXCHANGE_PUBKEY_EXCHANGED, /**< Public key received; awaiting KDF or confirm */
	BT_ACS_KEY_EXCHANGE_KDF_DONE,         /**< KDF step completed; awaiting confirmation */
	BT_ACS_KEY_EXCHANGE_CONFIRM_CODE,     /**< Confirmation code exchanged; awaiting random */
	BT_ACS_KEY_EXCHANGE_CONFIRM_RAND,     /**< Confirmation random exchanged; verifying */
	BT_ACS_KEY_EXCHANGE_PENDING_RESPONSE, /**< Exchange done, KEX_RESPONSE not yet sent */
	BT_ACS_KEY_EXCHANGE_PENDING_STATUS,   /**< KEX_RESPONSE sent, Status not yet indicated */
	BT_ACS_KEY_EXCHANGE_COMPLETE,         /**< Session key derived and ready for use */
};

/**
 * @brief Discriminator for which caller holds a reference on a request context.
 */
enum acs_procedure_ref_who {
	ACS_PROCEDURE_REF_ALLOC = 0, /**< Owner reference (dispatch / work-handler / reply seq) */
	ACS_PROCEDURE_REF_TX = 1,    /**< TX path reference (queued/in-flight) */
};

/**
 * @brief Descriptor for a multi-indication reply sequence.
 */
struct acs_seq_desc {
	const acs_seq_step_fn *steps;
	uint8_t step_count;
	/** Optional hook invoked by acs_seq_abort before the sequence state is cleared. */
	void (*on_abort)(struct acs_procedure *proc);
};

/**
 * @brief Bookkeeping state for a multi-step CP indication reply sequence.
 */
struct acs_reply_seq_state {
	const struct acs_seq_desc *desc; /**< NULL = no active sequence */
	uint8_t step;                    /**< Next index into desc->steps[] */
};

/**
 * @brief Persistent per-connection crypto session (survives key exchange).
 */
struct bt_acs_crypto_session {
	/** Operational AEAD key — whichever key is currently loaded for
	 *  encryption/decryption.  Holds the ECDHKey (parent) after ECDH
	 *  exchange, or the KDF-derived child key after standalone KDF.
	 *  @c kdf_child_active in bt_acs_conn tracks which one is present.
	 *  The original parent is preserved in @c ecdh_parent_key. */
	uint8_t active_key[CONFIG_BT_ACS_SESSION_KEY_SIZE];
#if IS_ENABLED(CONFIG_BT_ACS_HAS_NONCE_FIXED)
	uint8_t server_nonce_fixed[CONFIG_BT_ACS_NONCE_FIXED_BUF_SIZE]; /**< Server fixed nonce */
	uint8_t client_nonce_fixed[CONFIG_BT_ACS_NONCE_FIXED_BUF_SIZE]; /**< Client fixed nonce */
	bool client_nonce_set; /**< Set in this connection or restored from NVS */
#endif
	uint32_t tx_nonce_counter; /**< TX nonce counter */
	uint32_t rx_nonce_counter; /**< RX nonce counter */
	psa_key_id_t psa_key_id;   /**< Persistent PSA key handle (0 when not imported) */
};

/**
 * @brief NVS-serialised snapshot of an ACS crypto session.
 *
 * When KDF key exchange is enabled (persistent mode), this struct carries two
 * keys: the ECDH parent key in @p parent_key and, optionally, the KDF child
 * key in the @p child_key fields. The parent is the root used to derive
 * the child; only the child is ever used for AEAD, so @p tx/rx_nonce_counter
 * always remain zero for the parent. Separating the two allows Invalidate Key
 * to target either independently across connections.
 *
 * In session mode (CONFIG_BT_ACS_KDF_SESSION_KEY), only the ECDH parent is
 * stored; the child key is discarded at disconnect and re-derived on reconnect.
 */
struct bt_acs_session_store {
	/** ECDH/OOB exchanged parent key (spec §4.4.3.12: "top-level parent key").
	 *  When KDF is active, the child "session" key is in @c child_key. */
	uint8_t parent_key[CONFIG_BT_ACS_SESSION_KEY_SIZE];
#if IS_ENABLED(CONFIG_BT_ACS_HAS_NONCE_FIXED)
	uint8_t server_nonce_fixed[CONFIG_BT_ACS_NONCE_FIXED_BUF_SIZE]; /**< Server fixed nonce */
	uint8_t client_nonce_fixed[CONFIG_BT_ACS_NONCE_FIXED_BUF_SIZE]; /**< Client fixed nonce */
#endif
	uint32_t tx_nonce_counter;   /**< Reserved (always 0 when KDF active; parent has no AEAD
					nonces) */
	uint32_t rx_nonce_counter;   /**< Reserved (always 0 when KDF active; parent has no AEAD
					nonces) */
	uint16_t restriction_map_id; /**< Active restriction map ID */
#if IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_KDF) && !IS_ENABLED(CONFIG_BT_ACS_KDF_SESSION_KEY)
	/** When true, the KDF child key below is valid and should be restored as
	 *  the active AEAD key; parent_key holds the ECDH parent for reference. */
	bool kdf_child_valid;
	uint8_t child_key[CONFIG_BT_ACS_SESSION_KEY_SIZE]; /**< KDF-derived child key */
	uint32_t kdf_tx_nonce_counter; /**< TX nonce counter consumed against the child key */
	uint32_t kdf_rx_nonce_counter; /**< RX nonce counter consumed against the child key */
#endif
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
	uint8_t auth_value[ACS_HMAC_SHA256_SIZE];       /**< AuthValue (OOB number / static key) */
	uint8_t server_random[ACS_HMAC_SHA256_SIZE];    /**< Server random nonce */
	uint8_t client_random[ACS_HMAC_SHA256_SIZE];    /**< Client random nonce */
	uint8_t server_confirm[ACS_HMAC_SHA256_SIZE];   /**< Server confirmation code */
	uint8_t client_confirm[ACS_HMAC_SHA256_SIZE];   /**< Client confirmation code */
};

/**
 * @brief Live execution state of one ACS Control Point or protected resource procedure.
 *
 * This is the unified procedure object referenced as @c acs_procedure in the
 * runtime model. Two flavours share this struct:
 *
 *  - **Slab-allocated protected procedure**: one per in-flight protected request
 *    (Data In path), allocated from the per-connection slab and tracked in
 *    @c acs_conn->pending_reqs[]. Lifetime is governed by @c ref_count;
 *    @c is_singleton is false.
 *
 *  - **Singleton plain-CP procedure**: embedded in @c bt_acs_conn::plain_cp_proc.
 *    Manages plain ACS Control Point flow on its connection. Lifetime is the
 *    connection's lifetime; @c is_singleton is true and the slab/refcount/slot
 *    bookkeeping fields are unused. Concurrency on the plain-CP path is gated
 *    by @c locked (single in-flight procedure per connection); the @c locked
 *    field is unused for slab-allocated protected procedures because the slot
 *    pool itself enforces concurrency.
 *
 * Both flavours share a single @c reply_seq state, so the multi-indication
 * reply-sequence engine has one home regardless of dispatch path.
 *
 * The legacy public name @c acs_procedure is retained as the struct
 * tag so existing handler signatures (@ref bt_acs_prot_resource_handler_t and
 * @ref ACS_PROT_RESOURCE_HANDLER_DEFINE) keep working unchanged. The typedef
 * @c acs_procedure below is the conceptual name used by new internal code.
 */
struct acs_procedure {
	sys_snode_t node;                  /**< k_fifo linkage for send queue (DOI) */
	struct bt_acs_conn *acs_conn;      /**< Owning ACS connection; NULL after disconnect */
	struct net_buf *response;          /**< Pool buffer for plaintext response staging */
	struct net_buf *decrypted_request; /**< Reference-counted buffer from pool (request data) */
	atomic_t ref_count;                /**< Request lifetime refcount (unused if @c is_singleton) */
	ATOMIC_DEFINE(ref_flags, 2);       /**< Per-caller bitmask (debug: catch double-release) */
	struct k_work work;                /**< Deferred dispatch work item */
	struct acs_reply_seq_state reply_seq; /**< Multi-indication reply-sequence state */
	uint16_t resource_handle; /**< Protected resource handle (0 for plain CP singleton) */
	uint16_t isc_id;          /**< ISC_ID associated with this request (0 for plain CP) */
	uint16_t data_offset;     /**< Offset within decrypted_request->data where payload starts */
	uint16_t data_length;     /**< Plaintext request payload length */
	uint8_t req_slot;         /**< Index in acs_conn->pending_reqs[] (unused if singleton) */
	/* (decrypted_request ownership: implicit — non-NULL ⇒ owned, freed in acs_req_free.) */
	/* Plain-CP singleton fields (unused for slab-allocated protected procedures): */
	bool is_singleton;        /**< True for the embedded plain-CP procedure */
	atomic_t locked;          /**< Plain-CP busy gate: 1 while a procedure is active */
	bool abort_pending;       /**< Plain-CP deferred-abort flag (indication in flight) */
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
	enum bt_acs_key_exchange_state key_state; /**< Key exchange state */
#if IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_KDF)
	/** True when crypto.active_key holds a KDF-derived child key.
	 *
	 *  The KDF child key is the key actually used for all AEAD operations.
	 *  The ECDH parent key (ecdh_parent_key below) is stored only to enable
	 *  per-connection re-derivation and to satisfy the Get Current Key List
	 *  and Invalidate Key procedures, which must be able to address the two
	 *  keys independently. This flag tells the session-store, key-list, and
	 *  disconnect handlers which key's nonces are the current live counters.
	 */
	bool kdf_child_active;
	/** Snapshot of the ECDH parent key, taken when the KDF child key was
	 *  derived and overwrote crypto.active_key.  Kept in RAM so the
	 *  session-store can write it back to NVS alongside the child key
	 *  without having to decrypt the stored session just to read the parent.
	 */
	uint8_t ecdh_parent_key[CONFIG_BT_ACS_SESSION_KEY_SIZE];
#endif
	uint8_t status_flags;                /**< Status flags */
	uint16_t restriction_map_id;         /**< Restriction map ID */
	struct bt_acs_crypto_session crypto; /**< Persistent crypto session */
	struct bt_acs_kex_ctx *kex;          /**< Transient key exchange context */
	uint8_t status_data[3];              /**< Embedded status indication payload */
	struct bt_gatt_indicate_params status_indicate_params; /**< Status indication params */
	struct acs_procedure plain_cp_proc; /**< Singleton plain-CP procedure
							 *  (is_singleton=true; not slab-managed)
							 */
	struct acs_seg_tx_ctx cp_tx;    /**< Plain CP indication transport context */
#if IS_ENABLED(CONFIG_BT_ACS_PROTECTED_RESOURCE_INDICATION)
	struct acs_seg_tx_ctx indicate_tx; /**< DOI indication segmentation context */
#endif
	struct acs_seg_rx_ctx cp_rx;   /**< CP path RX reassembly context */
	struct acs_seg_rx_ctx data_rx; /**< Data In path RX reassembly context */
	/** In-flight request slots, one per concurrent protected request. */
	atomic_ptr_t pending_reqs[CONFIG_BT_ACS_MAX_INFLIGHT_REQ_PER_CONN];
#if IS_ENABLED(CONFIG_BT_ACS_PROTECTED_RESOURCE_INDICATION)
	struct k_fifo indicate_fifo;    /**< Pending Data Out Indicate response FIFO */
	atomic_ptr_t active_indication; /**< Currently in-flight DOI response slot */
#endif
};

/**
 * @brief Internal per-resource request-aware handler registration entry.
 */
struct bt_acs_prot_resource_handler_entry {
	const struct bt_uuid *char_uuid;        /**< Characteristic UUID to match */
	bt_acs_prot_resource_handler_t handler; /**< Handler invoked on match */
};

/**
 * @brief Register a protected resource handler at compile time.
 *
 * Only available when at least one data protection algorithm is enabled.
 * The handler is dispatched when an encrypted request arrives via Data In
 * for the characteristic identified by @p _char_uuid.
 */
#if IS_ENABLED(CONFIG_BT_ACS_ANY_DATA_PROTECTION)
#define ACS_PROT_RESOURCE_HANDLER_DEFINE(_name, _char_uuid, _handler)                              \
	STRUCT_SECTION_ITERABLE(bt_acs_prot_resource_handler_entry, _name) = {                     \
		.char_uuid = (_char_uuid),                                                         \
		.handler = (_handler),                                                             \
	}
#else
#define ACS_PROT_RESOURCE_HANDLER_DEFINE(_name, _char_uuid, _handler)                              \
	BUILD_ASSERT(0, "ACS_PROT_RESOURCE_HANDLER_DEFINE requires data protection to be enabled")
#endif

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
		.channel = proc->is_singleton ? ACS_REPLY_CP : ACS_REPLY_DOI,
		.encrypted = !proc->is_singleton,
		.needs_confirm = true,
	};
}

#ifdef __cplusplus
}
#endif

#endif /* BT_GATT_ACS_TYPES_H_ */
