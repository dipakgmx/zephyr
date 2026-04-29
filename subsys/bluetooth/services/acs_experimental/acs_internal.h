/*
 * Copyright (c) 2026 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_SUBSYS_BLUETOOTH_SERVICES_ACS_EXPERIMENTAL_INTERNAL_H_
#define ZEPHYR_SUBSYS_BLUETOOTH_SERVICES_ACS_EXPERIMENTAL_INTERNAL_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/services/acs.h>
#include <zephyr/net_buf.h>
#include <zephyr/sys/atomic.h>

#include "acs_crypto.h"
#include "acs_kex.h"
#include "acs_seg.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Identifies which ATT-facing ACS channel produced an @ref acs_frame. */
enum acs_source_channel {
	ACS_SOURCE_CP_CHANNEL = 0,
	ACS_SOURCE_DATA_IN_CHANNEL,
};

/** High-level execution kinds selected by the protected-resource router. */
enum acs_route_kind {
	ACS_ROUTE_ACS_CP = 0,
	ACS_ROUTE_PROTECTED_CHAR,
	ACS_ROUTE_PROTECTED_SERVICE_CP,
};

/** Outbound ACS reply channels. */
enum acs_reply_channel {
	ACS_REPLY_CP = 0,
	ACS_REPLY_DON,
	ACS_REPLY_DOI,
	ACS_REPLY_STATUS,
};

/** Lifecycle states for a live @ref acs_procedure instance. */
enum acs_proc_status {
	ACS_PROC_IDLE = 0,
	ACS_PROC_RUNNING,
	ACS_PROC_WAIT_CONFIRM,
	ACS_PROC_COMPLETE,
	ACS_PROC_ABORTED,
	ACS_PROC_FAILED,
};

/** Common procedure-engine step results. */
enum acs_proc_result {
	/** Step completed synchronously; procedure may be released immediately. */
	ACS_PROC_RES_COMPLETE = 0,
	/** Current step is waiting for indication confirmation. */
	ACS_PROC_STEP_WAIT_IND_CONFIRM,
};

/**
 * @brief Normalized inbound ACS request.
 *
 * A frame is the handoff object between the channel layer and the router.
 * It always represents a complete message after any needed reassembly.
 *
 * Ownership:
 * - @ref payload points into @ref backing_buf when reassembly was required.
 * - @ref backing_buf stays owned by the runtime until the frame leaves
 *   @c acs_runtime_dispatch_frame().
 */
struct acs_frame {
	/** Owning Bluetooth connection for this request. */
	struct bt_conn *conn;
	/**
	 * Protected resource handle selected by the channel decode or secure
	 * unwrap stage.
	 *
	 * For plain ACS CP writes this is typically 0 until routing decides that
	 * the request targets the ACS Control Point transport itself.
	 */
	uint16_t resource_handle;
	/**
	 * Information Security Configuration identifier carried by the request.
	 *
	 * For plain CP writes this is 0. For secure Data In it is the outer
	 * ISC_ID extracted before decryption.
	 */
	uint16_t isc_id;
	/** Pointer to the normalized request payload bytes. */
	const uint8_t *payload;
	/** Length in bytes of @ref payload. */
	uint16_t payload_len;
	/** Channel on which the request entered the ACS service. */
	enum acs_source_channel source_channel;
	/** True when the payload arrived through the secure Data In path. */
	bool encrypted;
	/**
	 * Buffer owning @ref payload storage, if any.
	 *
	 * The runtime frees this after dispatch. For single-buffer flows this may
	 * be the segmented reassembly buffer; after secure unwrap it may instead
	 * be the decrypted plaintext buffer.
	 */
	struct net_buf *backing_buf;
};

/** Router output describing how the runtime should execute an @ref acs_frame. */
struct acs_route {
	/** High-level execution path chosen for the request. */
	enum acs_route_kind kind;
	/** Resource handle to attach to the resulting procedure instance. */
	uint16_t resource_handle;
	/** ISC identifier to attach to the resulting procedure instance. */
	uint16_t isc_id;
	/** True when replies for this route should use secure transport semantics. */
	bool encrypted;
};

/**
 * @brief Logical outbound reply produced by a procedure step.
 *
 * The procedure engine and service/domain logic populate this object, then
 * the outbound channel layer decides how to transmit it.
 */
struct acs_reply {
	/** ACS channel used to send this reply. */
	enum acs_reply_channel channel;
	/**
	 * Plaintext reply payload buffer.
	 *
	 * For secure DON/DOI replies this may later be replaced by a wrapped wire
	 * buffer in the outbound channel layer.
	 */
	struct net_buf *plaintext;
	/** True when the reply must be wrapped with ACS secure data protection. */
	bool encrypted;
	/** True when transport confirmation is required before the procedure continues. */
	bool needs_confirm;
};

enum acs_proc_flag {
	ACS_PROC_FLAG_SECURE_TRANSPORT = BIT(0),
};

/** Typed procedure-private state carried by @ref acs_procedure. */
enum acs_proc_state_kind {
	/** Procedure does not carry additional typed state. */
	ACS_PROC_STATE_NONE = 0,
	/** Procedure is in the final key-exchange result stage and stores a key ID. */
	ACS_PROC_STATE_KEX_FINAL,
};

/** Tagged procedure-private state payload. */
union acs_proc_state_data {
	/** Key identifier associated with a final KEX result sequence. */
	uint16_t key_id;
};

struct acs_procedure;

/**
 * @brief Procedure callbacks for one procedure implementation.
 */
struct acs_proc_ops {
	/** Start the first step of the procedure for a normalized inbound frame. */
	int (*start)(struct acs_procedure *proc, const struct acs_frame *frame);
	/** Advance the procedure after a transport confirmation callback. */
	int (*on_confirm)(struct acs_procedure *proc);
	/** Abort hook called when the procedure is terminated early. */
	void (*on_abort)(struct acs_procedure *proc, int reason);
	/** Final cleanup hook before the procedure storage is zeroed and reused. */
	void (*destroy)(struct acs_procedure *proc);
};

/**
 * @brief Live execution context for one ACS operation.
 */
struct acs_procedure {
	/** Procedure callback table that implements this live operation. */
	const struct acs_proc_ops *ops;
	/** Connection on which the procedure is executing. */
	struct bt_conn *conn;
	/** Resource handle associated with the operation or reply channel wrapping. */
	uint16_t resource_handle;
	/** ISC identifier associated with the operation or secure reply. */
	uint16_t isc_id;
	/** Generic step counter for chained or confirm-driven procedures. */
	uint8_t step;
	/** Bitset of @ref acs_proc_flag values. */
	uint8_t flags;
	/** Selects the active member in @ref state. */
	enum acs_proc_state_kind state_kind;
	/** Typed procedure-private state payload. */
	union acs_proc_state_data state;
	/** Current lifecycle state of the procedure. */
	enum acs_proc_status status;
	/** Reply currently owned by the procedure while waiting for completion/reset. */
	struct acs_reply pending_reply;
};

/**
 * @brief Per-connection runtime state owned by @c acs_runtime.c.
 */
struct acs_conn_ctx {
	/** Referenced Bluetooth connection that owns this runtime slot. */
	struct bt_conn *conn;
	/** Connection-scoped crypto session, nonce, and active key state. */
	struct acs_crypto_ctx crypto;
	/** CP receive-side segmentation/reassembly context. */
	struct acs_seg_rx_ctx cp_rx;
	/** Data In receive-side segmentation/reassembly context. */
	struct acs_seg_rx_ctx data_rx;
	/** Segmented TX lane for CP indications. */
	struct acs_seg_tx_ctx cp_tx;
	/** Segmented TX lane for DOI indications. */
	struct acs_seg_tx_ctx doi_tx;
	/** Deferred completion work that releases the active procedure slot. */
	struct k_work cp_complete_work;
	/** Single active procedure slot for this connection. */
	struct acs_procedure active_proc;
	/** Admission gate indicating whether the procedure slot is currently occupied. */
	atomic_t proc_busy;
	/** Currently active restriction map identifier for this peer. */
	uint16_t active_map_id;
	/** Per-connection key-exchange state machine and derived key material. */
	struct acs_kex_ctx kex;
	/** ACS Status characteristic flags for this peer. */
	uint8_t status_flags;
	/** Input OOB value staged by the application for confirmation-based KEX. */
	uint8_t input_oob[32];
	/** Number of valid bytes in @ref input_oob. */
	uint16_t input_oob_len;
	/** Deferred ABORT request pending until the current indication lane unwinds. */
	bool abort_pending;
	/** Secure transport flags to apply to the deferred ABORT response. */
	uint8_t abort_flags;
	/** Resource handle to use when sending the deferred ABORT response. */
	uint16_t abort_resource_handle;
	/** ISC identifier to use when sending the deferred ABORT response. */
	uint16_t abort_isc_id;
};

/** Return the default crypto mode implied by the enabled ACS protection features. */
enum acs_crypto_mode acs_runtime_crypto_mode_default(void);

/** Initialize the experimental ACS runtime state for all connection slots. */
int acs_runtime_init(void);

/**
 * @brief Handle a complete or fragmented ACS Control Point write.
 *
 * This is the intended top-level entry point for CP GATT writes. The runtime
 * performs reassembly, routing, procedure creation, and first-step dispatch.
 */
int acs_runtime_handle_cp_write(struct bt_conn *conn, const void *buf, uint16_t len);

/**
 * @brief Handle a complete or fragmented ACS Data In write.
 *
 * This mirrors @ref acs_runtime_handle_cp_write() but uses the Data In
 * channel's framing and reassembly rules.
 */
int acs_runtime_handle_data_in(struct bt_conn *conn, const void *buf, uint16_t len);

/** Reset all runtime state associated with a disconnected link. */
void acs_runtime_handle_disconnect(struct bt_conn *conn);

/** Look up an existing runtime slot for @p conn without creating one. */
struct acs_conn_ctx *acs_runtime_lookup_conn(struct bt_conn *conn);

/** Return the runtime slot for @p conn, creating it if needed. */
struct acs_conn_ctx *acs_runtime_acquire_conn(struct bt_conn *conn);
bool acs_runtime_client_nonce_conflicts(struct bt_conn *exclude_conn, const uint8_t *nonce,
					size_t nonce_len);
int acs_runtime_invalidate_all_security(void);
int acs_runtime_invalidate_key(struct bt_conn *conn, uint16_t key_id);

int acs_persist_save_conn(struct acs_conn_ctx *conn_ctx);
void acs_persist_restore_conn(struct acs_conn_ctx *conn_ctx);
int acs_persist_delete_conn(struct bt_conn *conn);
int acs_persist_delete_all(void);
bool acs_persist_client_nonce_conflicts(struct bt_conn *exclude_conn, const uint8_t *nonce,
					size_t nonce_len);

/** Allocate a temporary channel buffer used during reassembly or reply build. */
struct net_buf *acs_channel_buf_alloc(void);

/** Release a buffer previously returned by @ref acs_channel_buf_alloc(). */
void acs_channel_buf_free(struct net_buf *buf);

/** Reassemble a CP-channel request into a complete @ref acs_frame. */
int acs_cp_channel_reassemble(struct acs_conn_ctx *conn_ctx, const void *buf, uint16_t len,
			      struct acs_frame *frame);

/** Reassemble a Data In request into a complete @ref acs_frame. */
int acs_data_in_channel_reassemble(struct acs_conn_ctx *conn_ctx, const void *buf, uint16_t len,
				   struct acs_frame *frame);

/** Normalize an already-complete CP write into an @ref acs_frame. */
int acs_cp_channel_frame_from_write(struct bt_conn *conn, const void *buf, uint16_t len,
				    struct acs_frame *frame);

/**
 * @brief Normalize an already-complete Data In write into an @ref acs_frame.
 *
 * This helper extracts the outer ISC_ID and records the encrypted payload as
 * received. The protected resource handle is not available until decryption
 * later in the pipeline.
 */
int acs_data_in_channel_frame_from_write(struct bt_conn *conn, const void *buf, uint16_t len,
					 struct acs_frame *frame);

/** Classify a normalized request into an ACS route kind. */
int acs_classify_frame(const struct acs_frame *frame, struct acs_route *route);

/** Populate a procedure instance for the selected route. */
int acs_build_procedure_for_route(const struct acs_frame *frame,
					   const struct acs_route *route,
					   struct acs_procedure *proc);

/** Start executing the first step of a procedure. */
int acs_procedure_engine_start(struct acs_procedure *proc, const struct acs_frame *frame);

/** Resume a procedure after a transport confirm for its previous reply. */
int acs_procedure_engine_on_confirm(struct acs_procedure *proc);

/** Abort a procedure and notify its procedure-specific abort hook. */
void acs_procedure_engine_abort(struct acs_procedure *proc, int reason);

/** Destroy and zero a procedure before reusing its storage slot. */
void acs_procedure_engine_reset(struct acs_procedure *proc);

/**
 * @brief Queue or send the outbound message represented by @p reply.
 *
 * The outbound channel currently serializes CP indications through the
 * segmented TX engine so confirm-driven procedures can safely chain replies.
 * Additional channels still need their full queueing and crypto behavior.
 */
int acs_data_out_channel_send(struct acs_procedure *proc, struct acs_reply *reply);

/**
 * @brief Temporary service-adapter bridge.
 *
 * The real implementation will resolve protected resources and invoke the
 * corresponding service-specific handlers.
 */
int acs_service_adapter_dispatch(struct acs_procedure *proc, const struct acs_frame *frame);

/**
 * @brief Return the compile-time default crypto mode for this build.
 *
 * The actual per-session mode may still vary later when ISC descriptors and
 * key exchange are implemented, but this gives the rest of the scaffold a
 * stable default selection point.
 */
enum acs_crypto_mode acs_runtime_crypto_mode_default(void);

/** Return the registered ACS application callbacks, if any. */
const struct bt_acs_cb *acs_runtime_callbacks(void);

/**
 * @brief Dispatch an ACS Control Point procedure request.
 *
 * This is the first real CP domain entrypoint for the procedure-centric
 * scaffold. It owns opcode validation, operand checks, and response payload
 * construction for plain ACS CP requests.
 */
int acs_cp_domain_handle(struct acs_procedure *proc, const struct acs_frame *frame);
int acs_cp_domain_send_response_code(struct acs_procedure *proc, uint8_t req_opcode,
				     uint8_t response_code);

/** Return the ACS Control Point value attribute. */
const struct bt_gatt_attr *acs_service_attr_cp(void);
uint16_t acs_service_handle_cp(void);
int acs_service_cache_attrs(void);

/** Return the ACS Data Out Notify value attribute. */
const struct bt_gatt_attr *acs_service_attr_don(void);

/** Return the ACS Data Out Indicate value attribute. */
const struct bt_gatt_attr *acs_service_attr_doi(void);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_SUBSYS_BLUETOOTH_SERVICES_ACS_EXPERIMENTAL_INTERNAL_H_ */
