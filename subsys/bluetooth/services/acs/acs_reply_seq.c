/*
 * Copyright (c) 2026 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>

#include "acs_internal.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(bt_acs, CONFIG_BT_ACS_LOG_LEVEL);

/* Forward declaration — acs_seq_continue calls acs_seq_clear. */
void acs_seq_clear(struct acs_cp_ctx *ctx);

/**
 * @brief Resolve the active procedure object from a CP context.
 *
 * Returns the slab-allocated protected procedure when present, otherwise the
 * connection's embedded plain-CP singleton.
 */
static acs_procedure *acs_seq_proc_from_ctx(const struct acs_cp_ctx *ctx)
{
	if (!ctx) {
		return NULL;
	}
	if (ctx->prot_req) {
		return ctx->prot_req;
	}
	if (ctx->acs_conn) {
		return &ctx->acs_conn->plain_cp_proc;
	}
	return NULL;
}

/**
 * @brief Resolve the reply sequence state from a CP context.
 *
 * Both plain CP and protected CP procedures now share a single
 * @c reply_seq home on the unified procedure object.
 */
static struct acs_reply_seq_state *acs_seq_state_from_ctx(const struct acs_cp_ctx *ctx)
{
	acs_procedure *proc = acs_seq_proc_from_ctx(ctx);

	return proc ? &proc->reply_seq : NULL;
}

/**
 * @brief Advance to the next step in the active reply sequence.
 *
 * Invokes the next step function from the sequence descriptor.  If no steps
 * remain, the sequence is cleared automatically.
 *
 * @param ctx  CP dispatch context.
 * @return 0 on success or sequence complete, negative errno from the step function on failure.
 */
static int acs_seq_continue(struct acs_cp_ctx *ctx)
{
	struct acs_reply_seq_state *seq = acs_seq_state_from_ctx(ctx);

	if (!seq || !seq->desc || seq->step >= seq->desc->step_count) {
		acs_seq_clear(ctx);
		return 0;
	}

	acs_seq_step_fn fn = seq->desc->steps[seq->step];
	seq->step++;

	return fn(ctx);
}

bool acs_seq_active(const struct acs_cp_ctx *ctx)
{
	struct acs_reply_seq_state const *seq = acs_seq_state_from_ctx(ctx);

	return seq && seq->desc != NULL;
}

void acs_seq_begin(struct acs_cp_ctx *ctx, const struct acs_seq_desc *desc)
{
	struct acs_reply_seq_state *seq = acs_seq_state_from_ctx(ctx);

	__ASSERT_NO_MSG(seq != NULL);

	/* The ALLOC (owner) ref keeps the request alive for the full duration
	 * of the reply sequence.  acs_runtime_dispatch_protected_cp_frame() defers release_owner()
	 * when it sees reply_seq.desc != NULL after dispatch, and acs_seq_clear()
	 * calls release_owner() when the sequence completes or aborts.
	 */
	seq->desc = desc;
	seq->step = 0;
}

void acs_seq_clear(struct acs_cp_ctx *ctx)
{
	struct acs_reply_seq_state *seq = acs_seq_state_from_ctx(ctx);
	bool was_active;

	if (!seq) {
		return;
	}

	was_active = seq->desc != NULL;
	memset(seq, 0, sizeof(*seq));

	/* The sequence owned the ALLOC ref for its duration (deferred from
	 * acs_runtime_dispatch_protected_cp_frame).  Release it now that the sequence is done.
	 */
	if (was_active && ctx->prot_req) {
		acs_prot_resource_req_release_owner(ctx->prot_req);
	}
}

void acs_seq_abort(struct acs_cp_ctx *ctx)
{
	struct acs_reply_seq_state const *seq = acs_seq_state_from_ctx(ctx);

	if (seq && seq->desc && seq->desc->on_abort) {
		seq->desc->on_abort(ctx);
	}

	acs_seq_clear(ctx);
}

void acs_seq_on_cp_confirm(struct bt_conn *conn, const struct bt_gatt_attr *attr)
{
	struct bt_acs_conn *acs_conn = acs_conn_lookup(conn);
	struct acs_cp_ctx ctx;
	int err;

	if (!acs_conn) {
		return;
	}

	ctx = (struct acs_cp_ctx){
		.prot_req = NULL,
		.conn = conn,
		.attr = attr,
		.acs_conn = acs_conn,
	};

	err = acs_seq_continue(&ctx);
	if (err) {
		LOG_WRN("Plain CP reply sequence advance failed: %d", err);
		acs_seq_abort(&ctx);
	}
}

void acs_seq_on_req_confirm(struct bt_acs_prot_resource_req *req, struct bt_conn *conn,
			    const struct bt_gatt_attr *attr)
{
	struct acs_cp_ctx ctx;
	int err;

	if (!req || !req->reply_seq.desc) {
		return;
	}

	ctx = (struct acs_cp_ctx){
		.prot_req = req,
		.conn = conn,
		.attr = attr,
		.acs_conn = req->acs_conn ? req->acs_conn : acs_conn_lookup(conn),
	};

	err = acs_seq_continue(&ctx);
	if (err) {
		LOG_WRN("Protected CP reply sequence advance failed for handle 0x%04x: %d",
			req->resource_handle, err);
		acs_seq_abort(&ctx);
	}
}
