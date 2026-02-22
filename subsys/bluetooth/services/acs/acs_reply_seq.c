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
 * @brief Resolve the reply sequence state from a CP context.
 *
 * Protected CP procedures store their sequence state in the request context;
 * plain CP procedures store it in the per-connection cp_proc.
 *
 * @param ctx  CP dispatch context.
 * @return Pointer to the active reply sequence state, or NULL if ctx is invalid.
 */
static struct acs_reply_seq_state *acs_seq_state_from_ctx(const struct acs_cp_ctx *ctx)
{
	if (!ctx) {
		return NULL;
	}

	if (ctx->prot_req) {
		return &ctx->prot_req->reply_seq;
	}

	if (ctx->acs_conn) {
		return &ctx->acs_conn->cp_proc.reply_seq;
	}

	return NULL;
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

	if (ctx->prot_req &&
	    !atomic_test_bit(ctx->prot_req->ref_flags, PROT_RESOURCE_REF_REPLY_CHAIN)) {
		acs_prot_resource_req_ref(ctx->prot_req, PROT_RESOURCE_REF_REPLY_CHAIN);
	}

	seq->desc = desc;
	seq->step = 0;
}

void acs_seq_clear(struct acs_cp_ctx *ctx)
{
	struct acs_reply_seq_state *seq = acs_seq_state_from_ctx(ctx);
	bool drop_ref;

	if (!seq) {
		return;
	}

	drop_ref = ctx->prot_req &&
		   atomic_test_bit(ctx->prot_req->ref_flags, PROT_RESOURCE_REF_REPLY_CHAIN);
	memset(seq, 0, sizeof(*seq));

	if (drop_ref) {
		acs_prot_resource_req_unref(ctx->prot_req, PROT_RESOURCE_REF_REPLY_CHAIN);
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
