/*
 * Copyright (c) 2026 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>

#include "acs_internal.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(bt_acs, CONFIG_BT_ACS_LOG_LEVEL);

/**
 * @brief Resolve the reply-sequence state behind an owner view.
 *
 * Single source of truth — both plain-CP and protected procedures share the
 * same @c reply_seq home on the unified @c acs_procedure object.
 */
static struct acs_reply_seq_state *acs_seq_state(const struct acs_exec_owner *owner)
{
	acs_procedure *proc;

	if (!owner) {
		return NULL;
	}
	proc = acs_owner_proc(owner);
	return proc ? &proc->reply_seq : NULL;
}

/**
 * @brief Advance to the next step in the active reply sequence.
 *
 * @return 0 on success or sequence complete, negative errno from the step
 *         function on failure.
 */
static int acs_seq_continue(const struct acs_exec_owner *owner)
{
	struct acs_reply_seq_state *seq = acs_seq_state(owner);

	if (!seq || !seq->desc || seq->step >= seq->desc->step_count) {
		acs_seq_clear(owner);
		return 0;
	}

	acs_seq_step_fn fn = seq->desc->steps[seq->step];
	seq->step++;

	return fn(owner);
}

bool acs_seq_active(const struct acs_exec_owner *owner)
{
	struct acs_reply_seq_state const *seq = acs_seq_state(owner);

	return seq && seq->desc != NULL;
}

void acs_seq_begin(const struct acs_exec_owner *owner, const struct acs_seq_desc *desc)
{
	struct acs_reply_seq_state *seq = acs_seq_state(owner);

	__ASSERT_NO_MSG(seq != NULL);

	/* The ALLOC (owner) ref keeps the request alive for the full duration
	 * of the reply sequence. acs_runtime_dispatch_protected_cp_frame()
	 * defers release_owner() when it sees reply_seq.desc != NULL after
	 * dispatch, and acs_seq_clear() calls release_owner() when the
	 * sequence completes or aborts.
	 */
	seq->desc = desc;
	seq->step = 0;
}

void acs_seq_clear(const struct acs_exec_owner *owner)
{
	struct acs_reply_seq_state *seq = acs_seq_state(owner);
	bool was_active;

	if (!seq) {
		return;
	}

	was_active = seq->desc != NULL;
	memset(seq, 0, sizeof(*seq));

	/* The sequence owned the ALLOC ref for its duration (deferred from
	 * acs_runtime_dispatch_protected_cp_frame). Release it now that the
	 * sequence is done.
	 */
	if (was_active && owner->kind == ACS_EXEC_OWNER_PROTECTED_REQ && owner->req != NULL) {
		acs_procedure_release_owner(owner->req);
	}
}

void acs_seq_abort(const struct acs_exec_owner *owner)
{
	struct acs_reply_seq_state const *seq = acs_seq_state(owner);

	if (seq && seq->desc && seq->desc->on_abort) {
		seq->desc->on_abort(owner);
	}

	acs_seq_clear(owner);
}

void acs_seq_on_owner_confirm(const struct acs_exec_owner *owner)
{
	int err;

	if (!owner || !owner->acs_conn) {
		return;
	}

	/* Skip protected-side fast-out if there is nothing to advance. The plain-CP
	 * path always tries to advance — acs_seq_continue clears stale state on no-op.
	 */
	if (owner->kind == ACS_EXEC_OWNER_PROTECTED_REQ &&
	    (!owner->req || !owner->req->reply_seq.desc)) {
		return;
	}

	err = acs_seq_continue(owner);
	if (err) {
		if (owner->kind == ACS_EXEC_OWNER_PROTECTED_REQ) {
			LOG_WRN("Protected CP reply sequence advance failed for handle 0x%04x: %d",
				owner->req->resource_handle, err);
		} else {
			LOG_WRN("Plain CP reply sequence advance failed: %d", err);
		}
		acs_seq_abort(owner);
	}
}
