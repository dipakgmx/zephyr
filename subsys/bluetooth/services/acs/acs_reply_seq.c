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
 * @brief Resolve the reply-sequence state on a procedure.
 *
 * Both plain-CP and protected procedures share the same @c reply_seq home on
 * the unified @c acs_procedure object — single source of truth.
 */
static struct acs_reply_seq_state *acs_seq_state(struct acs_procedure *proc)
{
	return proc ? &proc->reply_seq : NULL;
}

/**
 * @brief Advance to the next step in the active reply sequence.
 *
 * @return 0 on success or sequence complete, negative errno from the step
 *         function on failure.
 */
static int acs_seq_continue(struct acs_procedure *proc)
{
	struct acs_reply_seq_state *seq = acs_seq_state(proc);

	if (!seq || !seq->desc || seq->step >= seq->desc->step_count) {
		acs_seq_clear(proc);
		return 0;
	}

	acs_seq_step_fn fn = seq->desc->steps[seq->step];
	seq->step++;

	return fn(proc);
}

bool acs_seq_active(struct acs_procedure *proc)
{
	struct acs_reply_seq_state const *seq = acs_seq_state(proc);

	return seq && seq->desc != NULL;
}

void acs_seq_begin(struct acs_procedure *proc, const struct acs_seq_desc *desc)
{
	struct acs_reply_seq_state *seq = acs_seq_state(proc);

	__ASSERT_NO_MSG(seq != NULL);

	/* The ALLOC ref keeps the request alive for the full duration of the
	 * reply sequence. acs_runtime_dispatch_protected_cp_frame() defers
	 * release_owner() when it sees reply_seq.desc != NULL after dispatch,
	 * and acs_seq_clear() calls release_owner() when the sequence completes
	 * or aborts.
	 */
	seq->desc = desc;
	seq->step = 0;
}

void acs_seq_clear(struct acs_procedure *proc)
{
	struct acs_reply_seq_state *seq = acs_seq_state(proc);
	bool was_active;

	if (!seq) {
		return;
	}

	was_active = seq->desc != NULL;
	memset(seq, 0, sizeof(*seq));

	/* The sequence owned the ALLOC ref for its duration (deferred from
	 * acs_runtime_dispatch_protected_cp_frame). Release it now that the
	 * sequence is done. Singleton plain-CP procedures don't carry an
	 * ALLOC ref — only slab-allocated protected procedures do.
	 */
	if (was_active && proc->kind == ACS_PROC_KIND_PROTECTED_REQ) {
		acs_procedure_release_owner(proc);
	}
}

void acs_seq_abort(struct acs_procedure *proc)
{
	struct acs_reply_seq_state const *seq = acs_seq_state(proc);

	if (seq && seq->desc && seq->desc->on_abort) {
		seq->desc->on_abort(proc);
	}

	acs_seq_clear(proc);
}

void acs_seq_on_confirm(struct acs_procedure *proc)
{
	int err;

	if (!proc || !proc->acs_conn) {
		return;
	}

	/* Protected-side fast-out: if no sequence is staged, nothing to advance.
	 * Plain CP always falls through — acs_seq_continue clears stale state
	 * cleanly when there's nothing to do.
	 */
	if (proc->kind == ACS_PROC_KIND_PROTECTED_REQ && !proc->reply_seq.desc) {
		return;
	}

	err = acs_seq_continue(proc);
	if (err) {
		if (proc->kind == ACS_PROC_KIND_PROTECTED_REQ) {
			LOG_WRN("Protected CP reply sequence advance failed for handle 0x%04x: %d",
				proc->route.resource_handle, err);
		} else {
			LOG_WRN("Plain CP reply sequence advance failed: %d", err);
		}
		acs_seq_abort(proc);
	}
}
