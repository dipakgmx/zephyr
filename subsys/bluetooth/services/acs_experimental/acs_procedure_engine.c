/*
 * Copyright (c) 2026 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>

#include "acs_internal.h"

/** See @ref acs_procedure_engine_start. */
int acs_procedure_engine_start(struct acs_procedure *proc, const struct acs_frame *frame)
{
	int result;

	if (!proc || !proc->ops || !proc->ops->start || !frame) {
		return -EINVAL;
	}

	proc->status = ACS_PROC_RUNNING;
	result = proc->ops->start(proc, frame);
	if (result == ACS_PROC_RES_COMPLETE) {
		proc->status = ACS_PROC_COMPLETE;
	} else if (result == ACS_PROC_STEP_WAIT_IND_CONFIRM) {
		proc->status = ACS_PROC_WAIT_CONFIRM;
	}

	return result;
}

/** See @ref acs_procedure_engine_on_confirm. */
int acs_procedure_engine_on_confirm(struct acs_procedure *proc)
{
	int result;

	if (!proc || !proc->ops || !proc->ops->on_confirm) {
		return -EINVAL;
	}

	result = proc->ops->on_confirm(proc);
	if (result == ACS_PROC_RES_COMPLETE) {
		proc->status = ACS_PROC_COMPLETE;
	} else if (result == ACS_PROC_STEP_WAIT_IND_CONFIRM) {
		proc->status = ACS_PROC_WAIT_CONFIRM;
	}

	return result;
}

/** See @ref acs_procedure_engine_abort. */
void acs_procedure_engine_abort(struct acs_procedure *proc, int reason)
{
	if (!proc) {
		return;
	}

	proc->status = ACS_PROC_ABORTED;
	if (proc->ops && proc->ops->on_abort) {
		proc->ops->on_abort(proc, reason);
	}
}

/** See @ref acs_procedure_engine_reset. */
void acs_procedure_engine_reset(struct acs_procedure *proc)
{
	if (!proc) {
		return;
	}

	if (proc->pending_reply.plaintext) {
		acs_channel_buf_free(proc->pending_reply.plaintext);
		proc->pending_reply.plaintext = NULL;
	}

	if (proc->ops && proc->ops->destroy) {
		proc->ops->destroy(proc);
	}

	memset(proc, 0, sizeof(*proc));
}
