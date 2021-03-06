/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file param.c
 *
 * Global parameter store.
 *
 * Note that it might make sense to convert this into a driver.  That would
 * offer some interesting options regarding state for e.g. ORB advertisements
 * and background parameter saving.
 */

#include <debug.h>
#include <string.h>
#include <stdbool.h>
#include <fcntl.h>
#include <unistd.h>
#include <systemlib/err.h>
#include <errno.h>
#include <semaphore.h>

#include <sys/stat.h>

#include <drivers/drv_hrt.h>

#include "systemlib/param/param.h"
#include "systemlib/uthash/utarray.h"
#include "systemlib/bson/tinybson.h"

#include "uORB/uORB.h"
#include "uORB/topics/parameter_update.h"

#if 1
# define debug(fmt, args...)		do { warnx(fmt, ##args); } while(0)
#else
# define debug(fmt, args...)		do { } while(0)
#endif

/**
 * Array of static parameter info.
 */
extern char __param_start, __param_end;
static const struct param_info_s	*param_info_base = (struct param_info_s *) &__param_start;
static const struct param_info_s	*param_info_limit = (struct param_info_s *) &__param_end;
#define	param_info_count		((unsigned)(param_info_limit - param_info_base))

/**
 * Storage for modified parameters.
 */
struct param_wbuf_s {
	param_t			param;
	union param_value_u	val;
	bool			unsaved;
};

/** flexible array holding modified parameter values */
UT_array	*param_values;

/** array info for the modified parameters array */
const UT_icd	param_icd = {sizeof(struct param_wbuf_s), NULL, NULL, NULL};

/** parameter update topic */
ORB_DEFINE(parameter_update, struct parameter_update_s);

/** parameter update topic handle */
static orb_advert_t param_topic = -1;

static sem_t param_sem = { .semcount = 1 };

/** lock the parameter store */
static void
param_lock(void)
{
	//do {} while (sem_wait(&param_sem) != 0);
}

/** unlock the parameter store */
static void
param_unlock(void)
{
	//sem_post(&param_sem);
}

/** assert that the parameter store is locked */
static void
param_assert_locked(void)
{
	/* XXX */
}

/**
 * Test whether a param_t is value.
 *
 * @param param			The parameter handle to test.
 * @return			True if the handle is valid.
 */
static bool
handle_in_range(param_t param)
{
	return (param < param_info_count);
}

/**
 * Compare two modifid parameter structures to determine ordering.
 *
 * This function is suitable for passing to qsort or bsearch.
 */
static int
param_compare_values(const void *a, const void *b)
{
	struct param_wbuf_s *pa = (struct param_wbuf_s *)a;
	struct param_wbuf_s *pb = (struct param_wbuf_s *)b;

	if (pa->param < pb->param)
		return -1;

	if (pa->param > pb->param)
		return 1;

	return 0;
}

/**
 * Locate the modified parameter structure for a parameter, if it exists.
 *
 * @param param			The parameter being searched.
 * @return			The structure holding the modified value, or
 *				NULL if the parameter has not been modified.
 */
static struct param_wbuf_s *
param_find_changed(param_t param) {
	struct param_wbuf_s	*s = NULL;

	param_assert_locked();

	if (param_values != NULL) {
#if 0	/* utarray_find requires bsearch, not available */
		struct param_wbuf_s key;
		key.param = param;
		s = utarray_find(param_values, &key, param_compare_values);
#else

		while ((s = (struct param_wbuf_s *)utarray_next(param_values, s)) != NULL) {
			if (s->param == param)
				break;
		}

#endif
	}

	return s;
}

static void
param_notify_changes(void)
{
	struct parameter_update_s pup = { .timestamp = hrt_absolute_time() };

	/*
	 * If we don't have a handle to our topic, create one now; otherwise
	 * just publish.
	 */
	if (param_topic == -1) {
		param_topic = orb_advertise(ORB_ID(parameter_update), &pup);

	} else {
		orb_publish(ORB_ID(parameter_update), param_topic, &pup);
	}
}

param_t
param_find(const char *name)
{
	param_t param;

	/* perform a linear search of the known parameters */
	for (param = 0; handle_in_range(param); param++) {
		if (!strcmp(param_info_base[param].name, name))
			return param;
	}

	/* not found */
	return PARAM_INVALID;
}

unsigned
param_count(void)
{
	return param_info_count;
}

param_t
param_for_index(unsigned index)
{
	if (index < param_info_count)
		return (param_t)index;

	return PARAM_INVALID;
}

int
param_get_index(param_t param)
{
	if (handle_in_range(param))
		return (unsigned)param;

	return -1;
}

const char *
param_name(param_t param)
{
	if (handle_in_range(param))
		return param_info_base[param].name;

	return NULL;
}

bool
param_value_is_default(param_t param)
{
	return param_find_changed(param) ? false : true;
}

bool
param_value_unsaved(param_t param)
{
	static struct param_wbuf_s *s;

	s = param_find_changed(param);

	if (s && s->unsaved)
		return true;

	return false;
}

enum param_type_e
param_type(param_t param)
{
	if (handle_in_range(param))
		return param_info_base[param].type;

	return PARAM_TYPE_UNKNOWN;
}

size_t
param_size(param_t param)
{
	if (handle_in_range(param)) {
		switch (param_type(param)) {
		case PARAM_TYPE_INT32:
		case PARAM_TYPE_FLOAT:
			return 4;

		case PARAM_TYPE_STRUCT ... PARAM_TYPE_STRUCT_MAX:
			/* decode structure size from type value */
			return param_type(param) - PARAM_TYPE_STRUCT;

		default:
			return 0;
		}
	}

	return 0;
}

/**
 * Obtain a pointer to the storage allocated for a parameter.
 *
 * @param param			The parameter whose storage is sought.
 * @return			A pointer to the parameter value, or NULL
 *				if the parameter does not exist.
 */
static const void *
param_get_value_ptr(param_t param)
{
	const void *result = NULL;

	param_assert_locked();

	if (handle_in_range(param)) {

		const union param_value_u *v;

		/* work out whether we're fetching the default or a written value */
		struct param_wbuf_s *s = param_find_changed(param);

		if (s != NULL) {
			v = &s->val;

		} else {
			v = &param_info_base[param].val;
		}

		if (param_type(param) == PARAM_TYPE_STRUCT) {
			result = v->p;

		} else {
			result = v;
		}
	}

	return result;
}

int
param_get(param_t param, void *val)
{
	int result = -1;

	param_lock();

	const void *v = param_get_value_ptr(param);

	if (val != NULL) {
		memcpy(val, v, param_size(param));
		result = 0;
	}

	param_unlock();

	return result;
}


int
param_set(param_t param, const void *val)
{
	return param_set_internal(param, val, false);
}

void
param_reset(param_t param)
{
	struct param_wbuf_s *s = NULL;

	param_lock();

	if (handle_in_range(param)) {

		/* look for a saved value */
		s = param_find_changed(param);

		/* if we found one, erase it */
		if (s != NULL) {
			int pos = utarray_eltidx(param_values, s);
			utarray_erase(param_values, pos, 1);
		}
	}

	param_unlock();

	if (s != NULL)
		param_notify_changes();
}

void
param_reset_all(void)
{
	param_lock();

	if (param_values != NULL) {
		utarray_free(param_values);
	}

	/* mark as reset / deleted */
	param_values = NULL;

	param_unlock();

	param_notify_changes();
}

