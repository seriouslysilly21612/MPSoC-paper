/*****************************************************************************
*	Name: obs_receiver.h
*	Description: The control task's side of the camera results (eq. (3),
*	Alg. 1; D-19, D-20, D-27): which result a period may use, and what became
*	of every other one. C, on CPU3 now and on the R5 later.
*
*	  states   the joint angles of the last 256 periods, by period number.
*	           A result brings its capture time t_e; the angles at t_e come
*	           from the two states around it. Without two such states, or
*	           with the two more than 1.5 periods apart, the state at t_e
*	           cannot be recovered.
*	  held     up to 8 results out of the mailbox, the newest capture first:
*	           one that this period cannot use may do for the next. A ninth
*	           pushes the oldest capture out.
*	  period   obs_step first takes what the mailbox holds, one look per slot,
*	           and checks each result once: consistent, before its deadline,
*	           the state. Then it gives the newest held capture that was
*	           published before this release, is before its deadline and,
*	           under OBS_CONTRACT, has its error bound (eq. (7)) within the
*	           tolerance of the period. What is older than that one goes, and
*	           an older capture that comes later is not taken: the results
*	           used come in the order of their captures.
*	  policy   OBS_CONTRACT: all of this. OBS_MAX_AGE: no error bound, and
*	           the angles of the period of arrival in place of those at t_e.
*	  verdict  every result taken leaves with exactly one (OBS_V_*); those of
*	           a period are in rec[] after obs_step.
*
*	The work of a period is bounded: 8 slots, 8 held results, 3 pairs of
*	states per result. No lock, no allocation, no system call.
*****************************************************************************/
#ifndef G2T_OBS_RECEIVER_H
#define G2T_OBS_RECEIVER_H

#include "obs_mailbox.h"

#ifdef __cplusplus
extern "C" {
#endif

enum { OBS_DOF = 6, OBS_STATES = 256, OBS_HELD = 8, OBS_RECORDS = OBS_SLOTS + OBS_HELD };
enum { OBS_CONTRACT = 0, OBS_MAX_AGE = 1 };		/* policy */

typedef struct
{
	int			policy;
	uint64_t	period_ticks;	/* the period of the control task, ticks of the system counter */
	uint64_t	age_cap_ticks;	/* no result is used later than this after its capture, whatever its deadline says */
	double		ticks_per_s;
	double		e0, g, eps_v, a_max, eps_cal;	/* eq. (7): [m], 1, [m/s], [m/s^2], [m] */
} obs_settings_t;

/* the state of one period: 32 bytes. t = 0: no state of that period */
typedef struct
{
	uint64_t	t;				/* the time the angles belong to */
	float		q[OBS_DOF];		/* [rad] */
} obs_state_t;

/* a result out of the mailbox */
typedef struct
{
	obs_desc_t	d;				/* the copy. d.s: the period of the state in q; d.t_valid_until: at most t_e + age_cap_ticks */
	float		q[OBS_DOF];		/* the joint angles at t_e (OBS_CONTRACT), of the period of arrival (OBS_MAX_AGE) */
	float		ebar;			/* [m] eq. (7) in the last period that checked it */
	uint64_t	t_in;			/* the release of the period that took it */
	uint8_t		accepted;		/* used at least once: its verdict is out */
	uint8_t		over;			/* eq. (7) was above the tolerance at least once */
} obs_held_t;

enum
{
	OBS_V_ACCEPTED = 0,			/* a period used it */
	OBS_V_SUPERSEDED,			/* a newer capture was used first */
	OBS_V_EXPIRED,				/* past its deadline, on arrival or while held */
	OBS_V_STATE_UNRECOVERABLE,	/* no state at t_e, or the tag was not seen */
	OBS_V_SEQ_FAIL,				/* not a result of this epoch (OBS_STALE), held already, or one that cannot be:
								 * captured after its publication or after this release, a sigma that is no error */
	OBS_V_ERROR_EXCEEDED,		/* eq. (7) above the tolerance until its deadline */
	OBS_V_EVICTED,				/* the oldest of nine */
	OBS_VERDICTS
};

typedef struct
{
	obs_held_t	h;
	uint64_t	t_out;			/* the release of the period of the verdict */
	int			verdict;
} obs_record_t;

typedef struct
{
	obs_settings_t	set;
	obs_state_t		state[OBS_STATES];		/* [k % OBS_STATES]: the state of period k */
	uint32_t		k_new;					/* the period of the newest state */
	int				have_state;
	obs_held_t		held[OBS_HELD];			/* [0]: the newest capture */
	unsigned		n_held;
	uint64_t		t_used;					/* the capture of the newest result used so far; 0: none */
	obs_record_t	rec[OBS_RECORDS];		/* the verdicts of this period: at most one per slot and per held result */
	unsigned		n_rec;
	uint32_t		count[OBS_VERDICTS];	/* verdicts since the start */
	uint32_t		taken;					/* results out of the mailbox since the start */
} obs_receiver_t;

/* the control task starts (before its first obs_step; its side of the mailbox: obs_reader_start).
 * 0: settings that cannot be used. The states must reach back to the oldest capture a result may have. */
static inline int obs_receiver_start(obs_receiver_t* r, const obs_settings_t* set)
{
	if ((set->policy != OBS_CONTRACT && set->policy != OBS_MAX_AGE) || set->period_ticks == 0u ||
	    !(set->ticks_per_s > 0.0) || set->age_cap_ticks / set->period_ticks >= OBS_STATES - 3u)
		return 0;
	memset(r, 0, sizeof(*r));
	r->set = *set;
	return 1;
}

/* the state of period k: the joint angles q[OBS_DOF] and the time t (> 0) they belong to. Consecutive periods
 * have consecutive k. */
static inline void obs_state_push(obs_receiver_t* r, uint32_t k, uint64_t t, const float* q)
{
	obs_state_t* p = &r->state[k % OBS_STATES];

	p->t = t;
	memcpy(p->q, q, sizeof(p->q));
	r->k_new = k;
	r->have_state = 1;
}

/* the joint angles at t_e, between the two states around it; *s: the period of the earlier one. 0: no such states. */
static inline int obs_state_at(const obs_receiver_t* r, uint64_t t_e, float* q, uint32_t* s)
{
	static const unsigned char back_of[3] = { 1, 0, 2 };	/* the pair the times point at, then its neighbours: the times jitter */
	const obs_state_t *a, *b;
	uint64_t t_new, back;
	uint32_t p;
	unsigned i, j;
	float w;

	if (!r->have_state)
		return 0;
	t_new = r->state[r->k_new % OBS_STATES].t;
	if (t_e >= t_new)
		return 0;
	back = (t_new - t_e) / r->set.period_ticks;		/* whole periods from t_e to the newest state */
	for (i = 0; i < 3u; i++)
	{
		if (back + back_of[i] < 1u || back + back_of[i] > OBS_STATES - 1u)
			continue;
		p = r->k_new - (uint32_t)(back + back_of[i]);
		a = &r->state[p % OBS_STATES];
		b = &r->state[(p + 1u) % OBS_STATES];
		/* the times decide: an entry of a period without a state is 256 periods old, or 0 */
		if (a->t == 0u || a->t > t_e || t_e >= b->t || b->t - a->t > r->set.period_ticks + r->set.period_ticks / 2u)
			continue;
		w = (float)(t_e - a->t) / (float)(b->t - a->t);
		for (j = 0; j < OBS_DOF; j++)
			q[j] = a->q[j] + w * (b->q[j] - a->q[j]);
		*s = p;
		return 1;
	}
	return 0;
}

/* eq. (7): the bound of the task error that a result of this sigma [m] causes at the age age_s [s] */
static inline double obs_error_bound(const obs_settings_t* set, double sigma, double age_s)
{
	return set->e0 + set->g * (sigma + set->eps_v * age_s + 0.5 * set->a_max * age_s * age_s + set->eps_cal);
}

static inline void obs_verdict(obs_receiver_t* r, const obs_held_t* h, int verdict, uint64_t t_k)
{
	obs_record_t* p = &r->rec[r->n_rec++];

	p->h = *h;
	p->t_out = t_k;
	p->verdict = verdict;
	r->count[verdict]++;
}

/* one period: its release t_k and its tolerance tol [m] (delta of the contract). Once per period, after
 * obs_state_push. Returns the result to use, or NULL: none. *fresh: it is used for the first time.
 * The pointer holds until the next obs_step. */
static inline const obs_held_t* obs_step(obs_receiver_t* r, volatile obs_mailbox_t* mb, uint64_t t_k, double tol,
                                         int* fresh)
{
	obs_held_t h, *p;
	unsigned b, i, pos, keep = 0, use = OBS_HELD;
	int st, ok;
	double e;

	r->n_rec = 0;
	*fresh = 0;

	/* Alg. 1, lines 2-5: what the mailbox holds, each result checked once */
	for (b = 0; b < OBS_SLOTS; b++)
	{
		memset(&h, 0, sizeof(h));
		st = obs_take(mb, b, &h.d);
		if (st == OBS_NO_PEER)
			break;
		if (st == OBS_EMPTY)
			continue;
		r->taken++;
		h.t_in = t_k;
		for (i = 0; i < r->n_held; i++)
			if (r->held[i].d.n == h.d.n && r->held[i].d.epoch == h.d.epoch)
				st = OBS_STALE;
		if (st != OBS_OK || h.d.t_e > h.d.f || h.d.t_e > t_k || !(h.d.sigma >= 0.0f))
		{
			obs_verdict(r, &h, OBS_V_SEQ_FAIL, t_k);
			continue;
		}
		if (h.d.t_valid_until > h.d.t_e + r->set.age_cap_ticks)		/* the writer's deadline is not trusted beyond the cap */
			h.d.t_valid_until = h.d.t_e + r->set.age_cap_ticks;
		if (t_k > h.d.t_valid_until)
		{
			obs_verdict(r, &h, OBS_V_EXPIRED, t_k);
			continue;
		}
		if (h.d.t_e <= r->t_used)
		{
			obs_verdict(r, &h, OBS_V_SUPERSEDED, t_k);
			continue;
		}
		if (h.d.flags & OBS_TAG_LOST)
			ok = 0;
		else if (r->set.policy == OBS_CONTRACT)
			ok = obs_state_at(r, h.d.t_e, h.q, &h.d.s);
		else if (r->have_state)
		{
			memcpy(h.q, r->state[r->k_new % OBS_STATES].q, sizeof(h.q));
			h.d.s = r->k_new;
			ok = 1;
		}
		else
			ok = 0;
		if (!ok)
		{
			obs_verdict(r, &h, OBS_V_STATE_UNRECOVERABLE, t_k);
			continue;
		}
		for (pos = 0; pos < r->n_held && r->held[pos].d.t_e >= h.d.t_e; pos++)
			;
		if (r->n_held == OBS_HELD)
		{
			if (pos == OBS_HELD)							/* itself the oldest capture */
			{
				obs_verdict(r, &h, OBS_V_EVICTED, t_k);
				continue;
			}
			if (!r->held[OBS_HELD - 1].accepted)
				obs_verdict(r, &r->held[OBS_HELD - 1], OBS_V_EVICTED, t_k);
			r->n_held--;
		}
		memmove(&r->held[pos + 1], &r->held[pos], (r->n_held - pos) * sizeof(h));
		r->held[pos] = h;
		r->n_held++;
	}

	/* Alg. 1, lines 6-8: the newest capture that this period may use; the list closes up over what goes */
	for (i = 0; i < r->n_held; i++)
	{
		p = &r->held[i];
		if (use != OBS_HELD)								/* older than the one in use */
		{
			if (!p->accepted)
				obs_verdict(r, p, OBS_V_SUPERSEDED, t_k);
			continue;
		}
		if (t_k > p->d.t_valid_until)
		{
			if (!p->accepted)
				obs_verdict(r, p, p->over ? OBS_V_ERROR_EXCEEDED : OBS_V_EXPIRED, t_k);
			continue;
		}
		if (p->d.f <= t_k)									/* else published after this release: the next period */
		{
			e = obs_error_bound(&r->set, p->d.sigma, (double)(t_k - p->d.t_e) / r->set.ticks_per_s);
			p->ebar = (float)e;
			if (r->set.policy != OBS_CONTRACT || e <= tol)
				use = keep;
			else
				p->over = 1;
		}
		if (keep != i)
			r->held[keep] = *p;
		keep++;
	}
	r->n_held = keep;
	if (use == OBS_HELD)
		return NULL;
	p = &r->held[use];
	if (!p->accepted)
	{
		p->accepted = 1;
		r->t_used = p->d.t_e;
		*fresh = 1;
		obs_verdict(r, p, OBS_V_ACCEPTED, t_k);
	}
	return p;
}

#ifdef __cplusplus
}
#endif

#endif /* G2T_OBS_RECEIVER_H */
