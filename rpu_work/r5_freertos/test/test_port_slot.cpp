/* host test of src/ctrl/port/base/AtomicSlot.h: the slot between two tasks on one core, with the task of the
 * higher priority running at every place where the other touches the slot (slot_preempt_point()).  From r5_freertos/:
 *   g++ -std=gnu++17 -Wall -DSLOT_PREEMPT_TEST -Isrc/ctrl/port test/test_port_slot.cpp -o /tmp/t_slot && /tmp/t_slot
 * Every word of a payload is the number of that payload, so a copy of two payloads mixed shows.  It checks:
 * nothing published reads as nothing; a reader that interrupts a write anywhere after its start gets nothing and
 * its destination and the count stay as they were, and before the start the payload before, whole; a reader that
 * a write interrupts anywhere gets the new payload, whole; one that a write interrupts in every attempt gets
 * nothing, its destination unchanged. */
#include <cstdio>
#include <cstring>

#include "base/AtomicSlot.h"

static int fails;
#define CHECK(c) do { if (!(c)) { printf("FAIL line %d (place %d): %s\n", __LINE__, at, #c); fails++; } } while (0)

struct payload_t
{
    uint32_t word[9];
};
typedef CAtomicSlot<payload_t> slot_t;
enum { PLACES = slot_t::WORDS + 2 };        /* of a write, and of an attempt to read */

static payload_t payload(uint32_t number)
{
    payload_t p;
    for (uint32_t &w : p.word)
        w = number;
    return p;
}
/* the number of a whole payload; 0: mixed */
static uint32_t whole(const payload_t &p)
{
    for (uint32_t w : p.word)
        if (w != p.word[0])
            return 0;
    return p.word[0];
}

/* the other task: at place `at` of what the caller does (-1: never; with `every`, at every place) */
static slot_t *slot;
static int at = -1, place, every;
static bool inside;
static void (*other)(void);

extern "C" void slot_preempt_point(void)
{
    if (inside || other == NULL || (place++ != at && !every))
        return;
    inside = true;          /* the other task touches the slot as well */
    other();
    inside = false;
}

/* the reader as the other task: what it got */
static bool got;
static payload_t seen;
static uint32_t seen_count, count;
static void reads(void)
{
    seen = payload(77);
    seen_count = 77;
    got = slot->Read(seen, &seen_count);
    count = slot->Count();
}

/* the writer as the other task */
static uint32_t next_number;
static int writes_done;
static void writes(void)
{
    slot->Write(payload(next_number++));
    writes_done++;
}

int main()
{
    CHECK(slot_t::IsLockFree() && (int)slot_t::WORDS == 9);

    /* nothing published */
    {
        slot_t s;
        payload_t p = payload(77);
        uint32_t n = 77;
        CHECK(!s.Read(p, &n) && whole(p) == 77 && n == 77 && s.Count() == 0);
    }

    /* a reader interrupts the write of payload 2, at every place of it */
    for (at = 0; at < PLACES; at++)
    {
        slot_t s;
        slot = &s;
        s.Write(payload(1));
        other = reads;
        place = 0;
        got = false;
        seen = payload(0);
        s.Write(payload(2));
        other = NULL;
        if (at == 0)        /* before the write has marked the slot: payload 1 */
            CHECK(got && whole(seen) == 1 && seen_count == 1 && count == 1);
        else                /* in the middle: nothing, and nothing changed */
            CHECK(!got && whole(seen) == 77 && seen_count == 77 && count == 1);
        payload_t p = payload(77);
        uint32_t n = 0;
        CHECK(place == PLACES && s.Read(p, &n) && whole(p) == 2 && n == 2 && s.Count() == 2);
    }

    /* the write of payload 2 interrupts a reader, at every place of its first attempt: it reads once more */
    for (at = 0; at < PLACES; at++)
    {
        slot_t s;
        slot = &s;
        s.Write(payload(1));
        other = writes;
        next_number = 2;
        writes_done = 0;
        place = 0;
        payload_t p = payload(77);
        uint32_t n = 77;
        const bool ok = s.Read(p, &n);
        other = NULL;
        CHECK(writes_done == 1 && ok && whole(p) == 2 && n == 2);
        CHECK(place == (at == 0 ? PLACES : 2 * PLACES));
    }

    /* a write interrupts every attempt of a reader: nothing, and its destination as it was */
    {
        at = -1;
        slot_t s;
        slot = &s;
        s.Write(payload(1));
        other = writes;
        next_number = 2;
        writes_done = 0;
        place = 0;
        every = 1;
        payload_t p = payload(77);
        uint32_t n = 77;
        const bool ok = s.Read(p, &n);
        every = 0;
        other = NULL;
        CHECK(!ok && whole(p) == 77 && n == 77 && writes_done == (int)slot_t::MAX_ATTEMPTS * PLACES);
        CHECK(s.Read(p, &n) && whole(p) == 1 + (uint32_t)writes_done && n == 1 + (uint32_t)writes_done);
    }

    printf(fails ? "test_port_slot: %d FAILED\n" : "test_port_slot OK\n", fails);
    return fails != 0;
}
