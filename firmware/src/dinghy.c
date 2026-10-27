/* SPDX-License-Identifier: GPL-3.0-only */
/* fm1-midi-capture: a retrospective MIDI looper / clip sequencer for the FM-1, built on Dinghy.
 *
 * Always-listening retrospective capture (like Ableton Move / Live "Capture MIDI").
 * Every note you play is logged to the active track's rolling history (the last 16 bars). You never
 * "start" recording -- you just play. Pressing REC looks BACK at what you just played, detects the
 * tempo and fits a whole number of bars (1..16), and launches that as a loop. The first capture sets
 * the project's BPM; later captures snap to that grid so every track stays locked.
 *
 *   SELECT     the active track (1..8)
 *   top 8 keys (keys 19..26)   the 8 clip slots of the active track:
 *                - filled slot -> launch it (stops the track's current clip)
 *                - the playing slot pressed again -> stop it
 *   bottom keys (0..18)        play the internal sine; always logged to history
 *   REC        retrospectively capture the recent history into the selected slot, detect BPM/bars,
 *              and start looping. Pressing REC while a loop plays overdubs onto it.
 *   PLAY       start/stop the active track's clip
 *   OCT- OCT+  the octave (kept in flash)
 *   PRESETS    the MIDI channel: OMNI (in: all, out: 1) or 1..16 (kept in flash)
 *   MASTER     the volume (platform.c)
 *   MIDI in    notes from USB or the TRS jack play the sines and are logged; a clock shows its tempo
 *
 * Dinghy's platform is untouched below. Rules (the chip has no FPU, no libgcc): integers only, no
 * float, no 64-bit division. tests/host.c runs this file on a computer (screen PNG, sound WAV).
 * What platform.c offers: docs/API.md. The chip's rules: docs/RULES.md. */
static void app_idle(void);
#define LCD_IDLE app_idle                       /* while the screen's DMA runs: MIDI in (platform.c) */
#include "platform.c"
#if BOARD_SMK37
#include "smk_wheel.h"
#endif

#ifndef APP_NAME
#define APP_NAME "CAPTURE"
#endif

/* what survives a power-off (declared early: the looper code below reads it) */
static struct { uint8_t oct, ch; } set = {0, 0};
static uint32_t out_channel(void) { return set.ch ? set.ch - 1u : 0u; }

/* ---- the sound (runs in the audio ISR: keep it short, no flash, no screen) */
#define NV 8u                                   /* voices (raised from Dinghy's 4: looper layers notes) */
static struct {
    volatile uint32_t inc;                      /* phase step per frame: f * 2^32 / 44100 (unbent) */
    volatile int32_t gate;                      /* target level, Q15: 0 = released */
    int32_t amp;                                /* the level now: glides to gate, no clicks */
    uint32_t ph, age;
    uint8_t note;
} voice[NV];
static volatile uint32_t bend_q16 = 65536;      /* the pitch wheel: the steps times this / 65536 */

static int32_t sine(uint32_t ph)                /* a full cycle over 2^32 -> -32767..32767 (a refined parabola, ~0.1 %) */
{
    int32_t x = (int32_t)ph >> 16, ax = x < 0 ? -x : x, y, ay;
    y = (x * (32768 - ax)) >> 13;               /* 4x(1-|x|), Q15 */
    ay = y < 0 ? -y : y;
    y += ((((y * ay) >> 15) - y) * 7373) >> 15; /* + 0.225 (y|y| - y) */
    return y > 32767 ? 32767 : y < -32767 ? -32767 : y;
}

static void app_audio(int32_t *out, uint32_t n)
{
    uint32_t v, i;
    for (v = 0; v < NV; v++) {
        uint32_t ph = voice[v].ph, step = (uint32_t)(((uint64_t)voice[v].inc * bend_q16) >> 16);   /* (int64 multiply: fine) */
        int32_t amp = voice[v].amp, g = voice[v].gate;
        if (!amp && !g)
            continue;                           /* silent: costs nothing */
        for (i = 0; i < n; i++) {
            int32_t s;
            if (amp < g) { amp += 48; if (amp > g) amp = g; }        /* attack ~5 ms */
            else if (amp > g) { amp -= 6; if (amp < g) amp = g; }    /* release ~45 ms */
            ph += step;
            s = (sine(ph) * amp) >> 15;
            out[2u * i] += s;                   /* L */
            out[2u * i + 1u] += s;              /* R */
        }
        voice[v].ph = ph;
        voice[v].amp = amp;
    }
}

/* ---- notes (main loop) */
static const uint32_t INC_C9[12] = {            /* MIDI 120..131 (C9..B9); lower octaves shift right */
    815363807u, 863847862u, 915214929u, 969636441u, 1027294024u, 1088380105u,
    1153098554u, 1221665363u, 1294309365u, 1371273005u, 1452813141u, 1539201906u};
static uint32_t ages, last_note = 255;

static void play(uint32_t note, uint32_t vel)
{
    uint32_t v, pick = 0, best = 0xFFFFFFFFu;
    if (note > 127u || !vel) return;
    /* A given pitch may occupy ONLY ONE voice: if this note already has a voice (even one in its
     * release tail, gate == 0 but amp > 0), reuse it. Two voices on the same pitch with different
     * phases beat against each other -> the "ring modulator" sound. */
    for (v = 0; v < NV; v++)
        if (voice[v].note == (uint8_t)note && (voice[v].gate || voice[v].amp)) { pick = v; goto set; }
    for (v = 0; v < NV; v++) {                  /* else a free voice, else the oldest */
        uint32_t score = voice[v].gate ? 0x80000000u + voice[v].age : (uint32_t)voice[v].amp + 1u;
        if (score < best) { best = score; pick = v; }
    }
set:
    voice[pick].note = (uint8_t)note;
    voice[pick].age = ++ages;
    voice[pick].inc = INC_C9[note % 12u] >> (10u - note / 12u);
    voice[pick].gate = (int32_t)(vel * 45u);    /* 127 -> 5715: eight voices stay under full scale */
    last_note = note;
}
static void stop(uint32_t note)
{
    uint32_t v;
    for (v = 0; v < NV; v++)
        if (voice[v].note == note)
            voice[v].gate = 0;
}
static int quiet(void)                          /* nothing sounds, not even a release */
{
    uint32_t v;
    for (v = 0; v < NV; v++)
        if (voice[v].gate || voice[v].amp) return 0;
    return 1;
}

/* ====================================================================================
 * LOOPER (retrospective, always-listening)
 *
 * HISTORY: every incoming/played event on the active track is appended to a per-track ring with an
 * absolute ms timestamp (fm1_ms). The ring always holds the last ~64 s (16 bars at the 60 BPM floor).
 *
 * CAPTURE: REC takes the span from the start of the recent phrase up to the REC press, detects the
 * tempo (folded into 80..160 BPM, the range Ableton targets), fits 1..16 whole bars, back-solves the
 * BPM so the loop is an exact whole number of bars (seamless), and launches it. The first capture sets
 * the global BPM; later captures snap to that bar grid.
 * ==================================================================================== */
#define NTRACK 8u
#define NSLOT  8u
#define CLIP_EVENTS 384u                        /* events per clip. 384*6 ~= 2.3 KB each */
#define HIST_EVENTS 512u                        /* per-track rolling history ring */
#define PAD0 19u                                /* key 19..26 = the 8 clip slots (top row); keys 0..18 play notes */
#define BPM_MIN 80u                             /* detection folds tempo into [80,160) */
#define BPM_MAX 160u
#define MAX_BARS 16u
#define PHRASE_GAP_MS 2000u                     /* a gap longer than this starts a fresh phrase */

typedef struct {
    uint16_t ms;                                /* time from clip start, ms */
    uint8_t  status;                            /* MIDI status incl. channel (0x90/0x80/0xB0/0xE0) */
    uint8_t  d1, d2;
} ev_t;

typedef struct {
    uint16_t len_ms;                            /* loop length; 0 = empty slot */
    uint16_t n;
    ev_t     ev[CLIP_EVENTS];
} clip_t;

typedef struct {                                /* a history entry: absolute ms, raw MIDI */
    uint32_t t_ms;
    uint8_t  status, d1, d2, pad;
} hev_t;

/* big buffers in the zeroed pool */
static clip_t clips[NTRACK][NSLOT] __attribute__((section(".pool")));
static hev_t  hist[NTRACK][HIST_EVENTS] __attribute__((section(".pool")));

/* per-track transport + history */
enum { T_IDLE = 0, T_PLAY, T_OVERDUB };
static struct {
    uint8_t  state;
    int8_t   active;        /* the slot being played/overdubbed, -1 = none */
    uint32_t origin_ms;     /* fm1_ms at which the current loop iteration began */
    uint16_t play_pos;      /* next ms position to emit this iteration */
    uint32_t h_head;        /* history ring write index (monotone; & (HIST_EVENTS-1) to index) */
    uint32_t last_ev_ms;    /* fm1_ms of the last logged event (for phrase-gap detection) */
    uint8_t  sounding[128]; /* notes this track's playback currently holds on */
} tr[NTRACK];

static uint8_t  cur_track;                      /* 0..7, chosen by SELECT */
static uint32_t proj_bpm;                       /* 0 until the first capture sets it */
static uint32_t proj_bar_ms;                    /* one 4/4 bar in ms at proj_bpm */
static uint32_t last_cap_bars;                  /* last capture's bar count (shown on screen) */

/* ---- history ring (always on) */
static void hist_add(uint32_t t, uint8_t st, uint8_t d1, uint8_t d2)
{
    uint32_t i = tr[t].h_head & (HIST_EVENTS - 1u);
    hist[t][i].t_ms = fm1_ms;
    hist[t][i].status = st;
    hist[t][i].d1 = d1;
    hist[t][i].d2 = d2;
    tr[t].h_head++;
    tr[t].last_ev_ms = fm1_ms;
}

/* ---- clip helpers */
static void clip_add(clip_t *c, uint32_t ms, uint8_t st, uint8_t d1, uint8_t d2)
{
    if (c->n >= CLIP_EVENTS) return;
    if (ms > 0xFFFFu) ms = 0xFFFFu;
    c->ev[c->n].ms = (uint16_t)ms;
    c->ev[c->n].status = st;
    c->ev[c->n].d1 = d1;
    c->ev[c->n].d2 = d2;
    c->n++;
}

static void track_hush(uint32_t t, uint32_t out_ch)
{
    uint32_t i;
    for (i = 0; i < 128u; i++)
        if (tr[t].sounding[i]) {
            stop(i);
            midi_send(0x80u | out_ch, (uint8_t)i, 0);
            tr[t].sounding[i] = 0;
        }
}

static void clip_emit(uint32_t t, uint32_t out_ch, const ev_t *e)
{
    uint32_t ty = e->status & 0xF0u, note = e->d1 & 0x7Fu;
    if (ty == 0x90u && e->d2) {                         /* note-on: only if not already sounding */
        if (!tr[t].sounding[note]) {
            midi_send((uint8_t)(0x90u | out_ch), e->d1, e->d2);
            play(e->d1, e->d2);
            tr[t].sounding[note] = 1;
        }
    } else if (ty == 0x80u || ty == 0x90u) {            /* note-off: only if sounding */
        if (tr[t].sounding[note]) {
            midi_send((uint8_t)(0x80u | out_ch), e->d1, 0);
            stop(e->d1);
            tr[t].sounding[note] = 0;
        }
    } else {                                            /* CC / pitch-bend: pass out only */
        midi_send((uint8_t)(ty | out_ch), e->d1, e->d2);
    }
}

static void player_window(uint32_t t, uint32_t out_ch, uint32_t from, uint32_t to)
{
    clip_t *c = &clips[t][tr[t].active];
    uint32_t i;
    for (i = 0; i < c->n; i++)
        if (c->ev[i].ms >= from && c->ev[i].ms <= to)
            clip_emit(t, out_ch, &c->ev[i]);
}

static void player_tick(void)
{
    uint32_t t;
    for (t = 0; t < NTRACK; t++) {
        uint32_t out_ch;
        clip_t *c;
        if (tr[t].state != T_PLAY && tr[t].state != T_OVERDUB) continue;
        if (tr[t].active < 0) continue;
        c = &clips[t][tr[t].active];
        if (!c->len_ms) continue;
        out_ch = out_channel();
        {
            uint32_t elapsed = fm1_ms - tr[t].origin_ms;
            uint32_t pos = elapsed % c->len_ms;
            uint32_t from = tr[t].play_pos;
            if (pos < from) {                    /* wrapped: finish the tail, silence, then the new head */
                player_window(t, out_ch, from, (uint32_t)c->len_ms - 1u);
                track_hush(t, out_ch);           /* no held note bleeds into the next iteration */
                from = 0;
            }
            player_window(t, out_ch, from, pos);
            tr[t].play_pos = (uint16_t)(pos + 1u);
        }
    }
}

/* ---- tempo/length detection on the recent history (integer only) --------------------
 * Returns 1 on success, filling *out_bars and (if the project BPM is not yet set) *out_bpm.
 * start_ms/end_ms frame the captured region (absolute ms). */
static uint32_t insert_sort_u32(uint32_t *a, uint32_t n)   /* small n; returns n */
{
    uint32_t i, j;
    for (i = 1; i < n; i++) {
        uint32_t v = a[i];
        for (j = i; j > 0 && a[j - 1] > v; j--) a[j] = a[j - 1];
        a[j] = v;
    }
    return n;
}

/* gather note-on onset times (absolute ms) from a track's history within [start,end] */
static uint32_t gather_onsets(uint32_t t, uint32_t start_ms, uint32_t end_ms, uint32_t *onset, uint32_t cap)
{
    uint32_t count = 0, scanned = tr[t].h_head < HIST_EVENTS ? tr[t].h_head : HIST_EVENTS;
    uint32_t base = tr[t].h_head - scanned, k;
    for (k = 0; k < scanned && count < cap; k++) {
        hev_t *h = &hist[t][(base + k) & (HIST_EVENTS - 1u)];
        if ((h->status & 0xF0u) == 0x90u && h->d2 &&
            h->t_ms >= start_ms && h->t_ms <= end_ms)
            onset[count++] = h->t_ms;
    }
    return count;
}

/* detect bars (and bpm if unknown) for a captured span. */
static uint32_t detect_fit(uint32_t t, uint32_t start_ms, uint32_t end_ms,
                           uint32_t *out_bpm, uint32_t *out_bars)
{
    uint32_t onset[64];
    uint32_t m = gather_onsets(t, start_ms, end_ms, onset, 64u);
    uint32_t span = end_ms - start_ms;
    uint32_t bar_ms, bars, bpm;
    if (span < 1u) return 0;

    if (proj_bpm) {                              /* grid already known: just snap to whole bars */
        bar_ms = proj_bar_ms;
        bars = (span + bar_ms / 2u) / bar_ms;
        if (bars < 1u) bars = 1u;
        if (bars > MAX_BARS) bars = MAX_BARS;
        *out_bpm = proj_bpm;
        *out_bars = bars;
        return 1;
    }

    /* first capture: derive the tempo. Use the median inter-onset gap as the beat unit. */
    if (m >= 3u) {
        uint32_t gaps[63], g = 0, i, beat_ms;
        insert_sort_u32(onset, m);               /* onsets may be out of order across the ring wrap */
        for (i = 1; i < m; i++)
            if (onset[i] > onset[i - 1]) gaps[g++] = onset[i] - onset[i - 1];
        if (!g) return 0;
        insert_sort_u32(gaps, g);
        beat_ms = gaps[g / 2u];                  /* median gap ~ one beat (or a subdivision) */
        if (!beat_ms) return 0;
        /* fold the beat into the 80..160 BPM range: a quarter = 60000/BPM ms => 375..750 ms */
        while (beat_ms > 60000u / BPM_MIN) beat_ms /= 2u;   /* too slow: halve */
        while (beat_ms < 60000u / BPM_MAX) beat_ms *= 2u;   /* too fast: double */
        if (!beat_ms) beat_ms = 60000u / 120u;
        bpm = (60000u + beat_ms / 2u) / beat_ms;
    } else {
        bpm = 120u;                              /* too few notes: fall back */
    }

    bar_ms = 4u * 60000u / bpm;                  /* 4/4 */
    if (!bar_ms) return 0;

    /* Weighted snap (first clip only): prefer common lengths so the project grid is sane and other
     * clips don't phase by accident. Powers of two first, then the musical 3/6/12 (waltz, blues),
     * then any whole number. Each candidate gets a tolerance bonus by its priority: a span near a
     * "nice" length is pulled to it, but a clearly-3-bar phrase still resolves to 3. */
    {
        static const uint8_t CAND[] = {1, 2, 4, 8, 16, 3, 6, 12};
        uint32_t best = 0, best_score = 0xFFFFFFFFu, ci;
        for (ci = 0; ci < sizeof CAND; ci++) {
            uint32_t cand = CAND[ci];
            uint32_t want = cand * bar_ms;        /* ideal span for this many bars */
            uint32_t diff = span > want ? span - want : want - span;
            uint32_t score = diff + ci * (bar_ms / 8u);   /* +cost by priority (earlier = cheaper) */
            if (score < best_score) { best_score = score; best = cand; }
        }
        bars = best ? best : 1u;
    }
    if (bars > MAX_BARS) bars = MAX_BARS;

    /* back-solve: make the loop exactly `bars` bars long so it is seamless. Keep the resulting tempo
     * in the 80..160 range the same way Ableton does: a 2-bar loop at 67 BPM is a 4-bar loop at 134
     * for the same audio, so double the bars (halve the implied tempo) until the BPM lands in range. */
    bar_ms = span / bars;
    if (bar_ms < 1u) bar_ms = 1u;
    bpm = 4u * 60000u / bar_ms;
    while (bpm && bpm < BPM_MIN && bars * 2u <= MAX_BARS) { bars *= 2u; bar_ms = span / bars; bpm = bar_ms ? 4u * 60000u / bar_ms : 0u; }
    while (bpm > BPM_MAX && bars > 1u && (bars & 1u) == 0u) { bars /= 2u; bar_ms = span / bars; bpm = bar_ms ? 4u * 60000u / bar_ms : 0u; }
    *out_bpm = bpm;
    *out_bars = bars;
    return 1;
}

/* build a clip from a track's recent history, [start,end] -> slot, rebased to start, length = len_ms */
static void build_clip(uint32_t t, uint32_t slot, uint32_t start_ms, uint32_t end_ms, uint32_t len_ms)
{
    clip_t *c = &clips[t][slot];
    uint32_t scanned = tr[t].h_head < HIST_EVENTS ? tr[t].h_head : HIST_EVENTS;
    uint32_t base = tr[t].h_head - scanned, k;
    c->n = 0;
    for (k = 0; k < scanned; k++) {
        hev_t *h = &hist[t][(base + k) & (HIST_EVENTS - 1u)];
        uint32_t ty = h->status & 0xF0u, rel;
        if (h->t_ms < start_ms || h->t_ms > end_ms) continue;
        if (ty != 0x90u && ty != 0x80u && ty != 0xB0u && ty != 0xE0u) continue;
        rel = h->t_ms - start_ms;               /* clamp into the loop (never wrap: keeps on/off order) */
        if (len_ms && rel >= len_ms) rel = len_ms - 1u;
        clip_add(c, rel, h->status, h->d1, h->d2);
    }
    c->len_ms = (uint16_t)(len_ms > 0xFFFFu ? 0xFFFFu : len_ms < 1u ? 1u : len_ms);
}

/* find where the recent phrase started: walk history back from the press until a gap > PHRASE_GAP_MS */
static uint32_t phrase_start_ms(uint32_t t, uint32_t end_ms)
{
    uint32_t scanned = tr[t].h_head < HIST_EVENTS ? tr[t].h_head : HIST_EVENTS;
    uint32_t base = tr[t].h_head - scanned, k, prev = 0, start = end_ms, have = 0;
    for (k = 0; k < scanned; k++) {
        hev_t *h = &hist[t][(base + k) & (HIST_EVENTS - 1u)];
        if ((h->status & 0xF0u) != 0x90u || !h->d2) continue;
        if (h->t_ms > end_ms) continue;
        if (have && h->t_ms - prev > PHRASE_GAP_MS) start = h->t_ms;   /* a gap: phrase restarts here */
        else if (!have) start = h->t_ms;
        prev = h->t_ms;
        have = 1;
    }
    return have ? start : end_ms;
}

/* ---- the screen */
#define C_ACC RGB(180, 140, 255)                /* lilac */
#define C_OFF RGB(90, 90, 90)
#define C_REC RGB(255, 70, 70)
#define C_PLY RGB(80, 230, 120)
#define KEY0 53u                                /* MIDI note of key 0 (F3) */

static const char *state_name(uint32_t s)
{
    return s == T_OVERDUB ? "DUB" : s == T_PLAY ? "PLAY" : "LISTEN";
}

static void draw_track(void)                    /* big: active track number + its state */
{
    char s[14] = "TRK 1 ";
    const char *p;
    uint32_t i = 6;
    s[4] = (char)('1' + cur_track);
    p = state_name(tr[cur_track].state);
    while (*p) s[i++] = *p++;
    s[i] = 0;
    draw_text_box(0, 82, 240, &FONT_L, s, C_WHITE, 1);
}

static void draw_slots(void)
{
    uint32_t i;
    cv_begin(240, 40, C_BLACK);
    for (i = 0; i < NSLOT; i++) {
        uint32_t x = 8 + i * 29;
        uint32_t filled = clips[cur_track][i].len_ms != 0u;
        uint32_t playing = (tr[cur_track].state == T_PLAY || tr[cur_track].state == T_OVERDUB)
                           && tr[cur_track].active == (int8_t)i;
        uint32_t col = tr[cur_track].state == T_OVERDUB && playing ? C_REC
                     : playing ? C_PLY : filled ? C_ACC : C_OFF;
        cv_rect((int32_t)x, 8, 24, 24, col);
    }
    cv_blit(0, 140);
}

static void draw_status(int32_t oct, uint32_t ch)
{
    char s[24] = "OCT +0  MIDI OMNI";
    s[4] = oct < 0 ? '-' : '+';
    s[5] = (char)('0' + (oct < 0 ? -oct : oct));
    if (ch) {
        s[13] = 'C'; s[14] = 'H'; s[15] = (char)(ch < 10u ? ' ' : '1'); s[16] = (char)('0' + ch % 10u);
    }
    draw_text_box(0, 212, 240, &FONT_S, s, C_OFF, 1);
}

/* shows the project BPM and the last capture's bar count */
static void draw_info(void)
{
    char s[32], *p = s;
    const char *a;
    uint32_t bpm = proj_bpm;
    if (bpm) {
        a = "BPM ";
        while (*a) *p++ = *a++;
        if (bpm >= 100u) *p++ = (char)('0' + bpm / 100u % 10u);
        *p++ = (char)('0' + bpm / 10u % 10u);
        *p++ = (char)('0' + bpm % 10u);
        if (last_cap_bars) {
            *p++ = ' '; *p++ = ' ';
            if (last_cap_bars >= 10u) *p++ = (char)('0' + last_cap_bars / 10u);
            *p++ = (char)('0' + last_cap_bars % 10u);
            a = " BAR";
            while (*a) *p++ = *a++;
        }
    } else {
        a = "PLAY THEN REC";
        while (*a) *p++ = *a++;
    }
    *p = 0;
    draw_text_box(0, 120, 240, &FONT_S, s, C_OFF, 1);
}

/* ---- MIDI in: from the main loop, and from app_idle while the screen draws (idle_ok) */
static uint32_t idle_ok, midi_src, clk_n, clk_t0, clk_bpm, clk_run, clk_beat_ms;

static void midi_in(void)
{
    uint8_t st, d1, d2;
    int src;
    while ((src = midi_get(&st, &d1, &d2)) != 0) {   /* 1 USB, 2 the TRS jack */
        uint32_t ty = st & 0xF0u;
        if (st == 0xF8u) {
            if (clk_n % 24u == 0u) {
                if (clk_n) {
                    uint32_t ms = fm1_ms - clk_t0;
                    clk_bpm = ms ? (60000u + ms / 2u) / ms : 0;
                }
                clk_t0 = fm1_ms;
                clk_beat_ms = fm1_ms;
            }
            clk_n++;
            continue;
        }
        if (st == 0xFAu) { clk_run = 1; clk_n = 0; continue; }
        if (st == 0xFBu) { clk_run = 1; continue; }
        if (st == 0xFCu) { clk_run = 0; continue; }
        if (set.ch && (st & 15u) != set.ch - 1u) continue;      /* not our channel */
        if (ty == 0x90u && d2) { play(d1, d2); midi_src = (uint32_t)src; }
        else if (ty == 0x80u || ty == 0x90u) stop(d1);
        /* NOTE: incoming MIDI is NOT echoed back out (that could loop back in the emulator and
         * double every note -> the "weird sine"). It still sounds via play() and is logged. */
        if (ty == 0x90u || ty == 0x80u || ty == 0xB0u || ty == 0xE0u)
            hist_add(cur_track, st, d1, d2);                    /* always logging */
    }
}
static void app_idle(void)
{
    if (idle_ok) midi_in();
}

/* ---- transport actions ---------------------------------------------------------------- */
static void clip_launch(uint32_t t, uint32_t slot)
{
    tr[t].active = (int8_t)slot;
    tr[t].origin_ms = fm1_ms;
    tr[t].play_pos = 0;
    tr[t].state = T_PLAY;
}

/* REC: retrospectively capture the recent phrase into the selected (or first empty) slot */
static void rec_press(void)
{
    uint32_t t = cur_track, end = fm1_ms, start, bpm = 0, bars = 0, len;
    int slot = tr[t].active;

    if ((tr[t].state == T_PLAY || tr[t].state == T_OVERDUB) && slot >= 0) {
        tr[t].state = tr[t].state == T_PLAY ? T_OVERDUB : T_PLAY;   /* toggle overdub */
        return;
    }
    if (slot < 0) {                              /* no slot chosen: use the first empty one */
        uint32_t i;
        slot = 0;
        for (i = 0; i < NSLOT; i++) if (!clips[t][i].len_ms) { slot = (int)i; break; }
    }
    start = phrase_start_ms(t, end);
    if (!detect_fit(t, start, end, &bpm, &bars)) return;          /* nothing usable */

    if (!proj_bpm) {                             /* first capture sets the project grid */
        proj_bpm = bpm;
        proj_bar_ms = 4u * 60000u / (bpm ? bpm : 120u);
    }
    len = bars * proj_bar_ms;                    /* exact whole bars at the grid */
    build_clip(t, (uint32_t)slot, start, end, len);
    last_cap_bars = bars;
    track_hush(t, out_channel());
    clip_launch(t, (uint32_t)slot);
}

static void slot_press(uint32_t slot)
{
    uint32_t t = cur_track;
    if (clips[t][slot].len_ms) {                 /* filled */
        if ((tr[t].state == T_PLAY || tr[t].state == T_OVERDUB) && tr[t].active == (int8_t)slot) {
            track_hush(t, out_channel());        /* pressing the playing slot again: stop */
            tr[t].state = T_IDLE;
        } else {
            track_hush(t, out_channel());
            clip_launch(t, slot);
        }
    } else {
        tr[t].active = (int8_t)slot;             /* empty: just select it as the capture target */
        if (tr[t].state != T_PLAY && tr[t].state != T_OVERDUB) tr[t].state = T_IDLE;
    }
}

/* PLAY is the master transport: if anything is playing, stop every track; otherwise (re)launch every
 * track that has an active clip, all from the same origin so they stay in sync. */
static void play_press(void)
{
    uint32_t t, any = 0;
    for (t = 0; t < NTRACK; t++)
        if (tr[t].state == T_PLAY || tr[t].state == T_OVERDUB) { any = 1; break; }
    if (any) {                                   /* stop all */
        for (t = 0; t < NTRACK; t++) {
            track_hush(t, out_channel());
            tr[t].state = T_IDLE;
        }
    } else {                                     /* start all that have a clip, in sync */
        uint32_t now = fm1_ms;
        for (t = 0; t < NTRACK; t++)
            if (tr[t].active >= 0 && clips[t][tr[t].active].len_ms) {
                tr[t].origin_ms = now;
                tr[t].play_pos = 0;
                tr[t].state = T_PLAY;
            }
    }
}

static void app_main(void)
{
    uint32_t prev = 0, k, t_change = 0, dirty = 0;
    uint32_t track_drawn = ~0u, slots_drawn = ~0u, status_drawn = ~0u, info_drawn = ~0u;
    uint8_t held[27] = {0};
    uint32_t i;
#if BOARD_SMK37
    smk_pw_t wheel = {0};
    smk_pad_events = 1;
#endif
    for (i = 0; i < NTRACK; i++) tr[i].active = -1;

    if (settings_load(&set, sizeof set) || (int8_t)set.oct < -2 || (int8_t)set.oct > 2 || set.ch > 16u)
        set.oct = 0, set.ch = 0;

    lcd_fill(0, 0, 240, 240, C_BLACK);
    draw_text_box(0, 18, 240, &FONT_L, APP_NAME, C_ACC, 1);
    draw_text_box(0, 52, 240, &FONT_S, "retrospective looper  v" APP_VER, C_OFF, 1);

    for (;;) {
        uint32_t keys, down, up, rel, b, out_ch = out_channel(), st;
        int32_t oct = (int8_t)set.oct, e;
        uint32_t base = (uint32_t)((int32_t)KEY0 + 12 * oct);
        platform_poll();

        keys = fm1_in.notes;
        down = keys & ~prev;
        up = prev & ~keys;
        prev = keys;
        for (k = 0; k < 27u; k++) {
#if BOARD_SMK37
            uint32_t vel = fm1_in.key_vel[SMK_KEY_F3 + k];
#else
            uint32_t vel = 100;
#endif
            if (k >= PAD0) {                        /* a clip-slot pad (top row) */
                if (down >> k & 1u) slot_press(k - PAD0);
                continue;
            }
            if (down >> k & 1u) {                   /* a note key: play, echo, and log to history */
                held[k] = (uint8_t)(base + k);
                play(held[k], vel);
                midi_send(0x90u | out_ch, held[k], vel);
                hist_add(cur_track, (uint8_t)(0x90u | out_ch), held[k], (uint8_t)vel);
            }
            if (up >> k & 1u) {
                stop(held[k]);
                midi_send(0x80u | out_ch, held[k], 0);
                hist_add(cur_track, (uint8_t)(0x80u | out_ch), held[k], 0);
            }
        }
#if BOARD_SMK37
        {
            int32_t bd = smk_pw_bend(&wheel, fm1_in.wheel[0], fm1_ms);
            bend_q16 = (uint32_t)(65536 + (bd * 3785 >> 12) + (((bd * bd) >> 12) * 109 >> 12));
        }
#endif

        b = fm1_input_edges(&rel);
        if (b & (1u << BTN_OCT_DOWN) && oct > -2) { set.oct = (uint8_t)(oct - 1); dirty = 1; t_change = fm1_ms; }
        if (b & (1u << BTN_OCT_UP) && oct < 2) { set.oct = (uint8_t)(oct + 1); dirty = 1; t_change = fm1_ms; }
        if (b & (1u << BTN_REC)) rec_press();
        if (b & (1u << BTN_PLAY)) play_press();
        if ((e = fm1_enc_take(ENC_SELECT)) != 0) {      /* SELECT: the active track 1..8 */
            int32_t c = (int32_t)cur_track + e;
            while (c < 0) c += NTRACK;
            cur_track = (uint8_t)((uint32_t)c % NTRACK);
        }
        if ((e = fm1_enc_take(ENC_PRESETS)) != 0) {     /* PRESETS: the MIDI channel */
            int32_t c = (int32_t)set.ch + e;
            set.ch = (uint8_t)(c < 0 ? 0 : c > 16 ? 16 : c);
            dirty = 1;
            t_change = fm1_ms;
        }

        midi_in();
        player_tick();
        if (clk_bpm && fm1_ms - clk_t0 > 2000u) clk_bpm = 0, clk_run = 0;

        /* LEDs: clip pads show state, note keys show what sounds */
        {
            uint32_t vlit = 0, v;
            for (v = 0; v < NV; v++)
                if (voice[v].gate && voice[v].note >= base && voice[v].note < base + 27u)
                    vlit |= 1u << (voice[v].note - base);
            for (k = 0; k < PAD0; k++)
                fm1_led_key(KEY_LED(k), vlit >> k & 1u);
            for (k = 0; k < NSLOT; k++) {
                uint32_t playing = (tr[cur_track].state == T_PLAY || tr[cur_track].state == T_OVERDUB)
                                   && tr[cur_track].active == (int8_t)k;
                uint32_t filled = clips[cur_track][k].len_ms != 0u;
                fm1_led_key(KEY_LED(PAD0 + k), playing);
                if (!playing && filled) fm1_led_key_glow(KEY_LED(PAD0 + k), 1);
            }
        }
        fm1_led_key(BTN_OCT_DOWN, oct < 0);
        fm1_led_key(BTN_OCT_UP, oct > 0);
        fm1_led_key(BTN_PLAY, tr[cur_track].state == T_PLAY || tr[cur_track].state == T_OVERDUB);
        fm1_led_key(BTN_REC, tr[cur_track].state == T_OVERDUB);

        idle_ok = 1;
        if ((st = (uint32_t)cur_track << 4 | tr[cur_track].state) != track_drawn) { track_drawn = st; draw_track(); }
        {
            uint32_t sig = cur_track << 24 | (uint32_t)tr[cur_track].state << 20 | (uint32_t)(tr[cur_track].active + 1) << 16;
            for (k = 0; k < NSLOT; k++) sig ^= (clips[cur_track][k].len_ms ? 1u : 0u) << k;
            if (sig != slots_drawn) { slots_drawn = sig; draw_slots(); }
        }
        if ((st = (uint32_t)(oct + 2) | (uint32_t)set.ch << 4) != status_drawn) { status_drawn = st; draw_status(oct, set.ch); }
        if ((st = proj_bpm | last_cap_bars << 16) != info_drawn) { info_drawn = st; draw_info(); }
        idle_ok = 0;

        if (dirty && quiet() && fm1_ms - t_change > 2000u) {
            settings_save(&set, sizeof set);
            dirty = 0;
        }
    }
}
