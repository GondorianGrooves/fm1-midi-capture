/* SPDX-License-Identifier: GPL-3.0-only */
/* fm1-midi-capture: a retrospective MIDI looper / clip sequencer for the FM-1, built on Dinghy.
 *
 * Phase 1 (this build): one track at a time (SELECT picks 1..8), 8 clip slots on the top row of keys,
 * fixed-length loop capture with overdub, playback through the internal sine monitor and echoed to
 * USB-MIDI out. The bottom keys still play notes for testing.
 *
 *   SELECT     the active track (1..8)
 *   top 8 keys (keys 19..26)   the 8 clip slots of the active track:
 *                - empty slot  -> arm it (capture starts on the first note, or on REC)
 *                - filled slot -> stop whatever plays on this track, start this clip, arm it
 *   bottom keys (0..18)        play the internal sine, recorded as MIDI input
 *   REC        empty/armed: start capture; recording: close the loop; playing: overdub on/off
 *   PLAY       start/stop the active track's clip
 *   OCT- OCT+  the octave (kept in flash)
 *   PRESETS    the MIDI channel: OMNI (in: all, out: 1) or 1..16 (kept in flash)
 *   MASTER     the volume (platform.c)
 *   MIDI in    notes from USB or the TRS jack play the sines and are captured; a clock shows its tempo
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
    for (v = 0; v < NV; v++) {                  /* the same note, else a free voice, else the oldest */
        uint32_t score = voice[v].note == note && voice[v].gate ? 0u
                       : voice[v].gate ? 0x80000000u + voice[v].age : (uint32_t)voice[v].amp + 1u;
        if (score < best) { best = score; pick = v; }
    }
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
 * LOOPER
 *
 * One track is "active" (SELECT picks it, 1..8). Each track has 8 clip slots; in Phase 1 only the
 * active track's clips live in RAM (NTRACK * NSLOT clips in the pool), and one clip per track may be
 * playing. A clip is a list of timestamped MIDI events; timestamps are ms from the clip's start, and
 * the clip loops every clip.len_ms. Capture is fixed-length: it runs from the first recorded event
 * until the loop is closed (REC again, or the armed pad pressed again).
 * ==================================================================================== */
#define NTRACK 8u
#define NSLOT  8u
#define CLIP_EVENTS 384u                        /* events per clip (tunable). 384*4 = 1536 B each */
#define PAD0 19u                                /* key 19..26 = the 8 clip slots (top row); keys 0..18 play notes */

typedef struct {
    uint16_t ms;                                /* time from clip start, ms (loops < 64 s) */
    uint8_t  status;                            /* MIDI status incl. channel (0x90/0x80/0xB0/0xE0) */
    uint8_t  d1, d2;                            /* data bytes */
} ev_t;

typedef struct {
    uint16_t len_ms;                            /* loop length; 0 = empty slot */
    uint16_t n;                                 /* number of events */
    ev_t     ev[CLIP_EVENTS];
} clip_t;

/* clips live in the big zeroed pool (NTRACK*NSLOT*~1.5 KB = ~98 KB) */
static clip_t clips[NTRACK][NSLOT] __attribute__((section(".pool")));

/* per-track transport */
enum { T_IDLE = 0, T_ARMED, T_REC, T_PLAY, T_OVERDUB };
static struct {
    uint8_t  state;
    int8_t   active;        /* the slot being played/recorded, -1 = none */
    uint32_t origin_ms;     /* fm1_ms at which the current loop iteration (or capture) began */
    uint16_t play_pos;      /* last playback position emitted this iteration (ms) */
    uint8_t  sounding[128]; /* notes this track's playback currently holds on (for clean stop) */
} tr[NTRACK];

static uint8_t cur_track;   /* 0..7, chosen by SELECT */

/* record one event into a clip (used by live capture and overdub) */
static void clip_add(clip_t *c, uint32_t ms, uint8_t st, uint8_t d1, uint8_t d2)
{
    if (c->n >= CLIP_EVENTS) return;            /* clip full: drop (Phase 1 keeps it simple) */
    if (ms > 0xFFFFu) ms = 0xFFFFu;
    c->ev[c->n].ms = (uint16_t)ms;
    c->ev[c->n].status = st;
    c->ev[c->n].d1 = d1;
    c->ev[c->n].d2 = d2;
    c->n++;
}

/* silence every note a track is holding from playback */
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

/* emit one recorded event: out to MIDI and into the internal monitor */
static void clip_emit(uint32_t t, uint32_t out_ch, const ev_t *e)
{
    uint32_t ty = e->status & 0xF0u;
    midi_send((uint8_t)(ty | out_ch), e->d1, e->d2);   /* external out on the track's channel */
    if (ty == 0x90u && e->d2) { play(e->d1, e->d2); tr[t].sounding[e->d1 & 0x7Fu] = 1; }
    else if (ty == 0x80u || ty == 0x90u) { stop(e->d1); tr[t].sounding[e->d1 & 0x7Fu] = 0; }
    /* CC / pitch-bend (0xB0/0xE0) are sent out; the monitor ignores them for now (Phase 4) */
}

/* emit every event with from <= ms <= to (inclusive both ends) */
static void player_window(uint32_t t, uint32_t out_ch, uint32_t from, uint32_t to)
{
    clip_t *c = &clips[t][tr[t].active];
    uint32_t i;
    for (i = 0; i < c->n; i++)
        if (c->ev[i].ms >= from && c->ev[i].ms <= to)
            clip_emit(t, out_ch, &c->ev[i]);
}

/* called every main-loop pass: drive playback / overdub timing for every track.
 * play_pos holds the next ms position still to emit (0 at the start of a loop iteration). */
static void player_tick(void)
{
    uint32_t t;
    for (t = 0; t < NTRACK; t++) {
        uint32_t out_ch, st = tr[t].state;
        clip_t *c;
        if (st != T_PLAY && st != T_OVERDUB) continue;
        c = &clips[t][tr[t].active];
        if (!c->len_ms) continue;
        out_ch = set.ch ? set.ch - 1u : 0u;     /* Phase 1: one channel; per-track in Phase 3 */
        {
            uint32_t elapsed = fm1_ms - tr[t].origin_ms;
            uint32_t pos = elapsed % c->len_ms;  /* 0 .. len_ms-1 */
            uint32_t from = tr[t].play_pos;
            if (pos < from) {                    /* wrapped: finish the tail, then the new head */
                player_window(t, out_ch, from, (uint32_t)c->len_ms - 1u);
                from = 0;
            }
            player_window(t, out_ch, from, pos);
            tr[t].play_pos = (uint16_t)(pos + 1u);
        }
    }
}

/* ---- the screen */
#define C_ACC RGB(180, 140, 255)                /* lilac */
#define C_KEY RGB(210, 210, 210)
#define C_OFF RGB(90, 90, 90)
#define C_REC RGB(255, 70, 70)
#define C_PLY RGB(80, 230, 120)
#define C_ARM RGB(255, 190, 60)
#define KEY0 53u                                /* MIDI note of key 0 (F3) */

static const char *state_name(uint32_t s)
{
    return s == T_REC ? "REC" : s == T_OVERDUB ? "DUB" : s == T_PLAY ? "PLAY"
         : s == T_ARMED ? "ARM" : "IDLE";
}

static void draw_track(void)                    /* big: active track number + its state */
{
    char s[12] = "TRK 1 ";
    const char *p;
    uint32_t i = 6;
    s[4] = (char)('1' + cur_track);
    p = state_name(tr[cur_track].state);
    while (*p) s[i++] = *p++;
    s[i] = 0;
    draw_text_box(0, 82, 240, &FONT_L, s, C_WHITE, 1);
}

static void draw_slots(void)                    /* the 8 clip slots of the active track */
{
    uint32_t i;
    cv_begin(240, 40, C_BLACK);
    for (i = 0; i < NSLOT; i++) {
        uint32_t x = 8 + i * 29;
        uint32_t filled = clips[cur_track][i].len_ms != 0u;
        uint32_t playing = (tr[cur_track].state == T_PLAY || tr[cur_track].state == T_OVERDUB)
                           && tr[cur_track].active == (int8_t)i;
        uint32_t recing = (tr[cur_track].state == T_REC || tr[cur_track].state == T_ARMED)
                          && tr[cur_track].active == (int8_t)i;
        uint32_t col = recing ? (tr[cur_track].state == T_ARMED ? C_ARM : C_REC)
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

static void draw_midi(uint32_t src, uint32_t bpm, uint32_t running)
{
    char s[32] = "", *p = s;
    const char *a = src == 2u ? "TRS IN" : src == 1u ? "USB IN" : "";
    while (*a) *p++ = *a++;
    if (bpm) {
        if (p != s) { *p++ = ' '; *p++ = ' '; }
        a = running ? "CLOCK " : "clock ";
        while (*a) *p++ = *a++;
        if (bpm >= 100u) *p++ = (char)('0' + bpm / 100u);
        *p++ = (char)('0' + bpm / 10u % 10u);
        *p++ = (char)('0' + bpm % 10u);
        a = " BPM";
        while (*a) *p++ = *a++;
    }
    *p = 0;
    draw_text_box(0, 120, 240, &FONT_S, s, C_OFF, 1);
}

/* ---- MIDI in: from the main loop, and from app_idle while the screen draws (idle_ok) */
static uint32_t idle_ok, midi_src, clk_n, clk_t0, clk_bpm, clk_run, clk_beat_ms;

/* capture an incoming event into whatever track is recording/overdubbing */
static void capture_event(uint8_t st, uint8_t d1, uint8_t d2)
{
    uint32_t t = cur_track, state = tr[t].state;
    clip_t *c;
    uint32_t ms;
    if (state == T_ARMED) {                      /* first event starts the clock */
        tr[t].state = T_REC;
        tr[t].origin_ms = fm1_ms;
        state = T_REC;
    }
    if (state != T_REC && state != T_OVERDUB) return;
    c = &clips[t][tr[t].active];
    ms = fm1_ms - tr[t].origin_ms;
    if (state == T_OVERDUB && c->len_ms)         /* overdub: wrap into the loop so it replays */
        ms %= c->len_ms;
    clip_add(c, ms, st, d1, d2);
}

static void midi_in(void)
{
    uint8_t st, d1, d2;
    int src;
    while ((src = midi_get(&st, &d1, &d2)) != 0) {   /* 1 USB, 2 the TRS jack */
        uint32_t ty = st & 0xF0u;
        if (st == 0xF8u) {                      /* clock: 24 a quarter note */
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
        if (ty == 0x90u || ty == 0x80u || ty == 0xB0u || ty == 0xE0u) {
            uint32_t out_ch = set.ch ? set.ch - 1u : 0u;
            midi_send((uint8_t)(ty | out_ch), d1, d2);          /* echo / thru */
            capture_event(st, d1, d2);                          /* record if armed/recording */
        }
    }
}
static void app_idle(void)
{
    if (idle_ok) midi_in();
}

/* ---- looper transport actions (from the panel) */
static void clip_start(uint32_t t, uint32_t slot)   /* begin playing a filled slot */
{
    tr[t].active = (int8_t)slot;
    tr[t].origin_ms = fm1_ms;
    tr[t].play_pos = 0;
    tr[t].state = T_PLAY;
}

static void slot_press(uint32_t slot)               /* a clip pad on the active track */
{
    uint32_t t = cur_track;
    track_hush(t, out_channel());                   /* stop whatever this track was holding */
    tr[t].play_pos = 0;
    if (clips[t][slot].len_ms) {                    /* filled: stop current, launch this one */
        clip_start(t, slot);
    } else {                                        /* empty: arm for capture */
        tr[t].active = (int8_t)slot;
        tr[t].state = T_ARMED;
    }
}

static void rec_press(void)
{
    uint32_t t = cur_track, state = tr[t].state;
    clip_t *c;
    if (tr[t].active < 0) {                          /* nothing selected: arm slot 0 */
        tr[t].active = 0;
        tr[t].state = T_ARMED;
        return;
    }
    c = &clips[t][tr[t].active];
    if (state == T_ARMED) {                          /* arm -> start empty capture now */
        tr[t].state = T_REC;
        tr[t].origin_ms = fm1_ms;
        c->n = 0;
    } else if (state == T_REC) {                     /* close the loop */
        uint32_t len = fm1_ms - tr[t].origin_ms;
        c->len_ms = (uint16_t)(len < 1u ? 1u : len > 0xFFFFu ? 0xFFFFu : len);
        clip_start(t, (uint32_t)tr[t].active);       /* straight into playback */
    } else if (state == T_PLAY) {                    /* play -> overdub */
        tr[t].state = T_OVERDUB;
    } else if (state == T_OVERDUB) {                 /* overdub -> play */
        tr[t].state = T_PLAY;
    }
}

static void play_press(void)
{
    uint32_t t = cur_track;
    if (tr[t].state == T_PLAY || tr[t].state == T_OVERDUB) {
        track_hush(t, out_channel());
        tr[t].state = T_IDLE;
    } else if (tr[t].active >= 0 && clips[t][tr[t].active].len_ms) {
        tr[t].play_pos = 0;
        clip_start(t, (uint32_t)tr[t].active);
    }
}

static void app_main(void)
{
    uint32_t prev = 0, k, t_change = 0, dirty = 0;
    uint32_t track_drawn = ~0u, slots_drawn = ~0u, status_drawn = ~0u, midi_drawn = ~0u;
    uint8_t held[27] = {0};
    uint32_t i;
#if BOARD_SMK37
    static const uint8_t WHITE[16] = {0, 2, 4, 6, 7, 9, 11, 12, 14, 16, 18, 19, 21, 23, 24, 26};
    uint8_t pad_note[16] = {0}, ext_note[FM1_NKEY] = {0};
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
            if (down >> k & 1u) {                   /* a note key: play, echo, and capture */
                held[k] = (uint8_t)(base + k);
                play(held[k], vel);
                midi_send(0x90u | out_ch, held[k], vel);
                capture_event((uint8_t)(0x90u | out_ch), held[k], (uint8_t)vel);
            }
            if (up >> k & 1u) {
                stop(held[k]);
                midi_send(0x80u | out_ch, held[k], 0);
                capture_event((uint8_t)(0x80u | out_ch), held[k], 0);
            }
        }
#if BOARD_SMK37
        {
            int32_t bd = smk_pw_bend(&wheel, fm1_in.wheel[0], fm1_ms);
            bend_q16 = (uint32_t)(65536 + (bd * 3785 >> 12) + (((bd * bd) >> 12) * 109 >> 12));
        }
        (void)pad_note; (void)ext_note;
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
        player_tick();                                  /* drive playback / overdub */
        if (clk_bpm && fm1_ms - clk_t0 > 2000u) clk_bpm = 0, clk_run = 0;

        /* LEDs: the 8 clip pads show state, the note keys show what sounds */
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
                uint32_t recing = (tr[cur_track].state == T_REC || tr[cur_track].state == T_ARMED)
                                  && tr[cur_track].active == (int8_t)k;
                uint32_t filled = clips[cur_track][k].len_ms != 0u;
                fm1_led_key(KEY_LED(PAD0 + k), playing || recing);
                if (!(playing || recing) && filled) fm1_led_key_glow(KEY_LED(PAD0 + k), 1);
            }
        }
        fm1_led_key(BTN_OCT_DOWN, oct < 0);
        fm1_led_key(BTN_OCT_UP, oct > 0);
        fm1_led_key(BTN_REC, tr[cur_track].state == T_REC || tr[cur_track].state == T_OVERDUB
                              || tr[cur_track].state == T_ARMED);
        fm1_led_key(BTN_PLAY, tr[cur_track].state == T_PLAY || tr[cur_track].state == T_OVERDUB);

        idle_ok = 1;
        if ((st = (uint32_t)cur_track << 4 | tr[cur_track].state) != track_drawn) { track_drawn = st; draw_track(); }
        {   /* slots view changes with selection, fill, and transport */
            uint32_t sig = cur_track << 24 | (uint32_t)tr[cur_track].state << 20 | (uint32_t)(tr[cur_track].active + 1) << 16;
            for (k = 0; k < NSLOT; k++) sig ^= (clips[cur_track][k].len_ms ? 1u : 0u) << k;
            if (sig != slots_drawn) { slots_drawn = sig; draw_slots(); }
        }
        if ((st = (uint32_t)(oct + 2) | (uint32_t)set.ch << 4) != status_drawn) { status_drawn = st; draw_status(oct, set.ch); }
        if ((st = midi_src | clk_bpm << 2 | clk_run << 12) != midi_drawn) { midi_drawn = st; draw_midi(midi_src, clk_bpm, clk_run); }
        idle_ok = 0;

        if (dirty && quiet() && fm1_ms - t_change > 2000u) {
            settings_save(&set, sizeof set);
            dirty = 0;
        }
    }
}
