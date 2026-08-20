/*
 * Copyright © 2014-2018  Rinat Ibragimov
 *
 * This file is part of "apulse" project.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "apulse.h"
#include "trace.h"
#include "util.h"
#include <dirent.h>
#include <errno.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>

#define MAKE_SND_LIB_VERSION(a, b, c) (((a) << 16) | ((b) << 8) | (c))

#define HAVE_SND_PCM_AVAIL SND_LIB_VERSION >= MAKE_SND_LIB_VERSION(1, 0, 18)

// ---------------------------------------------------------------------------
// Diagnostics, entirely gated behind APULSE_DIAG=1. Unset means every function
// below returns immediately and nothing is logged, so this changes no
// behaviour.
//
// Exists because WITH_TRACE=0 in the shipped build and snd_pcm_recover is
// called with silent=1, so short writes, xruns and inserted silence are all
// invisible. Diagnosing playback faults by ear cost several wrong hypotheses.
//
// The lifecycle lines matter most. Soloist applies a quality change on the
// next track, not mid-stream, so a format change should appear here as a
// disconnect/connect pair with a different sample spec. If the pair is absent,
// the stream was reused and s->ss is stale. If it is present but the second
// connect negotiated different ALSA parameters, the fault is in reconnect.
// ---------------------------------------------------------------------------

static int
diag_on(void)
{
    static int cached = -1;

    if (cached < 0) {
        const char *e = getenv("APULSE_DIAG");

        cached = (e && (e[0] == '1' || e[0] == '2')) ? 1 : 0;
    }
    return cached;
}

// APULSE_DIAG=2 removes the rate limits. Intended for a trace capture, where
// every timing read has to be visible next to the upstream trace line for the
// call that produced it. One in ten is enough to see the shape of a steady
// state; it is not enough to correlate two logs event by event.
static int
diag_full(void)
{
    static int cached = -1;

    if (cached < 0) {
        const char *e = getenv("APULSE_DIAG");

        cached = (e && e[0] == '2') ? 1 : 0;
    }
    return cached;
}

static void
diag_logf(const char *fmt, ...)
{
    va_list ap;

    if (!diag_on())
        return;
    fputs("apulse diag: ", stderr);
    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    va_end(ap);
    fputc('\n', stderr);
    fflush(stderr);
}

static const char *
diag_fmt_name(pa_sample_format_t f)
{
    const char *n = pa_sample_format_to_string(f);

    return n ? n : "?";
}

// Per-second write-loop counters. Reported once a second from the io callback,
// which is the only place they are touched, so no locking is needed.
static struct {
    struct timeval t0;
    unsigned wakeups;     // io callback entries with POLLOUT
    unsigned writes;      // snd_pcm_writei calls actually made
    unsigned short_w;     // writes that took fewer frames than offered
    unsigned err;         // writes that returned < 0
    unsigned xrun;        // recover() paths taken
    unsigned pads;        // wakeups that wrote silence
    long frames;          // frames accepted by ALSA
    long pad_frames;      // of which silence
    long client_bytes;    // bytes accepted from the client
} dstat;

static void
diag_reset(void)
{
    memset(&dstat, 0, sizeof(dstat));
    gettimeofday(&dstat.t0, NULL);
}

static void
diag_note_write(snd_pcm_sframes_t wr, size_t offered, int padded)
{
    if (!diag_on())
        return;
    dstat.writes++;
    if (wr < 0) {
        dstat.err++;
        return;
    }
    dstat.frames += (long)wr;
    if ((size_t)wr < offered)
        dstat.short_w++;
    if (padded) {
        dstat.pads++;
        dstat.pad_frames += (long)wr;
    }
}

// The tripwire. Soloist decodes lossy to 16-bit and lossless to float32, so a
// format change alters its byte rate into pa_stream_write while the frame rate
// into ALSA stays at the sample rate. Dividing one by the other gives the frame
// size the client is really using; if that disagrees with pa_frame_size(&s->ss)
// then the stream is being fed a format it was not opened with, and every
// index and latency calculation is out by that ratio.
static void
diag_tick(pa_stream *s)
{
    struct timeval now;
    long ms;
    size_t declared;
    long implied_x100 = 0;

    if (!diag_on())
        return;

    gettimeofday(&now, NULL);
    if (dstat.t0.tv_sec == 0)
        dstat.t0 = now;
    ms = (now.tv_sec - dstat.t0.tv_sec) * 1000 +
         (now.tv_usec - dstat.t0.tv_usec) / 1000;
    if (ms < 1000)
        return;

    declared = pa_frame_size(&s->ss);
    if (dstat.frames > 0)
        implied_x100 = dstat.client_bytes * 100 / dstat.frames;

    {
        snd_pcm_sframes_t delay = 0;
        snd_pcm_sframes_t avail = 0;
        size_t rb = s->rb ? ringbuffer_readable_size(s->rb) : 0;

        if (s->ph) {
            if (snd_pcm_delay(s->ph, &delay) < 0)
                delay = 0;
            avail = snd_pcm_avail(s->ph);
            if (avail < 0)
                avail = 0;
        }

        diag_logf("1s wake=%u wr=%u short=%u err=%u xrun=%u pad=%u "
                  "frames=%ld padf=%ld cbytes=%ld frame_size=%zu "
                  "implied=%ld.%02ld delay=%ld avail=%ld rb=%zu",
                  dstat.wakeups, dstat.writes, dstat.short_w, dstat.err,
                  dstat.xrun, dstat.pads, dstat.frames, dstat.pad_frames,
                  dstat.client_bytes, declared, implied_x100 / 100,
                  implied_x100 % 100, (long)delay, (long)avail, rb);

        // Only meaningful once a full second of steady playback has passed.
        if (dstat.frames > 1000 && declared > 0 &&
            (implied_x100 < (long)declared * 100 * 3 / 4 ||
             implied_x100 > (long)declared * 100 * 5 / 4))
            diag_logf("FORMAT MISMATCH: client is writing ~%ld.%02ld bytes per "
                      "frame but the stream was opened with %zu "
                      "(%s %u Hz %u ch). Indices and latency are wrong by that "
                      "ratio.",
                      implied_x100 / 100, implied_x100 % 100, declared,
                      diag_fmt_name(s->ss.format), s->ss.rate, s->ss.channels);
    }

    diag_reset();
}

static void
deh_stream_state_changed(pa_mainloop_api *api, pa_defer_event *de,
                         void *userdata)
{
    pa_stream *s = userdata;
    if (s->state_cb)
        s->state_cb(s, s->state_cb_userdata);
    pa_stream_unref(s);
}

static void
deh_stream_first_readwrite_callback(pa_mainloop_api *api, pa_defer_event *de,
                                    void *userdata)
{
    pa_stream *s = userdata;

    if (s->direction == PA_STREAM_PLAYBACK) {
        size_t writable_size = pa_stream_writable_size(s);
        if (s->write_cb && writable_size > 0)
            s->write_cb(s, writable_size, s->write_cb_userdata);
    } else if (s->direction == PA_STREAM_RECORD) {
        size_t readable_size = pa_stream_readable_size(s);
        if (s->read_cb && readable_size > 0)
            s->read_cb(s, readable_size, s->read_cb_userdata);
    }
    pa_stream_unref(s);
}

// ---------------------------------------------------------------------------
// Hardware playback clock.
//
// snd_pcm_delay through volumioswitch is not a description of the pipeline we
// feed. That ioplug keeps its own buffer and separately sizes its target's, and
// reports the sum:
//
//     *delayp = local_delay + target_delay;
//
// Measured on device: delay=43282 against a negotiated buffer of 22050, with
// avail permanently 0. Soloist derives playback position and remaining backlog
// from that figure and steers speed by the difference, so it rushes, corrects,
// and chops while the write path is provably healthy (frames=44100/s, no
// xruns, no short writes).
//
// Capping the reported number does not fix it: a bounded wrong answer is still
// wrong, and the correction never converges. The fix is a different
// measurement, not a corrected one. hw_ptr from /proc/asound is frames the
// hardware has actually consumed, which no ioplug can inflate.
//
// If hw_ptr cannot be read, the clock holds its last value and says so once.
// It never falls back to the wall clock or to write_index; both were tried and
// both hunt, because neither is the DAC.
// ---------------------------------------------------------------------------

// "/proc/asound/" + NAME_MAX + NUL, and card path + "/" + NAME_MAX +
// "/sub0/status". Sized for NAME_MAX so a long entry cannot truncate a path and
// make the scan skip a PCM silently.
#define APULSE_CARD_PATH 288
#define APULSE_PCM_PATH 576

// SND_PCM_IOPLUG_HW_BUFFER_BYTES is 524288, which is 65536 frames at S24
// stereo. A PCM reporting a buffer that large is the switch, not the DAC.
#define IOPLUG_MAX_FRAMES 65536

struct hw_pcm_snap {
    char status_path[APULSE_PCM_PATH];
    long delay;
    long avail;
    long hw_ptr;
    long buffer_size;
    // hw_ptr advances one period at a time, so the period is the size of the
    // step and therefore the cap on any interpolation between steps.
    long period_size;
    unsigned rate;
    int ioplug;
};

static int
read_key_long(const char *path, const char *key, long *out)
{
    FILE *f = fopen(path, "r");
    char line[256];
    size_t klen;

    if (!f)
        return -1;
    klen = strlen(key);
    while (fgets(line, sizeof(line), f)) {
        if (strncmp(line, key, klen) != 0)
            continue;
        char *colon = strchr(line, ':');

        if (!colon)
            continue;
        *out = strtol(colon + 1, NULL, 10);
        fclose(f);
        return 0;
    }
    fclose(f);
    return -1;
}

static int
read_key_u(const char *path, const char *key, unsigned *out)
{
    long v;

    if (read_key_long(path, key, &v) < 0)
        return -1;
    if (v < 0)
        return -1;
    *out = (unsigned)v;
    return 0;
}

static int
status_is_running(const char *path)
{
    FILE *f = fopen(path, "r");
    char line[256];

    if (!f)
        return 0;
    while (fgets(line, sizeof(line), f)) {
        if (strncmp(line, "state", 5) != 0)
            continue;
        fclose(f);
        return strstr(line, "RUNNING") != NULL;
    }
    fclose(f);
    return 0;
}

// snd-aloop presents a playback PCM that is not a DAC. Skip it, or the clock
// can lock onto a loopback that no speaker is attached to.
static int
card_is_loopback(const char *card_dir)
{
    char idpath[APULSE_PCM_PATH];
    char id[64];
    FILE *f;
    int n;

    n = snprintf(idpath, sizeof(idpath), "%s/id", card_dir);
    if (n < 0 || (size_t)n >= sizeof(idpath))
        return 0;
    f = fopen(idpath, "r");
    if (!f)
        return 0;
    if (!fgets(id, sizeof(id), f)) {
        fclose(f);
        return 0;
    }
    fclose(f);
    return strncmp(id, "Loopback", 8) == 0;
}

static int
read_hw_pcm_snap(const char *status_path, struct hw_pcm_snap *o)
{
    char hw_params[APULSE_PCM_PATH];
    size_t n;

    memset(o, 0, sizeof(*o));
    n = strlen(status_path);
    if (n >= sizeof(o->status_path) || n < 7)
        return -1;
    memcpy(o->status_path, status_path, n + 1);
    if (strcmp(status_path + n - 6, "status") != 0)
        return -1;
    memcpy(hw_params, status_path, n - 6);
    memcpy(hw_params + n - 6, "hw_params", 10);

    if (!status_is_running(status_path))
        return -1;
    if (read_key_long(status_path, "hw_ptr", &o->hw_ptr) < 0)
        return -1;
    if (read_key_long(status_path, "delay", &o->delay) < 0)
        o->delay = -1;
    if (read_key_long(status_path, "avail", &o->avail) < 0)
        o->avail = -1;
    if (read_key_long(hw_params, "buffer_size", &o->buffer_size) < 0)
        o->buffer_size = -1;
    if (read_key_long(hw_params, "period_size", &o->period_size) < 0)
        o->period_size = 0;
    if (read_key_u(hw_params, "rate", &o->rate) < 0)
        o->rate = 0;
    o->ioplug = (o->buffer_size >= IOPLUG_MAX_FRAMES);
    return 0;
}

static int
scan_hw_pcm(unsigned want_rate, struct hw_pcm_snap *best)
{
    DIR *cards;
    struct dirent *ce;
    int found = 0;
    struct hw_pcm_snap pick;

    memset(&pick, 0, sizeof(pick));
    cards = opendir("/proc/asound");
    if (!cards)
        return -1;

    while ((ce = readdir(cards))) {
        char card_dir[APULSE_CARD_PATH];
        DIR *pcms;
        struct dirent *pe;
        int n;

        if (strncmp(ce->d_name, "card", 4) != 0)
            continue;
        n = snprintf(card_dir, sizeof(card_dir), "/proc/asound/%s",
                     ce->d_name);
        if (n < 0 || (size_t)n >= sizeof(card_dir))
            continue;
        if (card_is_loopback(card_dir))
            continue;
        pcms = opendir(card_dir);
        if (!pcms)
            continue;
        while ((pe = readdir(pcms))) {
            char status_path[APULSE_PCM_PATH];
            struct hw_pcm_snap snap;
            size_t plen = strlen(pe->d_name);
            int sn;

            if (plen < 2 || pe->d_name[0] != 'p' ||
                strncmp(pe->d_name, "pcm", 3) != 0)
                continue;
            if (pe->d_name[plen - 1] != 'p')
                continue;
            sn = snprintf(status_path, sizeof(status_path),
                          "%s/%s/sub0/status", card_dir, pe->d_name);
            if (sn < 0 || (size_t)sn >= sizeof(status_path))
                continue;
            if (read_hw_pcm_snap(status_path, &snap) < 0)
                continue;
            if (want_rate && snap.rate && snap.rate != want_rate)
                continue;
            if (!found) {
                pick = snap;
                found = 1;
                continue;
            }
            // Prefer the DAC over the ioplug-sized buffer, then the smaller
            // delay, which is the one closer to the hardware pointer.
            if (pick.ioplug && !snap.ioplug)
                pick = snap;
            else if (pick.ioplug == snap.ioplug && snap.delay >= 0 &&
                     (pick.delay < 0 || snap.delay < pick.delay))
                pick = snap;
        }
        closedir(pcms);
    }
    closedir(cards);
    if (!found)
        return -1;
    *best = pick;
    return 0;
}

// hw_ptr in /proc/asound is printed as a 32-bit value and wraps. Track the high
// word so a long session does not jump backwards every 27 hours at 44100.
static int64_t
unwrap_hw_ptr(int64_t last, long raw)
{
    int64_t cur = (int64_t)(uint32_t)raw;

    if (last < 0)
        return (int64_t)raw;
    {
        int64_t last32 = last & 0xffffffffLL;
        int64_t hi = last - last32;

        if (cur + 0x40000000LL < last32)
            hi += 0x100000000LL;
        return hi + cur;
    }
}

static void
stream_clock_reset(pa_stream *s)
{
    s->clock_origin_hw = 0;
    s->clock_last_hw = -1;
    s->clock_frozen_usec = 0;
    s->clock_last_played = 0;
    s->clock_running = 0;
    s->clock_have_origin = 0;
    s->clock_have_path = 0;
    s->clock_status_path[0] = '\0';
    s->clock_model_at.tv_sec = 0;
    s->clock_model_at.tv_usec = 0;
    s->clock_model_frames = 0;
    s->clock_model_rate = 0.0;
    s->clock_model_valid = 0;
}

// ---------------------------------------------------------------------------
// Smoothed clock model.
//
// The raw hardware position is accurate. Measured over 30 s of lossless
// playback, correlating the derived playback rate against the interval between
// consecutive client reads:
//
//   gap between reads   share   rate outside 0.9-1.1
//   over 20 ms           5.9%     0.0%
//   5 to 20 ms           2.0%     0.0%
//   1 to 5 ms            1.2%     1.0%
//   0.2 to 1 ms          1.3%    61.0%
//   under 0.2 ms        89.6%    84.1%
//
// Above a millisecond the clock is essentially exact: median 0.996 to 1.000,
// tenth and ninetieth percentiles within a fraction of a percent. Nothing is
// wrong with the position or the timestamp. But Soloist takes 90% of its reads
// in bursts under 200 us apart, and at 44100 the /proc read costs 70 us, which
// is 3 frames. Differencing two samples that close divides a 3-frame quantum by
// a near-zero interval, so the apparent rate swings between 0.5x and 1.5x.
// That is arithmetic, not a fault, and no amount of measurement accuracy fixes
// it.
//
// Real PulseAudio does not answer with a fresh measurement each time. It keeps
// a model of the clock and answers from that, so two reads microseconds apart
// return values consistent with each other by construction.
//
// The model here is a position anchor plus a rate, re-fitted whenever the
// hardware has advanced far enough to measure a rate reliably. Between fits the
// answer is anchor + rate * elapsed, which is smooth at any sampling interval.
// The rate is fitted from the hardware, not assumed from the sample rate, so a
// device clock that runs slightly fast or slow is tracked rather than papered
// over.
// ---------------------------------------------------------------------------

// Refit no more often than this: below it the hardware has not moved enough for
// the measured rate to mean anything.
#define CLOCK_MODEL_MIN_FIT_US 100000

// A fitted rate this far from nominal is not a clock, it is a glitch: a period
// boundary landing badly, or the process being descheduled mid-read. Keep the
// previous rate rather than tracking noise.
#define CLOCK_MODEL_MAX_DRIFT 0.05

static void
stream_clock_model_update(pa_stream *s, int64_t hw_frames,
                          const struct timeval *now)
{
    long elapsed_us;
    double measured;

    if (!s->clock_model_valid) {
        s->clock_model_at = *now;
        s->clock_model_frames = hw_frames;
        s->clock_model_rate = (double)s->ss.rate;
        s->clock_model_valid = 1;
        return;
    }

    elapsed_us = (long)(now->tv_sec - s->clock_model_at.tv_sec) * 1000000L +
                 (now->tv_usec - s->clock_model_at.tv_usec);
    if (elapsed_us < CLOCK_MODEL_MIN_FIT_US)
        return;

    measured = (double)(hw_frames - s->clock_model_frames) * 1000000.0 /
               (double)elapsed_us;

    if (s->ss.rate > 0) {
        double nominal = (double)s->ss.rate;

        if (measured > nominal * (1.0 - CLOCK_MODEL_MAX_DRIFT) &&
            measured < nominal * (1.0 + CLOCK_MODEL_MAX_DRIFT)) {
            // Ease toward the measurement rather than jumping to it, so one odd
            // fit cannot step the reported rate.
            s->clock_model_rate =
                s->clock_model_rate * 0.75 + measured * 0.25;
        }
    }

    // Re-anchor on the real position either way, so the model can never drift
    // away from the hardware even if its rate is briefly wrong.
    s->clock_model_at = *now;
    s->clock_model_frames = hw_frames;
}

// Position from the model: anchor plus fitted rate times elapsed. Clamped so it
// cannot run ahead of the hardware by more than one period, which is the
// furthest ahead the truth could plausibly be.
static int64_t
stream_clock_model_frames(pa_stream *s, const struct timeval *now,
                          int64_t hw_frames, long period)
{
    long elapsed_us;
    int64_t est;

    if (!s->clock_model_valid)
        return hw_frames;

    elapsed_us = (long)(now->tv_sec - s->clock_model_at.tv_sec) * 1000000L +
                 (now->tv_usec - s->clock_model_at.tv_usec);
    if (elapsed_us < 0)
        elapsed_us = 0;

    est = s->clock_model_frames +
          (int64_t)(s->clock_model_rate * (double)elapsed_us / 1000000.0);

    if (period > 0 && est > hw_frames + period)
        est = hw_frames + period;
    if (est < hw_frames)
        est = hw_frames;

    return est;
}

static void
stream_clock_start(pa_stream *s)
{
    s->clock_running = 1;
}

// Corking stops the DAC advancing on our behalf, so freeze at the last known
// position rather than letting hw_ptr keep running under a paused stream.
static void
stream_clock_freeze(pa_stream *s)
{
    if (!s->clock_running)
        return;
    s->clock_frozen_usec = s->clock_last_played;
    s->clock_running = 0;
}

static pa_usec_t
stream_hw_time(pa_stream *s)
{
    struct hw_pcm_snap snap;
    int64_t hw;
    int64_t frames;
    pa_usec_t played;
    unsigned rate = s->ss.rate;
    struct timeval read_t0, read_t1;

    if (!s->clock_running)
        return s->clock_frozen_usec;
    if (rate == 0)
        return s->clock_last_played;

    gettimeofday(&read_t0, NULL);

    if (!s->clock_have_path) {
        if (scan_hw_pcm(rate, &snap) == 0) {
            snprintf(s->clock_status_path, sizeof(s->clock_status_path), "%s",
                     snap.status_path);
            s->clock_have_path = 1;
            if (!s->clock_logged) {
                diag_logf("play_clock %s hw_ptr=%ld delay=%ld buffer=%ld "
                          "ioplug=%d",
                          snap.status_path, snap.hw_ptr, snap.delay,
                          snap.buffer_size, snap.ioplug);
                s->clock_logged = 1;
            }
        } else {
            // Explicit, once. No silent fallback to the wall clock or to
            // write_index: both were tried on this device and both hunt.
            if (!s->clock_logged) {
                diag_logf("play_clock: no hardware hw_ptr found; position held");
                s->clock_logged = 1;
            }
            return s->clock_last_played;
        }
    } else if (read_hw_pcm_snap(s->clock_status_path, &snap) < 0) {
        s->clock_have_path = 0;
        return s->clock_last_played;
    }

    // How long the position measurement itself took, and how stale
    // timing_info.timestamp already is by the time we have a position.
    //
    // stream_update_timing sets timestamp with gettimeofday and then calls this,
    // which opens, reads and parses two files under /proc. The client differences
    // consecutive (position, timestamp) pairs to derive a rate, so any variation
    // in that interval appears to it as the audio speeding up or slowing down.
    //
    // After the period interpolation landed, zero-motion reads fell from 32% to
    // 0.2% but the spread stayed wide: 40% of reads between 0.5x and 0.9x, 35%
    // between 1.1x and 2.0x, symmetric around 1.0. That symmetry is what a
    // varying measurement offset looks like, so measure it before changing
    // anything else.
    gettimeofday(&read_t1, NULL);
    if (diag_full()) {
        long proc_us = (long)(read_t1.tv_sec - read_t0.tv_sec) * 1000000L +
                       (read_t1.tv_usec - read_t0.tv_usec);
        long stale_us =
            (long)(read_t1.tv_sec - s->timing_info.timestamp.tv_sec) * 1000000L +
            (read_t1.tv_usec - s->timing_info.timestamp.tv_usec);

        diag_logf("clockread proc_us=%ld stale_us=%ld hw_ptr=%ld rate=%.1f",
                  proc_us, stale_us, snap.hw_ptr, s->clock_model_rate);
    }

    hw = unwrap_hw_ptr(s->clock_last_hw, snap.hw_ptr);
    s->clock_last_hw = hw;

    // Answer from the model, not from this sample.
    //
    // The previous version interpolated from the last time hw_ptr changed,
    // which removed the staircase (zero-motion reads fell from 32% to 0.2%)
    // but left the spread wide: 40% of reads still read 0.5x to 0.9x and 35%
    // read 1.1x to 2.0x. Correlating those against the sampling interval showed
    // why: every bad reading came from a burst where the client polled less
    // than a millisecond apart, and above 1 ms the clock was already exact.
    //
    // A per-read estimate cannot fix that, because the error is in dividing a
    // small measurement quantum by a smaller interval. The model can, because
    // it does not divide anything at read time.
    {
        struct timeval now;

        gettimeofday(&now, NULL);
        stream_clock_model_update(s, hw, &now);
        hw = stream_clock_model_frames(s, &now, hw, snap.period_size);
    }
    if (!s->clock_have_origin) {
        s->clock_origin_hw = hw;
        s->clock_have_origin = 1;
        // Resuming after a cork: place the origin so the position continues
        // from where it froze instead of restarting at zero.
        if (s->clock_frozen_usec > 0 && rate > 0)
            s->clock_origin_hw =
                hw - (int64_t)((uint64_t)s->clock_frozen_usec * rate /
                               1000000ULL);
    }
    frames = hw - s->clock_origin_hw;
    if (frames < 0)
        frames = 0;
    played = (pa_usec_t)((uint64_t)frames * 1000000ULL / rate);
    // Monotonic by contract: pa_stream_get_time must never go backwards.
    if (played < s->clock_last_played)
        played = s->clock_last_played;
    s->clock_last_played = played;
    return played;
}

// ---------------------------------------------------------------------------
// Playback flow control.
//
// POLLOUT is level-triggered. When the ring is empty the io callback has nothing
// to write, returns without changing the registration, and is re-entered
// immediately. Upstream masks this by writing a period of silence every time,
// which keeps the callback rate sane but inserts silence into a pipeline that is
// usually not short of audio, and advances the hardware pointer with frames the
// client never wrote.
//
// Measured on device before this change: wake=99 wr=99 pad=49 padf=0 in every
// single second. Half the wakeups wrote zero frames. Under a full ring after a
// track change it degenerated further, into tens of thousands of wakeups per
// second, saturating the mainloop thread that also serves the control socket.
//
// Instead, drop POLLOUT while there is nothing to send and restore it when the
// client writes. No timers, no polling, no silence.
// ---------------------------------------------------------------------------

static void
stream_set_output_enabled(pa_stream *s, int enable)
{
    pa_mainloop_api *api;
    pa_io_event_flags_t events;

    if (!s || !s->ioe || s->out_enabled == enable)
        return;

    api = s->c->mainloop_api;
    if (!api || !api->io_enable)
        return;

    // Keep apulse's high-bit marker and any input interest; only PA_IO_EVENT_OUTPUT
    // is toggled.
    events = enable ? s->ioe_events : (s->ioe_events & ~PA_IO_EVENT_OUTPUT);

    for (int k = 0; k < s->nioe; k++) {
        if (s->ioe[k])
            api->io_enable(s->ioe[k], events);
    }
    s->out_enabled = enable;
}

// Called from pa_stream_write and from cork, both of which mean the stream may
// have something to send again.
static int stream_maybe_yield(pa_stream *s);
static void stream_release_device(pa_stream *s, int keep_position);
static void stream_schedule_acquire(pa_stream *s);

static void
stream_wake_output(pa_stream *s)
{
    stream_set_output_enabled(s, 1);
}

static void
data_available_for_stream(pa_mainloop_api *a, pa_io_event *ioe, int fd,
                          pa_io_event_flags_t events, void *userdata)
{
    pa_stream *s = userdata;
    snd_pcm_sframes_t frame_count;
    size_t frame_size = pa_frame_size(&s->ss);
    char buf[16 * 1024];
    const size_t buf_size = pa_find_multiple_of(sizeof(buf), frame_size, 0);
    int paused = g_atomic_int_get(&s->paused);

    if (stream_maybe_yield(s))
        return;
    if (!s->ph)
        return;

    if (events & (PA_IO_EVENT_INPUT | PA_IO_EVENT_OUTPUT)) {

#if HAVE_SND_PCM_AVAIL
        frame_count = snd_pcm_avail(s->ph);
#else
        snd_pcm_hwsync(s->ph);
        frame_count = snd_pcm_avail_update(s->ph);
#endif

        if (frame_count < 0) {
            if (frame_count == -EBADFD) {
                // stream was closed
                return;
            }

            int cnt = 0, ret;

            dstat.xrun++;
            do {
                cnt++;
                ret = snd_pcm_recover(s->ph, frame_count, 1);
            } while (ret == -1 && errno == EINTR && cnt < 5);

            switch (snd_pcm_state(s->ph)) {
            case SND_PCM_STATE_OPEN:
                // Highly unlikely device will be here in this state. But if it
                // is, there is nothing can be done.
                trace_error(
                    "Stream '%s' of context '%s' have its associated PCM "
                    "device in SND_PCM_STATE_OPEN state. Reconfiguration is "
                    "required, but is not possible at the moment. Giving up.",
                    s->name ? s->name : "", s->c->name ? s->c->name : "");
                break;

            case SND_PCM_STATE_SETUP:
                // There is configuration, but device is not prepared and not
                // started.
                snd_pcm_prepare(s->ph);
                snd_pcm_start(s->ph);
                break;

            case SND_PCM_STATE_PREPARED:
                // Device prepared, but not started.
                snd_pcm_start(s->ph);
                break;

            case SND_PCM_STATE_RUNNING:
                // That's the expected state.
                break;

            case SND_PCM_STATE_XRUN:
                trace_error(
                    "Stream '%s' of context '%s' have its associated device in "
                    "SND_PCM_STATE_XRUN state even after xrun recovery.",
                    s->name ? s->name : "", s->c->name ? s->c->name : "");
                break;

            case SND_PCM_STATE_DRAINING:
                trace_error(
                    "Stream '%s' of context '%s' have its associated device in "
                    "SND_PCM_STATE_DRAINING state, which is highly unusual.",
                    s->name ? s->name : "", s->c->name ? s->c->name : "");
                break;

            case SND_PCM_STATE_PAUSED:
                // Resume from paused state.
                snd_pcm_pause(s->ph, 0);
                break;

            case SND_PCM_STATE_SUSPENDED:
                // Resume from suspended state.
                snd_pcm_resume(s->ph);
                break;

            case SND_PCM_STATE_DISCONNECTED:
                trace_error(
                    "Stream '%s' of context '%s' have its associated device in "
                    "SND_PCM_STATE_DISCONNECTED state. Giving up.",
                    s->name ? s->name : "", s->c->name ? s->c->name : "");
                break;
            default:
                // avoid compiler warnings of unhandled (library-private) enum values
                break;
            }

#if HAVE_SND_PCM_AVAIL
            frame_count = snd_pcm_avail(s->ph);
#else
            snd_pcm_hwsync(s->ph);
            frame_count = snd_pcm_avail_update(s->ph);
#endif

            if (frame_count < 0) {
                // volumioswitch can return EPIPE from a stale target while the
                // switcher handle stays open. recover/prepare does not reopen
                // that target; only close + snd_pcm_open does.
                diag_logf("pcm unrecovered (%ld), reopening",
                          (long)frame_count);
                stream_release_device(s, 1);
                if (s->want_running)
                    stream_schedule_acquire(s);
                return;
            }
        }
    } else {
        return;
    }

    if (events & PA_IO_EVENT_OUTPUT) {
        dstat.wakeups++;
        if (paused) {
            // client stream is corked. Pass silence to ALSA
            size_t bytecnt = MIN(buf_size, frame_count * frame_size);
            snd_pcm_sframes_t wr;

            memset(buf, 0, bytecnt);
            wr = snd_pcm_writei(s->ph, buf, bytecnt / frame_size);
            if (wr < 0 && wr != -EAGAIN) {
                diag_logf("writei failed (%ld), reopening", (long)wr);
                stream_release_device(s, 1);
                if (s->want_running)
                    stream_schedule_acquire(s);
                return;
            }
            diag_note_write(wr, bytecnt / frame_size, 1);
        } else {
            size_t writable_size = pa_stream_writable_size(s);

            // Ask client for data, but only if we are ready for at least
            // |minreq| bytes.
            if (s->write_cb && writable_size >= s->buffer_attr.minreq)
                s->write_cb(s, s->buffer_attr.minreq, s->write_cb_userdata);

            size_t bytecnt = MIN(buf_size, frame_count * frame_size);
            bytecnt = ringbuffer_read(s->rb, buf, bytecnt);

            pa_apply_volume_multiplier(buf, bytecnt, s->volume, &s->ss);
            pa_apply_output_trim(buf, bytecnt, &s->ss);

            snd_pcm_sframes_t wr;

            if (bytecnt == 0) {
                // Nothing to send. Stop asking until the client writes, rather
                // than inserting silence or spinning on a level-triggered
                // POLLOUT. The ALSA buffer keeps playing what it already holds.
                stream_set_output_enabled(s, 0);
                diag_tick(s);
                return;
            }
            wr = snd_pcm_writei(s->ph, buf, bytecnt / frame_size);
            if (wr < 0 && wr != -EAGAIN) {
                diag_logf("writei failed (%ld), reopening", (long)wr);
                stream_release_device(s, 1);
                if (s->want_running)
                    stream_schedule_acquire(s);
                return;
            }
            if (wr > 0)
                stream_clock_start(s);
            diag_note_write(wr, bytecnt / frame_size, 0);
        }
        diag_tick(s);
    }

    if (events & PA_IO_EVENT_INPUT) {
        if (paused) {
            // client stream is corked. Read data from ALSA and discard them
            size_t bytecnt = MIN(buf_size, frame_count * frame_size);
            snd_pcm_readi(s->ph, buf, bytecnt / frame_size);
        } else {
            size_t bytecnt = ringbuffer_writable_size(s->rb);

            if (bytecnt == 0) {
                // ringbuffer is full because app doesn't read data fast enough.
                // Make some room
                ringbuffer_drop(s->rb, frame_count * frame_size);
                bytecnt = ringbuffer_writable_size(s->rb);
            }

            bytecnt = MIN(bytecnt, frame_count * frame_size);
            bytecnt = MIN(bytecnt, buf_size);

            if (bytecnt > 0) {
                snd_pcm_readi(s->ph, buf, bytecnt / frame_size);
                pa_apply_volume_multiplier(buf, bytecnt, s->c->source_volume,
                                           &s->ss);
                ringbuffer_write(s->rb, buf, bytecnt);
            }

            size_t readable_size = pa_stream_readable_size(s);
            if (s->read_cb && readable_size > 0)
                s->read_cb(s, readable_size, s->read_cb_userdata);
        }
    }
}

static void
alsa_error_quiet(const char *file, int line, const char *function, int err,
                 const char *fmt, ...)
{
    (void)file;
    (void)line;
    (void)function;
    (void)err;
    (void)fmt;
}

static int
do_connect_pcm(pa_stream *s, snd_pcm_stream_t stream_direction)
{
    snd_pcm_hw_params_t *hw_params;
    snd_pcm_sw_params_t *sw_params;
    int errcode = 0;
    const char *device_name;
    const char *direction_name;

    switch (stream_direction) {
    default:
    case SND_PCM_STREAM_PLAYBACK:
        device_name = getenv("APULSE_PLAYBACK_DEVICE");
        direction_name = "playback";
        break;
    case SND_PCM_STREAM_CAPTURE:
        device_name = getenv("APULSE_CAPTURE_DEVICE");
        direction_name = "capture";
        break;
    }

    if (device_name == NULL)
        device_name = "default";

    char *device_description =
        g_strdup_printf("%s device \"%s\"", direction_name, device_name);
    if (!device_description) {
        trace_error("%s: can't allocate memory for device description string\n",
                    __func__);
        goto fatal_error;
    }

    // EBUSY is the other source still holding plug:volumio. The open is
    // retried. volumioswitch prints its own "failed to open the switcher
    // target" on that path; mute the ALSA handler for this call only so
    // a handover does not look like a device fault in the journal.
    snd_lib_error_set_handler(alsa_error_quiet);
    errcode = snd_pcm_open(&s->ph, device_name, stream_direction, 0);
    snd_lib_error_set_handler(NULL);
    if (errcode < 0) {
        if (errcode != -EBUSY)
            trace_error("%s: can't open %s. Error code %d (%s)\n", __func__,
                        device_description, errcode, snd_strerror(errcode));
        goto fatal_error;
    }

    errcode = snd_pcm_hw_params_malloc(&hw_params);
    if (errcode < 0) {
        trace_error(
            "%s: can't allocate memory for hw parameters for %s. Error code %d "
            "(%s)\n",
            __func__, device_description, errcode, snd_strerror(errcode));
        goto fatal_error;
    }

    errcode = snd_pcm_hw_params_any(s->ph, hw_params);
    if (errcode < 0) {
        trace_error(
            "%s: can't get initial hw parameters for %s. Error code %d (%s)\n",
            __func__, device_description, errcode, snd_strerror(errcode));
        goto fatal_error;
    }

    errcode = snd_pcm_hw_params_set_access(s->ph, hw_params,
                                           SND_PCM_ACCESS_RW_INTERLEAVED);
    if (errcode < 0) {
        trace_error(
            "%s: can't select interleaved mode for %s. Error code %d (%s)\n",
            __func__, device_description, errcode, snd_strerror(errcode));
        // TODO: is it worth to support non-interleaved mode?
        goto fatal_error;
    }

    errcode = snd_pcm_hw_params_set_format(s->ph, hw_params,
                                           pa_format_to_alsa(s->ss.format));
    if (errcode < 0) {
        snd_pcm_format_t alsa_format = pa_format_to_alsa(s->ss.format);
        trace_error(
            "%s: can't set sample format %d (\"%s\") for %s. Error code %d "
            "(%s)\n",
            __func__, alsa_format, snd_pcm_format_name(alsa_format),
            device_description, errcode, snd_strerror(errcode));
        goto fatal_error;
    }

    errcode = snd_pcm_hw_params_set_rate_resample(s->ph, hw_params, 1);
    if (errcode < 0) {
        trace_error(
            "%s: can't enable rate resample for %s. Error code %d (%s)\n",
            __func__, device_description, errcode, snd_strerror(errcode));
        // This is not a fatal error. Audio speed will be wrong, but there will
        // be something.
    }

    unsigned int rate = s->ss.rate;
    int dir = 0;

    errcode = snd_pcm_hw_params_set_rate_near(s->ph, hw_params, &rate, &dir);
    if (errcode < 0) {
        trace_error("%s: can't set sample rate for %s. Error code %d (%s)\n",
                    __func__, device_description, errcode,
                    snd_strerror(errcode));
        goto fatal_error;
    }

    trace_info_f("%s: demanded %d Hz sample rate, got %d Hz for %s, dir = %d\n",
                 __func__, (int)s->ss.rate, (int)rate, device_description, dir);

    if (rate != s->ss.rate)
        trace_error(
            "%s: actual sample rate, %d Hz, differs from required %d Hz\n",
            __func__, (int)rate, (int)s->ss.rate);

    errcode = snd_pcm_hw_params_set_channels(s->ph, hw_params, s->ss.channels);
    if (errcode < 0) {
        trace_error(
            "%s: can't set channel count to %d for %s. Error code %d (%s)\n",
            __func__, (int)s->ss.channels, device_description, errcode,
            snd_strerror(errcode));
        // TODO: channel count handling?
        goto fatal_error;
    }

    const size_t frame_size = pa_frame_size(&s->ss);
    snd_pcm_uframes_t requested_period_size =
        s->buffer_attr.minreq / frame_size;
    snd_pcm_uframes_t period_size = requested_period_size;
    dir = 1;  // Prefer larger period sizes, if exact is not possible.
    errcode = snd_pcm_hw_params_set_period_size_near(s->ph, hw_params,
                                                     &period_size, &dir);
    if (errcode < 0) {
        trace_error(
            "%s: can't set period size to %d frames for %s. Error code %d "
            "(%s)\n",
            __func__, (int)requested_period_size, device_description, errcode,
            snd_strerror(errcode));
        goto fatal_error;
    }

    trace_info_f(
        "%s: requested period size of %d frames, got %d frames for %s\n",
        __func__, (int)requested_period_size, (int)period_size,
        device_description);

    // Set up buffer size. Ensure it's at least four times larger than a period
    // size.
    snd_pcm_uframes_t requested_buffer_size =
        s->buffer_attr.tlength / frame_size;
    snd_pcm_uframes_t buffer_size = MAX(requested_buffer_size, 4 * period_size);
    errcode =
        snd_pcm_hw_params_set_buffer_size_near(s->ph, hw_params, &buffer_size);
    if (errcode < 0) {
        trace_error(
            "%s: can't set buffer size to %d frames for %s. Error code %d "
            "(%s)\n",
            __func__, (int)buffer_size, device_description, errcode,
            snd_strerror(errcode));
        goto fatal_error;
    }

    trace_info_f(
        "%s: requested buffer size of %d frames, got %d frames for %s\n",
        __func__, (int)requested_buffer_size, (int)buffer_size,
        device_description);

    snd_pcm_format_t negotiated_format = SND_PCM_FORMAT_UNKNOWN;

    errcode = snd_pcm_hw_params(s->ph, hw_params);
    if (errcode < 0) {
        trace_error("%s: can't apply configured hw parameter block for %s\n",
                    __func__, device_description);
        goto fatal_error;
    }

    // Captured before the params block is freed. The Pulse spec says what the
    // client asked for; this says what ALSA agreed to, which on a plug chain
    // can differ. Without it a format problem is indistinguishable from a
    // timing one.
    snd_pcm_hw_params_get_format(hw_params, &negotiated_format);

    snd_pcm_hw_params_free(hw_params);

    errcode = snd_pcm_sw_params_malloc(&sw_params);
    if (errcode < 0) {
        trace_error("%s: can't allocate memory for sw parameters for %s\n",
                    __func__, device_description);
        goto fatal_error;
    }

    errcode = snd_pcm_sw_params_current(s->ph, sw_params);
    if (errcode < 0) {
        trace_error("%s: can't acquire current sw parameters for %s\n",
                    __func__, device_description);
        goto fatal_error;
    }

    errcode = snd_pcm_sw_params_set_avail_min(s->ph, sw_params, period_size);
    if (errcode < 0) {
        trace_error("%s: can't set avail min for %s\n", __func__,
                    device_description);
        goto fatal_error;
    }

    // no period event requested

    errcode = snd_pcm_sw_params(s->ph, sw_params);
    if (errcode < 0) {
        trace_error("%s: can't apply sw parameters for %s\n", __func__,
                    device_description);
        goto fatal_error;
    }

    snd_pcm_sw_params_free(sw_params);

    errcode = snd_pcm_prepare(s->ph);
    if (errcode < 0) {
        trace_error("%s: can't prepare PCM device to use for %s\n", __func__,
                    device_description);
        goto fatal_error;
    }

    int nfds = snd_pcm_poll_descriptors_count(s->ph);
    struct pollfd *fds = calloc(nfds, sizeof(struct pollfd));
    s->ioe = calloc(nfds, sizeof(pa_io_event *));
    s->nioe = nfds;
    snd_pcm_poll_descriptors(s->ph, fds, nfds);
    for (int k = 0; k < nfds; k++) {
        pa_mainloop_api *api = s->c->mainloop_api;
        s->ioe[k] = api->io_new(api, fds[k].fd, 0x80000000 | fds[k].events,
                                data_available_for_stream, s);
        s->ioe[k]->pcm = s->ph;
        // Remember the full mask so flow control can restore it exactly.
        s->ioe_events = 0x80000000 | fds[k].events;
    }
    s->out_enabled = 1;
    free(fds);

    if (stream_direction == SND_PCM_STREAM_PLAYBACK && s->ss.rate > 0)
        s->configured_sink_usec =
            (pa_usec_t)((uint64_t)buffer_size * 1000000ULL / s->ss.rate);

    diag_logf("connect %s: pulse %s %u Hz %u ch frame=%zu | alsa %s period=%lu "
              "buffer=%lu periods=%lu.%02lu fds=%d | tlength=%u minreq=%u "
              "prebuf=%u",
              direction_name, diag_fmt_name(s->ss.format), s->ss.rate,
              s->ss.channels, pa_frame_size(&s->ss),
              snd_pcm_format_name(negotiated_format),
              (unsigned long)period_size, (unsigned long)buffer_size,
              period_size ? (unsigned long)(buffer_size / period_size) : 0UL,
              period_size
                  ? (unsigned long)((buffer_size % period_size) * 100 /
                                    period_size)
                  : 0UL,
              nfds, s->buffer_attr.tlength, s->buffer_attr.minreq,
              s->buffer_attr.prebuf);
    diag_reset();

    if (s->state != PA_STREAM_READY) {
        s->state = PA_STREAM_READY;
        pa_stream_ref(s);
        s->c->mainloop_api->defer_new(s->c->mainloop_api,
                                      deh_stream_state_changed, s);
        pa_stream_ref(s);
        s->c->mainloop_api->defer_new(
            s->c->mainloop_api, deh_stream_first_readwrite_callback, s);
    }

    g_free(device_description);
    return 0;

fatal_error:
    if (errcode != -EBUSY)
        trace_error(
            "%s: failed to open ALSA device. Apulse does no resampling or "
            "format conversion, leaving that task to ALSA plugins. Ensure that "
            "selected device is capable of playing a particular sample format "
            "at a particular rate. They have to be supported by either "
            "hardware directly, or by \"plug\" and \"dmix\" ALSA plugins which "
            "will perform required conversions on CPU.\n",
            __func__);

    if (errcode == -EACCES) {
        trace_error(
            "%s: additionally, the error code is %d, which means access was "
            "denied. That looks like access restriction in a sandbox. If the "
            "app you are running uses sandboxing techniques, make sure "
            "/dev/snd/ directory is added into the allowed list. Both reading "
            "and writing access to the files in that directory are required.\n",
            __func__, -EACCES);
    }

    g_free(device_description);
    return errcode < 0 ? errcode : -1;
}

APULSE_EXPORT
int
pa_stream_begin_write(pa_stream *p, void **data, size_t *nbytes)
{
    trace_info_f("F %s p=%p nbytes=%p(%" PRIu64 ")\n", __func__, p, nbytes,
                 (uint64_t)(nbytes ? *nbytes : 0));

    free(p->write_buffer);

    if (*nbytes == (size_t)-1)
        *nbytes = 8192;

    *nbytes = pa_find_multiple_of(*nbytes, pa_frame_size(&p->ss), 0);

    p->write_buffer = malloc(*nbytes);

    if (!p->write_buffer)
        return -1;

    *data = p->write_buffer;

    return 0;
}

APULSE_EXPORT
int
pa_stream_cancel_write(pa_stream *p)
{
    trace_info_f("F %s p=%p\n", __func__, p);

    free(p->write_buffer);
    p->write_buffer = NULL;

    return 0;
}

static void
stream_adjust_buffer_attrs(pa_stream *s, const pa_buffer_attr *attr)
{
    pa_buffer_attr *ba = &s->buffer_attr;
    const size_t frame_size = pa_frame_size(&s->ss);

    if (attr) {
        *ba = *attr;
    } else {
        // If client passed NULL, all parameters have default values.
        ba->maxlength = (uint32_t)-1;
        ba->tlength = (uint32_t)-1;
        ba->prebuf = (uint32_t)-1;
        ba->minreq = (uint32_t)-1;
        ba->fragsize = (uint32_t)-1;
    }

    // Adjust default values.
    // Overall buffer length.
    if (ba->maxlength == (uint32_t)-1)
        ba->maxlength = 4 * 1024 * 1024;

    if (ba->maxlength == 0)
        ba->maxlength = frame_size;

    // Target length of a buffer.
    if (ba->tlength == (uint32_t)-1)
        ba->tlength = pa_usec_to_bytes(2 * 1000 * 1000, &s->ss);

    if (ba->tlength == 0)
        ba->tlength = frame_size;

    ba->tlength = MIN(ba->tlength, ba->maxlength);

    // Cap the target buffer length when APULSE_MAX_TLENGTH_MS is set.
    //
    // On Volumio the chain runs through the volumioswitch ioplug, which keeps
    // its own buffer and separately sizes the buffer of its target PCM from
    // io->buffer_size. Both stages derive from tlength, so the default lands
    // twice in series.
    //
    // Applied after defaulting and before minreq and prebuf are derived, so
    // those stay consistent with the capped value.
    {
        const char *max_tlength_ms = getenv("APULSE_MAX_TLENGTH_MS");
        long ms = max_tlength_ms ? strtol(max_tlength_ms, NULL, 10) : 0;

        if (ms > 0) {
            size_t cap = pa_usec_to_bytes((pa_usec_t)ms * 1000, &s->ss);

            if (cap >= frame_size && ba->tlength > cap)
                ba->tlength = cap;
        }
    }

    // Minimum request (playback).
    if (ba->minreq == (uint32_t)-1) {
        ba->minreq = pa_usec_to_bytes(20 * 1000, &s->ss);
        ba->minreq = MIN(ba->minreq, ba->tlength / 4);
    }

    if (ba->minreq == 0)
        ba->minreq = frame_size;

    // Keep at least four requests per buffer, including when the client set
    // minreq explicitly and the cap above shrank tlength underneath it.
    if (ba->minreq > ba->tlength / 4 && ba->tlength / 4 >= frame_size)
        ba->minreq = ba->tlength / 4;

    // Fragment size (recording).
    if (ba->fragsize == (uint32_t)-1) {
        ba->fragsize = pa_usec_to_bytes(20 * 1000, &s->ss);
    }

    if (ba->fragsize == 0)
        ba->fragsize = frame_size;

    // Pre-buffering.
    if (ba->prebuf == (uint32_t)-1)
        ba->prebuf = ba->tlength - ba->minreq;

    if (ba->prebuf > ba->tlength - ba->minreq)
        ba->prebuf = ba->tlength - ba->minreq;

    // Ensure values are all multiple of |frame_size|.
    ba->maxlength = pa_find_multiple_of(ba->maxlength, frame_size, 1);
    ba->tlength = pa_find_multiple_of(ba->tlength, frame_size, 1);
    ba->prebuf = pa_find_multiple_of(ba->prebuf, frame_size, 1);
    ba->minreq = pa_find_multiple_of(ba->minreq, frame_size, 1);
    ba->fragsize = pa_find_multiple_of(ba->fragsize, frame_size, 1);
}

// ---------------------------------------------------------------------------
// Device ownership.
//
// plug:volumio is exclusive: the open handle is the lock. Cork is not a
// close. Track change is cork/flush/uncork in milliseconds; pause/play is
// the same pair. Closing or dropping on cork left the clock with no RUNNING
// hw_ptr, read_index stuck at 0, and Soloist reporting [0:00] forever.
//
// Cork freezes the clock and writes silence. The PCM stays open and
// RUNNING. Flush still resets the indices (new track). Volumio yield is a
// different event: the plugin creates APULSE_YIELD_PATH (default
// /data/soloist/alsa.yield) from unsetVolatile/stop. That is the only
// close. Uncork or a later write reacquires.
// ---------------------------------------------------------------------------

static void stream_schedule_acquire(pa_stream *s);

static const char *
yield_path(void)
{
    const char *e = getenv("APULSE_YIELD_PATH");

    if (e && e[0])
        return e;
    return "/data/soloist/alsa.yield";
}

static int
yield_requested(void)
{
    return access(yield_path(), F_OK) == 0;
}

static void
timeval_add_ms(struct timeval *tv, unsigned ms)
{
    tv->tv_usec += (long)ms * 1000;
    tv->tv_sec += tv->tv_usec / 1000000;
    tv->tv_usec %= 1000000;
}

static void
stream_cancel_time(pa_stream *s, pa_time_event **ev)
{
    pa_mainloop_api *api;

    if (!s || !ev || !*ev)
        return;
    api = s->c->mainloop_api;
    if (api && api->time_free)
        api->time_free(*ev);
    *ev = NULL;
}

static pa_time_event *
stream_after_ms(pa_stream *s, unsigned ms, pa_time_event_cb_t cb)
{
    pa_mainloop_api *api;
    struct timeval tv;

    if (!s || !s->c || !cb)
        return NULL;
    api = s->c->mainloop_api;
    if (!api || !api->time_new)
        return NULL;
    gettimeofday(&tv, NULL);
    timeval_add_ms(&tv, ms);
    return api->time_new(api, &tv, cb, s);
}

// Close the PCM but keep the playhead. The next open is a new hw_ptr
// generation; origin is rebuilt from the frozen usec on the first RUNNING
// sample so Soloist does not jump to 0:00 after a Volumio yield.
static void
stream_clock_hold(pa_stream *s)
{
    pa_usec_t held = s->clock_frozen_usec ? s->clock_frozen_usec
                                          : s->clock_last_played;
    int64_t bytes;

    s->clock_origin_hw = 0;
    s->clock_last_hw = -1;
    s->clock_have_origin = 0;
    s->clock_have_path = 0;
    s->clock_status_path[0] = '\0';
    s->clock_model_at.tv_sec = 0;
    s->clock_model_at.tv_usec = 0;
    s->clock_model_frames = 0;
    s->clock_model_rate = 0.0;
    s->clock_model_valid = 0;
    s->clock_running = 0;
    s->clock_frozen_usec = held;
    s->clock_last_played = held;

    bytes = (int64_t)pa_usec_to_bytes(held, &s->ss);
    if (bytes < 0)
        bytes = 0;
    s->timing_info.write_index = bytes;
    s->timing_info.read_index = bytes;
    s->timing_info.since_underrun = 0;
}

static void
stream_release_device(pa_stream *s, int keep_position)
{
    if (!s->ph)
        return;

    diag_logf("release device: rb=%zu keep_pos=%d",
              s->rb ? ringbuffer_readable_size(s->rb) : 0, keep_position);

    for (int k = 0; k < s->nioe; k++) {
        pa_mainloop_api *api = s->c->mainloop_api;

        api->io_free(s->ioe[k]);
    }
    free(s->ioe);
    s->ioe = NULL;
    s->nioe = 0;
    s->out_enabled = 0;

    snd_pcm_drop(s->ph);
    snd_pcm_close(s->ph);
    s->ph = NULL;

    if (s->rb)
        ringbuffer_drop(s->rb, ringbuffer_readable_size(s->rb));
    if (keep_position) {
        stream_clock_hold(s);
    } else {
        s->timing_info.write_index = 0;
        s->timing_info.read_index = 0;
        s->timing_info.since_underrun = 0;
        stream_clock_reset(s);
    }
}

static int
stream_maybe_yield(pa_stream *s)
{
    if (!s || !s->ph || !yield_requested())
        return 0;

    diag_logf("yield: releasing device");
    stream_cancel_time(s, &s->acquire_ev);
    s->acquire_attempts = 0;
    stream_clock_freeze(s);
    stream_release_device(s, 1);
    return 1;
}

static int
stream_acquire_device(pa_stream *s)
{
    if (s->ph)
        return 0;

    if (yield_requested())
        return -1;

    if (do_connect_pcm(s, SND_PCM_STREAM_PLAYBACK) < 0) {
        if (!s->acquire_failed) {
            s->acquire_failed = 1;
            diag_logf("reacquire device FAILED: device busy");
        }
        return -1;
    }
    s->acquire_failed = 0;
    s->acquire_attempts = 0;

    stream_set_output_enabled(s, 0);
    diag_logf("reacquired device");
    return 0;
}

static void
stream_request_write(pa_stream *s)
{
    size_t writable_size;

    if (!s || s->direction != PA_STREAM_PLAYBACK || !s->write_cb)
        return;
    writable_size = pa_stream_writable_size(s);
    if (writable_size > 0)
        s->write_cb(s, writable_size, s->write_cb_userdata);
}

static void
stream_become_running(pa_stream *s)
{
    stream_clock_start(s);
    g_atomic_int_set(&s->paused, 0);
    if (s->rb && ringbuffer_readable_size(s->rb) == 0)
        stream_request_write(s);
    stream_wake_output(s);
}

static void
stream_acquire_retry_cb(pa_mainloop_api *a, pa_time_event *e,
                        const struct timeval *tv, void *userdata)
{
    pa_stream *s = userdata;

    (void)tv;
    s->acquire_ev = NULL;
    if (a && a->time_free)
        a->time_free(e);
    if (!s->want_running)
        return;
    if (stream_acquire_device(s) == 0) {
        stream_become_running(s);
        return;
    }
    stream_schedule_acquire(s);
}

static void
stream_schedule_acquire(pa_stream *s)
{
    static const unsigned backoff_ms[] = {50, 100, 200, 400, 800};
    unsigned i;
    unsigned last;
    unsigned ms;

    if (yield_requested()) {
        stream_cancel_time(s, &s->acquire_ev);
        s->acquire_ev = stream_after_ms(s, 50, stream_acquire_retry_cb);
        return;
    }

    i = (unsigned)s->acquire_attempts;
    last = G_N_ELEMENTS(backoff_ms) - 1;
    ms = (i > last) ? backoff_ms[last] : backoff_ms[i];

    if (i <= last)
        s->acquire_attempts = (int)i + 1;
    if (i == last && !s->acquire_failed) {
        s->acquire_failed = 1;
        diag_logf("reacquire still busy, retrying");
    }
    stream_cancel_time(s, &s->acquire_ev);
    s->acquire_ev = stream_after_ms(s, ms, stream_acquire_retry_cb);
    if (!s->acquire_ev && !s->acquire_failed) {
        s->acquire_failed = 1;
        diag_logf("reacquire failed, no timer, staying corked");
    }
}

APULSE_EXPORT
int
pa_stream_connect_playback(pa_stream *s, const char *dev,
                           const pa_buffer_attr *attr, pa_stream_flags_t flags,
                           const pa_cvolume *volume, pa_stream *sync_stream)
{
    gchar *s_attr = trace_pa_buffer_attr_as_string(attr);
    trace_info_f(
        "P %s s=%p, dev=%s, attr=%s, flags=0x%x, volume=%p, sync_stream=%p\n",
        __func__, s, dev, s_attr, flags, volume, sync_stream);
    g_free(s_attr);

    s->direction = PA_STREAM_PLAYBACK;
    stream_adjust_buffer_attrs(s, attr);

    {
        int err = yield_requested()
                      ? -1
                      : do_connect_pcm(s, SND_PCM_STREAM_PLAYBACK);
        int start_corked = !!(flags & PA_STREAM_START_CORKED);

        g_atomic_int_set(&s->paused, start_corked);
        s->want_running = !start_corked;

        if (err < 0) {
            // The Pulse stream exists without a device. A real server does
            // the same when the sink is suspended. Volumio handover races
            // this open against MPD; failing the connect made Soloist retry
            // do_connect_pcm indefinitely and log EBUSY every time.
            g_atomic_int_set(&s->paused, 1);
            if (s->state != PA_STREAM_READY) {
                s->state = PA_STREAM_READY;
                pa_stream_ref(s);
                s->c->mainloop_api->defer_new(s->c->mainloop_api,
                                              deh_stream_state_changed, s);
                pa_stream_ref(s);
                s->c->mainloop_api->defer_new(
                    s->c->mainloop_api, deh_stream_first_readwrite_callback,
                    s);
            }
            if (!start_corked)
                stream_schedule_acquire(s);
            return 0;
        }
    }

    return 0;
}

static void
pa_stream_cork_impl(pa_operation *op)
{
    pa_stream *s = op->s;

    diag_logf("cork %d", op->int_arg_1 ? 1 : 0);
    if (op->int_arg_1) {
        stream_cancel_time(s, &s->acquire_ev);
        s->acquire_attempts = 0;
        s->want_running = 0;
        stream_clock_freeze(s);
        g_atomic_int_set(&s->paused, 1);
        if (!stream_maybe_yield(s))
            stream_wake_output(s);
    } else {
        s->want_running = 1;
        if (s->ph) {
            stream_clock_start(s);
            g_atomic_int_set(&s->paused, 0);
            stream_wake_output(s);
        } else if (stream_acquire_device(s) == 0) {
            stream_become_running(s);
        } else {
            stream_schedule_acquire(s);
        }
    }

    if (op->stream_success_cb)
        op->stream_success_cb(s, 1, op->cb_userdata);

    pa_operation_done(op);
}

APULSE_EXPORT
pa_operation *
pa_stream_cork(pa_stream *s, int b, pa_stream_success_cb_t cb, void *userdata)
{
    trace_info_f("F %s s=%p, b=%d, cb=%p, userdata=%p\n", __func__, s, b, cb,
                 userdata);

    pa_operation *op =
        pa_operation_new(s->c->mainloop_api, pa_stream_cork_impl);
    op->s = s;
    op->int_arg_1 = b;
    op->stream_success_cb = cb;
    op->cb_userdata = userdata;

    pa_operation_launch(op);
    return op;
}

APULSE_EXPORT
int
pa_stream_disconnect(pa_stream *s)
{
    trace_info_f("F %s s=%p\n", __func__, s);

    if (s->state != PA_STREAM_READY)
        return PA_ERR_BADSTATE;

    stream_cancel_time(s, &s->acquire_ev);

    // Logged before the close so the state ALSA is being left in is visible.
    // Upstream closes without dropping first, so a running stream with a full
    // buffer is torn down mid-flight; if a reconnect then misbehaves this line
    // and the next connect line are the pair to compare.
    if (diag_on()) {
        snd_pcm_sframes_t delay = 0;
        snd_pcm_state_t st = SND_PCM_STATE_DISCONNECTED;

        if (s->ph) {
            if (snd_pcm_delay(s->ph, &delay) < 0)
                delay = 0;
            st = snd_pcm_state(s->ph);
        }
        diag_logf("disconnect: alsa state=%s delay=%ld rb=%zu",
                  snd_pcm_state_name(st), (long)delay,
                  s->rb ? ringbuffer_readable_size(s->rb) : 0);
    }

    for (int k = 0; k < s->nioe; k++) {
        pa_mainloop_api *api = s->c->mainloop_api;
        api->io_free(s->ioe[k]);
    }
    free(s->ioe);
    s->ioe = NULL;
    s->nioe = 0;
    s->out_enabled = 0;

    // Upstream closed a RUNNING device with a full buffer. Observed on device:
    // "disconnect: alsa state=RUNNING delay=42512". Drop first so the device is
    // released idle and the next open starts from a known state.
    if (s->ph) {
        snd_pcm_drop(s->ph);
        snd_pcm_close(s->ph);
        s->ph = NULL;
    }
    if (s->rb)
        ringbuffer_drop(s->rb, ringbuffer_readable_size(s->rb));
    stream_clock_reset(s);
    // Same reasoning as the flush path: the indices describe audio that no
    // longer exists. A reconnect on this stream would otherwise start with a
    // fill level inherited from the previous session, and writable_size, being
    // bounded by tlength - fill, would report no room at all.
    s->timing_info.write_index = 0;
    s->timing_info.read_index = 0;
    s->timing_info.since_underrun = 0;
    s->state = PA_STREAM_TERMINATED;

    return PA_OK;
}

static void
pa_stream_drain_impl(pa_operation *op)
{
    snd_pcm_drain(op->s->ph);

    if (op->stream_success_cb)
        op->stream_success_cb(op->s, 1, op->cb_userdata);

    pa_operation_done(op);
}

APULSE_EXPORT
pa_operation *
pa_stream_drain(pa_stream *s, pa_stream_success_cb_t cb, void *userdata)
{
    trace_info_f("F %s s=%p, cb=%p, userdata=%p\n", __func__, s, cb, userdata);

    pa_operation *op =
        pa_operation_new(s->c->mainloop_api, pa_stream_drain_impl);
    op->s = s;
    op->stream_success_cb = cb;
    op->cb_userdata = userdata;

    pa_operation_launch(op);
    return op;
}

static void
pa_stream_flush_impl(pa_operation *op)
{
    pa_stream *s = op->s;
    size_t queued = s->rb ? ringbuffer_readable_size(s->rb) : 0;

    diag_logf("flush: discarding rb=%zu", queued);

    // Upstream did nothing here and returned success, so a skip or a seek left
    // the whole ring in place. Observed on device: Soloist corks, flushes and
    // uncorks at every track change, and the flush found rb=73728 -- the ring
    // completely full -- each time. The new track then had nowhere to go,
    // pa_stream_writable_size returned 0, and playback stalled against stale
    // audio. That is the choppiness after a quality change.
    //
    // Discard both stages: our ring, and whatever ALSA has already accepted.
    if (s->rb && queued > 0)
        ringbuffer_drop(s->rb, queued);

    // The position restarts with the audio it was measuring.
    stream_clock_reset(s);

    // Both indices describe the audio that was just discarded, so they reset
    // with it. Leaving write_index while read_index restarts makes the fill
    // level look enormous, and since writable_size is bounded by
    // tlength - fill, it then reports zero room forever and the client can
    // never write the next track. That is a track change that appears to hang.
    s->timing_info.write_index = 0;
    s->timing_info.read_index = 0;
    s->timing_info.since_underrun = 0;

    if (s->ph) {
        // snd_pcm_drop leaves the device in SETUP, where snd_pcm_avail returns
        // -EBADFD and the io callback treats the stream as closed. Prepare
        // immediately, in the same operation, so the callback never observes
        // that state.
        snd_pcm_drop(s->ph);
        snd_pcm_prepare(s->ph);
    }

    // The ring is empty by definition now, so leave output disabled until the
    // client writes; that is also what stops a post-flush spin.
    stream_set_output_enabled(s, 0);

    if (op->stream_success_cb)
        op->stream_success_cb(op->s, 1, op->cb_userdata);

    pa_operation_done(op);
}

APULSE_EXPORT
pa_operation *
pa_stream_flush(pa_stream *s, pa_stream_success_cb_t cb, void *userdata)
{
    trace_info_f("F %s s=%p, cb=%p, userdata=%p\n", __func__, s, cb, userdata);

    pa_operation *op =
        pa_operation_new(s->c->mainloop_api, pa_stream_flush_impl);
    op->s = s;
    op->stream_success_cb = cb;
    op->cb_userdata = userdata;

    pa_operation_launch(op);
    return op;
}

APULSE_EXPORT
uint32_t
pa_stream_get_index(pa_stream *s)
{
    trace_info_f("F %s s=%p\n", __func__, s);

    return s->idx;
}

// volumioswitch reports delay = local + target. That figure can sit at
// the ioplug maximum (65536 frames, ~1.48 s at 44100) even when the
// client asked for a few hundred milliseconds. Soloist uses Pulse
// latency to steer playback speed. A 1.5 s report makes it rush, then
// stall, then chop.
//
// Cap the number we hand back to the requested tlength, or to
// APULSE_MAX_TLENGTH_MS if that is set and smaller. Does not change
// ALSA rate, format, or buffer negotiation.
static snd_pcm_sframes_t
stream_reported_delay(pa_stream *s)
{
    snd_pcm_sframes_t delay = 0;
    const size_t frame_size = pa_frame_size(&s->ss);
    snd_pcm_sframes_t cap = 0;

    if (s->ph && snd_pcm_delay(s->ph, &delay) < 0)
        delay = 0;
    if (delay < 0)
        delay = 0;

    if (frame_size > 0 && s->buffer_attr.tlength > 0 &&
        s->buffer_attr.tlength != (uint32_t)-1)
        cap = (snd_pcm_sframes_t)(s->buffer_attr.tlength / frame_size);

    {
        const char *max_tlength_ms = getenv("APULSE_MAX_TLENGTH_MS");
        long ms = max_tlength_ms ? strtol(max_tlength_ms, NULL, 10) : 0;

        if (ms > 0 && s->ss.rate > 0) {
            snd_pcm_sframes_t env_cap =
                (snd_pcm_sframes_t)((int64_t)s->ss.rate * ms / 1000);

            if (env_cap > 0 && (cap <= 0 || env_cap < cap))
                cap = env_cap;
        }
    }

    if (cap > 0 && delay > cap)
        delay = cap;

    return delay;
}

APULSE_EXPORT
int
pa_stream_get_latency(pa_stream *s, pa_usec_t *r_usec, int *negative)
{
    trace_info_f("F %s s=%p\n", __func__, s);

    snd_pcm_sframes_t delay = stream_reported_delay(s);

    if (r_usec) {
        size_t frame_size = pa_frame_size(&s->ss);

        *r_usec = frame_size > 0
                      ? pa_bytes_to_usec((uint64_t)delay * frame_size, &s->ss)
                      : 0;
    }
    if (negative)
        *negative = 0;
    return 0;
}

APULSE_EXPORT
const pa_sample_spec *
pa_stream_get_sample_spec(pa_stream *s)
{
    trace_info_f("F %s s=%p\n", __func__, s);

    return &s->ss;
}

APULSE_EXPORT
pa_stream_state_t
pa_stream_get_state(pa_stream *s)
{
    trace_info_f("F %s s=%p\n", __func__, s);

    return s->state;
}

APULSE_EXPORT
int
pa_stream_get_time(pa_stream *s, pa_usec_t *r_usec)
{
    trace_info_f("F %s\n", __func__);

    // Hardware consumption, not write_index. Returning the write index made a
    // burst write jump the reported position by the duration written, which is
    // not a sink clock and is what Soloist steers from.
    if (r_usec)
        *r_usec = stream_hw_time(s);
    return 0;
}

// Observer for the four symbols Soloist actually resolves. The question this
// answers is whether it interpolates: if it reads timing far more often than it
// writes, it is estimating position between updates from timing_info.timestamp,
// and the freshness of that timestamp matters more than read_index. If reads and
// writes are one to one, it is not, and read_index is what has to be right.
//
// Four fixes have now been aimed at this symptom by inferring what Soloist does
// with pa_timing_info. This logs what it is actually given.
static struct {
    struct timeval t0;
    struct timeval last_read;
    unsigned reads;        // pa_stream_get_timing_info
    unsigned updates;      // pa_stream_update_timing_info
    unsigned wsize;        // pa_stream_writable_size
    unsigned writes;       // pa_stream_write
    long write_bytes;
    long wsize_zero;       // writable_size answers of 0: backpressure
    long gap_min_us;
    long gap_max_us;
    long gap_sum_us;
    unsigned gap_n;
} tstat;

static void
diag_timing_read(pa_stream *s, const char *how)
{
    struct timeval now;
    long gap = -1;

    if (!diag_on())
        return;

    gettimeofday(&now, NULL);
    if (tstat.last_read.tv_sec) {
        gap = (now.tv_sec - tstat.last_read.tv_sec) * 1000000L +
              (now.tv_usec - tstat.last_read.tv_usec);
        if (tstat.gap_n == 0 || gap < tstat.gap_min_us)
            tstat.gap_min_us = gap;
        if (gap > tstat.gap_max_us)
            tstat.gap_max_us = gap;
        tstat.gap_sum_us += gap;
        tstat.gap_n++;
    }
    tstat.last_read = now;

    // Every field of the struct, because which ones the client uses is exactly
    // what is unknown. One line in ten by default to keep the journal readable;
    // APULSE_DIAG=2 logs every read, which is what a trace capture needs.
    if (diag_full() || (tstat.reads + tstat.updates) % 10 == 0)
        diag_logf("timing[%s] gap=%ldus w=%lld r=%lld fill=%lld "
                  "sink=%llu transport=%llu cfg_sink=%llu since_underrun=%llu "
                  "playing=%d ts=%ld.%06ld",
                  how, gap,
                  (long long)s->timing_info.write_index,
                  (long long)s->timing_info.read_index,
                  (long long)(s->timing_info.write_index -
                              s->timing_info.read_index),
                  (unsigned long long)s->timing_info.sink_usec,
                  (unsigned long long)s->timing_info.transport_usec,
                  (unsigned long long)s->timing_info.configured_sink_usec,
                  (unsigned long long)s->timing_info.since_underrun,
                  s->timing_info.playing,
                  (long)s->timing_info.timestamp.tv_sec,
                  (long)s->timing_info.timestamp.tv_usec);
}

static void
diag_timing_tick(void)
{
    struct timeval now;
    long ms;

    if (!diag_on())
        return;

    gettimeofday(&now, NULL);
    if (tstat.t0.tv_sec == 0) {
        tstat.t0 = now;
        return;
    }
    ms = (now.tv_sec - tstat.t0.tv_sec) * 1000 +
         (now.tv_usec - tstat.t0.tv_usec) / 1000;
    if (ms < 1000)
        return;

    diag_logf("1s api reads=%u updates=%u wsize=%u (zero=%ld) writes=%u "
              "wbytes=%ld gap min/avg/max=%ld/%ld/%ld us",
              tstat.reads, tstat.updates, tstat.wsize, tstat.wsize_zero,
              tstat.writes, tstat.write_bytes,
              tstat.gap_n ? tstat.gap_min_us : 0,
              tstat.gap_n ? tstat.gap_sum_us / (long)tstat.gap_n : 0,
              tstat.gap_n ? tstat.gap_max_us : 0);

    {
        struct timeval keep = now;
        struct timeval last = tstat.last_read;

        memset(&tstat, 0, sizeof(tstat));
        tstat.t0 = keep;
        tstat.last_read = last;
    }
}

// Soloist reads only pa_timing_info. Confirmed from its dynamic symbols: it
// resolves pa_stream_get_timing_info, pa_stream_update_timing_info,
// pa_stream_writable_size and pa_stream_write, and neither pa_stream_get_time
// nor pa_stream_get_latency. So this struct is the entire timing contract, and
// it must be self-consistent on its own.
//
// Two counters and one clock, all describing the same audio:
//
//   write_index  bytes the client handed us          (pa_stream_write)
//   read_index   bytes the DAC has actually played   (hw_ptr)
//   fill level   write_index - read_index            (everything in flight)
//
// read_index was previously write_index minus snd_pcm_delay, which through
// volumioswitch is local + target: two stages against a buffer sized for one.
// Measured 43654 frames against a negotiated 22050. Deriving the fill level
// from that made Soloist believe it was roughly twice as far ahead as it was,
// so it slowed, overshot, and chopped.
static void
stream_update_timing(pa_stream *s)
{
    const size_t frame_size = pa_frame_size(&s->ss);
    pa_usec_t hw_usec;
    int64_t played;

    stream_maybe_yield(s);
    gettimeofday(&s->timing_info.timestamp, NULL);

    hw_usec = stream_hw_time(s);
    played = (int64_t)pa_usec_to_bytes(hw_usec, &s->ss);
    if (frame_size > 0)
        played -= played % (int64_t)frame_size;

    // read_index is monotonic within a stream generation. The hardware never
    // un-plays audio, so a lower answer means our clock lost its position, not
    // that the DAC went backwards: before the first write and while corked,
    // stream_hw_time legitimately returns 0.
    //
    // Reporting that 0 was a real defect. Observed on device:
    //
    //   r=14013064  fill=236168     playing=1
    //   r=0         fill=14249232   playing=0
    //
    // The fill level jumped from 236 KB to 14 MB in 29 us. Soloist polls
    // timing about ten times per write and interpolates between polls, so a
    // step of that size is not a glitch it can ignore; it is a backlog
    // fourteen million bytes deep appearing between two reads.
    //
    // Hold the previous value instead. A position that stops advancing is
    // honest about a clock that has stopped; a position that collapses to zero
    // is not.
    //
    // A flush ends the generation: it discards the audio both indices were
    // measuring and zeroes them, so there is nothing to be monotonic against
    // and the hold must not apply. Without that exception read_index stays
    // pinned at the old value, gets clamped down to the freshly zeroed
    // write_index, and the fill level is meaningless until the clock catches
    // up.
    if (s->timing_info.write_index > 0 &&
        played < s->timing_info.read_index)
        played = s->timing_info.read_index;

    // The hardware cannot have played more than the client wrote.
    if (played > s->timing_info.write_index)
        played = s->timing_info.write_index;
    if (played < 0)
        played = 0;

    s->timing_info.read_index = played;
    s->timing_info.read_index_corrupt = 0;
    s->timing_info.write_index_corrupt = 0;
    s->timing_info.playing = !g_atomic_int_get(&s->paused);

    // read_index is already hardware consumption, so nothing further sits
    // between the sink and the speaker. Reporting a separate sink latency here
    // would double-count the same audio.
    s->timing_info.sink_usec = 0;
    s->timing_info.transport_usec = 0;

    // Describe the pipeline being fed. This was left at 0, so the client had no
    // idea what buffer it was writing into. configured_sink_usec is set from
    // the ALSA buffer actually opened, in do_connect_pcm.
    if (s->configured_sink_usec)
        s->timing_info.configured_sink_usec = s->configured_sink_usec;
}

APULSE_EXPORT
const pa_timing_info *
pa_stream_get_timing_info(pa_stream *s)
{
    trace_info_f("F %s s=%p\n", __func__, s);

    stream_update_timing(s);
    tstat.reads++;
    diag_timing_read(s, "get");
    diag_timing_tick();

    return &s->timing_info;
}

APULSE_EXPORT
int
pa_stream_is_corked(pa_stream *s)
{
    trace_info_f("F %s s=%p\n", __func__, s);
    return g_atomic_int_get(&s->paused);
}

APULSE_EXPORT
int
pa_stream_is_suspended(pa_stream *s)
{
    trace_info_f("F %s s=%p\n", __func__, s);
    // ALSA sink is never suspended
    return 0;
}

APULSE_EXPORT
pa_stream *
pa_stream_new(pa_context *c, const char *name, const pa_sample_spec *ss,
              const pa_channel_map *map)
{
    gchar *s_map = trace_pa_channel_map_as_string(map);
    gchar *s_ss = trace_pa_sample_spec_as_string(ss);
    trace_info_f("F %s c=%p, name=%s, ss=%s, map=%s\n", __func__, c, name, s_ss,
                 s_map);
    g_free(s_ss);
    g_free(s_map);

    pa_proplist *p = pa_proplist_new();
    pa_stream *s = pa_stream_new_with_proplist(c, name, ss, map, p);
    pa_proplist_free(p);
    return s;
}

APULSE_EXPORT
pa_stream *
pa_stream_new_extended(pa_context *c, const char *name,
                       pa_format_info *const *formats, unsigned int n_formats,
                       pa_proplist *p)
{
    trace_info_f("P %s c=%p, name=%s, formats=%p, n_formats=%u, p=%p\n",
                 __func__, c, name, formats, n_formats, p);

    // TODO: multiple formats?

    // take first format
    if (n_formats < 1) {
        trace_error("%s, no formats\n", __func__);
        return NULL;
    }

    pa_sample_spec ss = {
        .format = PA_SAMPLE_S16LE, .rate = 48000, .channels = 2,
    };

    const char *val;

    val = pa_proplist_gets(formats[0]->plist, PA_PROP_FORMAT_SAMPLE_FORMAT);
    if (val)
        ss.format = pa_sample_format_from_string(val);

    val = pa_proplist_gets(formats[0]->plist, PA_PROP_FORMAT_RATE);
    if (val)
        ss.rate = atoi(val);

    val = pa_proplist_gets(formats[0]->plist, PA_PROP_FORMAT_CHANNELS);
    if (val)
        ss.channels = atoi(val);

    return pa_stream_new_with_proplist(c, name, &ss, NULL, p);
}

APULSE_EXPORT
pa_stream *
pa_stream_new_with_proplist(pa_context *c, const char *name,
                            const pa_sample_spec *ss, const pa_channel_map *map,
                            pa_proplist *p)
{
    gchar *s_map = trace_pa_channel_map_as_string(map);
    gchar *s_ss = trace_pa_sample_spec_as_string(ss);
    trace_info_f("F %s c=%p, name=%s, ss=%s, map=%s, p=%p\n", __func__, c, name,
                 s_ss, s_map, p);
    g_free(s_ss);
    g_free(s_map);

    pa_stream *s = calloc(1, sizeof(pa_stream));
    s->c = c;
    s->ref_cnt = 1;
    s->state = PA_STREAM_UNCONNECTED;
    s->ss = *ss;

    // The stream dereferences s->c throughout its lifetime, including in
    // pa_stream_unref after the client may already have dropped its own
    // reference. Own the context rather than borrowing it.
    pa_context_ref(c);

    s->idx = c->next_stream_idx++;
    g_hash_table_insert(c->streams_ht, GINT_TO_POINTER(s->idx), s);

    stream_clock_reset(s);

    diag_logf("new stream idx=%u: spec %s %u Hz %u ch frame=%zu", s->idx,
              diag_fmt_name(s->ss.format), s->ss.rate, s->ss.channels,
              pa_frame_size(&s->ss));

    // fill initial values of s->timing_info
    gettimeofday(&s->timing_info.timestamp, NULL);
    s->timing_info.synchronized_clocks = 1;
    s->timing_info.sink_usec = 0;
    s->timing_info.source_usec = 0;
    s->timing_info.transport_usec = 0;
    s->timing_info.playing = 1;
    s->timing_info.write_index_corrupt = 0;
    s->timing_info.write_index = 0;
    s->timing_info.read_index_corrupt = 0;
    s->timing_info.read_index = 0;
    s->timing_info.configured_sink_usec = 0;
    s->timing_info.configured_source_usec = 0;
    s->timing_info.since_underrun = 0;

    // Size the ring in TIME, not bytes.
    //
    // 72 KiB is 418 ms of S16 stereo but only 209 ms of FLOAT32 stereo. Soloist
    // decodes lossy to S16 and lossless to FLOAT32, so the same constant gives
    // lossless half the buffer, and it writes 93 ms per call rather than 46.
    // That is why the fault is lossless-only.
    //
    // Sized from tlength, which is what the client was told the target is, with
    // headroom for one write on top. Floored at the historic 72 KiB.
    {
        size_t fs = pa_frame_size(&s->ss);
        size_t rb_bytes = 72 * 1024;

        if (fs > 0 && s->ss.rate > 0) {
            // 500 ms at the client's own frame size.
            rb_bytes = fs * (size_t)((uint64_t)s->ss.rate / 2);
            if (rb_bytes < 72 * 1024)
                rb_bytes = 72 * 1024;
        }
        s->rb = ringbuffer_new(rb_bytes);
    }
    s->peek_buffer = malloc(s->rb->end - s->rb->start);

    for (uint32_t k = 0; k < PA_CHANNELS_MAX; k++)
        s->volume[k] = PA_VOLUME_NORM;

    return s;
}

APULSE_EXPORT
pa_stream *
pa_stream_ref(pa_stream *s)
{
    trace_info_f("F %s s=%p\n", __func__, s);

    s->ref_cnt++;
    return s;
}

APULSE_EXPORT
void
pa_stream_set_latency_update_callback(pa_stream *s, pa_stream_notify_cb_t cb,
                                      void *userdata)
{
    trace_info_f("F %s s=%p, cb=%p, userdata=%p\n", __func__, s, cb, userdata);

    s->latency_update_cb = cb;
    s->latency_update_cb_userdata = userdata;
}

static void
pa_stream_set_name_impl(pa_operation *op)
{
    free(op->s->name);
    op->s->name = op->char_ptr_arg_1;

    if (op->stream_success_cb)
        op->stream_success_cb(op->s, 1, op->cb_userdata);

    pa_operation_done(op);
}

APULSE_EXPORT
pa_operation *
pa_stream_set_name(pa_stream *s, const char *name, pa_stream_success_cb_t cb,
                   void *userdata)
{
    trace_info_f("P %s s=%p, name=%s, cb=%p, userdata=%p\n", __func__, s, name,
                 cb, userdata);

    pa_operation *op =
        pa_operation_new(s->c->mainloop_api, pa_stream_set_name_impl);
    op->s = s;
    op->stream_success_cb = cb;
    op->cb_userdata = userdata;
    op->char_ptr_arg_1 = strdup(name ? name : "");

    pa_operation_launch(op);
    return op;
}

APULSE_EXPORT
void
pa_stream_set_state_callback(pa_stream *s, pa_stream_notify_cb_t cb,
                             void *userdata)
{
    trace_info_f("F %s s=%p, cb=%p, userdata=%p\n", __func__, s, cb, userdata);

    s->state_cb = cb;
    s->state_cb_userdata = userdata;
}

APULSE_EXPORT
void
pa_stream_set_write_callback(pa_stream *s, pa_stream_request_cb_t cb,
                             void *userdata)
{
    trace_info_f("F %s s=%p, cb=%p, userdata=%p\n", __func__, s, cb, userdata);

    s->write_cb = cb;
    s->write_cb_userdata = userdata;
}

static void
pa_stream_trigger_impl(pa_operation *op)
{
    // TODO: does nothing?

    if (op->stream_success_cb)
        op->stream_success_cb(op->s, 1, op->cb_userdata);

    pa_operation_done(op);
}

APULSE_EXPORT
pa_operation *
pa_stream_trigger(pa_stream *s, pa_stream_success_cb_t cb, void *userdata)
{
    trace_info_f("F %s s=%p, cb=%p, userdata=%p\n", __func__, s, cb, userdata);

    pa_operation *op =
        pa_operation_new(s->c->mainloop_api, pa_stream_trigger_impl);
    op->s = s;
    op->stream_success_cb = cb;
    op->cb_userdata = userdata;

    pa_operation_launch(op);
    return op;
}

APULSE_EXPORT
void
pa_stream_unref(pa_stream *s)
{
    trace_info_f("F %s s=%p\n", __func__, s);

    s->ref_cnt--;
    if (s->ref_cnt == 0) {
        pa_context *c = s->c;

        stream_cancel_time(s, &s->acquire_ev);
        g_hash_table_remove(s->c->streams_ht, GINT_TO_POINTER(s->idx));
        ringbuffer_free(s->rb);
        free(s->peek_buffer);
        free(s->write_buffer);
        free(s->name);
        free(s);

        // Released only after the stream has removed itself from the table, so
        // the table is still alive above even when this drops the last
        // reference and frees the context.
        pa_context_unref(c);
    }
}

static void
pa_stream_update_timing_info_impl(pa_operation *op)
{
    // The client asked for fresh timing; give it the same data
    // pa_stream_get_timing_info would, not just a new timestamp.
    stream_update_timing(op->s);
    tstat.updates++;
    diag_timing_read(op->s, "upd");
    diag_timing_tick();

    if (op->s->latency_update_cb)
        op->s->latency_update_cb(op->s, op->s->latency_update_cb_userdata);

    if (op->stream_success_cb)
        op->stream_success_cb(op->s, 1, op->cb_userdata);

    pa_operation_done(op);
}

APULSE_EXPORT
pa_operation *
pa_stream_update_timing_info(pa_stream *s, pa_stream_success_cb_t cb,
                             void *userdata)
{
    trace_info_f("F %s s=%p, cb=%p, userdata=%p\n", __func__, s, cb, userdata);

    pa_operation *op =
        pa_operation_new(s->c->mainloop_api, pa_stream_update_timing_info_impl);
    op->s = s;
    op->stream_success_cb = cb;
    op->cb_userdata = userdata;

    pa_operation_launch(op);
    return op;
}

APULSE_EXPORT
size_t
pa_stream_writable_size(pa_stream *s)
{
    trace_info_f("F %s s=%p\n", __func__, s);

    size_t writable_size = ringbuffer_writable_size(s->rb);

    // Some applications try to push more data than reported to be available
    // by pa_stream_writable_size(), which is fine for original PulseAudio
    // but is a severe error in this implementation, since buffer size is
    // limited.
    //
    // Workaround issue by reserving certain amount for that case.

    const size_t limit = 16 * 1024;  // TODO: adaptive values?

    if (writable_size < limit)
        writable_size = 0;

    // Bound by tlength, as a real PulseAudio server does.
    //
    // Without this, writable_size reports the whole ring and the client keeps
    // writing until the ring is full rather than until the target is met.
    // Measured on device at lossless: Soloist supplied 1.234x realtime, writing
    // 93 ms of audio every 98 ms at the median but 43% of writes arriving
    // faster than that, 4% under 10 ms apart. It filled the ring, hit a zero
    // answer, stalled, then burst again. That alternation is the speeding up
    // and slowing down.
    //
    // tlength is the target the client was told about, so it is the level to
    // hold. Room = target minus what is already queued.
    if (s->buffer_attr.tlength > 0 &&
        s->buffer_attr.tlength != (uint32_t)-1) {
        int64_t fill = s->timing_info.write_index - s->timing_info.read_index;
        int64_t room;

        if (fill < 0)
            fill = 0;
        room = (int64_t)s->buffer_attr.tlength - fill;
        if (room < 0)
            room = 0;
        if ((size_t)room < writable_size)
            writable_size = (size_t)room;
    }

    {
        size_t out = pa_find_multiple_of(writable_size, pa_frame_size(&s->ss), 0);

        // A zero answer is backpressure: the client wanted to write and we told
        // it there was no room. Frequent zeros mean the ring is the constraint,
        // not the client.
        tstat.wsize++;
        if (out == 0)
            tstat.wsize_zero++;
        return out;
    }
}

APULSE_EXPORT
size_t
pa_stream_readable_size(pa_stream *s)
{
    trace_info_f("F %s s=%p\n", __func__, s);

    size_t readable_size = ringbuffer_readable_size(s->rb);
    return pa_find_multiple_of(readable_size, pa_frame_size(&s->ss), 0);
}

APULSE_EXPORT
int
pa_stream_write(pa_stream *s, const void *data, size_t nbytes,
                pa_free_cb_t free_cb, int64_t offset, pa_seek_mode_t seek)
{
    trace_info_f("F %s s=%p, data=%p, nbytes=%zu, free_cb=%p, offset=%" PRId64
                 ", seek=%u\n",
                 __func__, s, data, nbytes, free_cb, offset, seek);

    if (offset != 0)
        trace_error("%s, offset != 0\n", __func__);
    if (seek != PA_SEEK_RELATIVE)
        trace_error("%s, seek != PA_SEEK_RELATIVE\n", __func__);

    size_t written = ringbuffer_write(s->rb, data, nbytes);
    s->timing_info.since_underrun += written;
    dstat.client_bytes += (long)written;
    s->timing_info.write_index += written;
    tstat.writes++;
    tstat.write_bytes += (long)written;

    stream_maybe_yield(s);

    // There is something to send again. Flow control dropped POLLOUT when the
    // ring ran dry; restore it now rather than waiting for a timer that does
    // not exist.
    if (written > 0) {
        if (s->ph) {
            stream_wake_output(s);
        } else if (s->want_running) {
            if (stream_acquire_device(s) == 0)
                stream_become_running(s);
            else
                stream_schedule_acquire(s);
        }
    }

    if (data == s->write_buffer) {
        free(s->write_buffer);
        s->write_buffer = NULL;
    } else {
        if (free_cb)
            free_cb((void *)data);
    }

    return 0;
}

APULSE_EXPORT
int
pa_stream_connect_record(pa_stream *s, const char *dev,
                         const pa_buffer_attr *attr, pa_stream_flags_t flags)
{
    gchar *s_attr = trace_pa_buffer_attr_as_string(attr);
    trace_info_f("P %s s=%p, dev=%s, attr=%s, flags=0x%x\n", __func__, s, dev,
                 s_attr, flags);
    g_free(s_attr);

    s->direction = PA_STREAM_RECORD;
    stream_adjust_buffer_attrs(s, attr);

    if (do_connect_pcm(s, SND_PCM_STREAM_CAPTURE) < 0)
        goto err;

    snd_pcm_start(s->ph);

    return 0;
err:
    return -1;
}

APULSE_EXPORT
int
pa_stream_drop(pa_stream *s)
{
    trace_info_f("F %s s=%p\n", __func__, s);

    ringbuffer_drop(s->rb, s->peek_buffer_data_len);
    return 0;
}

APULSE_EXPORT
const pa_buffer_attr *
pa_stream_get_buffer_attr(pa_stream *s)
{
    trace_info_f("F %s\n", __func__);

    return &s->buffer_attr;
}

APULSE_EXPORT
uint32_t
pa_stream_get_device_index(pa_stream *s)
{
    trace_info_f("F %s s=%p\n", __func__, s);

    // apulse uses only one sink -- ALSA device, so index is always 0
    return 0;
}

APULSE_EXPORT
const char *
pa_stream_get_device_name(pa_stream *s)
{
    trace_info_f("F %s s=%p\n", __func__, s);
    return "apulse";
}

APULSE_EXPORT
int
pa_stream_peek(pa_stream *s, const void **data, size_t *nbytes)
{
    trace_info_f("F %s s=%p\n", __func__, s);

    if (!s)
        return -1;

    size_t len = ringbuffer_readable_size(s->rb);
    s->peek_buffer_data_len = ringbuffer_peek(s->rb, s->peek_buffer, len);

    if (nbytes)
        *nbytes = s->peek_buffer_data_len;
    if (data)
        *data = s->peek_buffer;
    return 0;
}

APULSE_EXPORT
void
pa_stream_set_read_callback(pa_stream *s, pa_stream_request_cb_t cb,
                            void *userdata)
{
    trace_info_f("F %s s=%p, cb=%p, userdata=%p\n", __func__, s, cb, userdata);

    if (s) {
        s->read_cb = cb;
        s->read_cb_userdata = userdata;
    }
}

APULSE_EXPORT
void
pa_stream_set_underflow_callback(pa_stream *p, pa_stream_notify_cb_t cb,
                                 void *userdata)
{
    trace_info_z("Z %s\n", __func__);
}

APULSE_EXPORT
pa_context *
pa_stream_get_context(pa_stream *p)
{
    trace_info_z("Z %s\n", __func__);
    return NULL;
}

APULSE_EXPORT
void
pa_stream_set_overflow_callback(pa_stream *p, pa_stream_notify_cb_t cb,
                                void *userdata)
{
    trace_info_z("Z %s\n", __func__);
}

APULSE_EXPORT
int64_t
pa_stream_get_underflow_index(pa_stream *p)
{
    trace_info_z("Z %s\n", __func__);
    return 0;
}

APULSE_EXPORT
void
pa_stream_set_started_callback(pa_stream *p, pa_stream_notify_cb_t cb,
                               void *userdata)
{
    trace_info_z("Z %s\n", __func__);
}

APULSE_EXPORT
void
pa_stream_set_moved_callback(pa_stream *p, pa_stream_notify_cb_t cb,
                             void *userdata)
{
    trace_info_z("Z %s\n", __func__);
}

APULSE_EXPORT
void
pa_stream_set_suspended_callback(pa_stream *p, pa_stream_notify_cb_t cb,
                                 void *userdata)
{
    trace_info_z("Z %s\n", __func__);
}

APULSE_EXPORT
void
pa_stream_set_event_callback(pa_stream *p, pa_stream_event_cb_t cb,
                             void *userdata)
{
    trace_info_z("Z %s\n", __func__);
}

APULSE_EXPORT
void
pa_stream_set_buffer_attr_callback(pa_stream *p, pa_stream_notify_cb_t cb,
                                   void *userdata)
{
    trace_info_z("Z %s\n", __func__);
}

APULSE_EXPORT
pa_operation *
pa_stream_prebuf(pa_stream *s, pa_stream_success_cb_t cb, void *userdata)
{
    trace_info_z("Z %s\n", __func__);
    return NULL;
}

APULSE_EXPORT
const pa_channel_map *
pa_stream_get_channel_map(pa_stream *s)
{
    trace_info_z("Z %s\n", __func__);
    return NULL;
}

APULSE_EXPORT
const pa_format_info *
pa_stream_get_format_info(pa_stream *s)
{
    trace_info_z("Z %s\n", __func__);
    return NULL;
}

APULSE_EXPORT
pa_operation *
pa_stream_set_buffer_attr(pa_stream *s, const pa_buffer_attr *attr,
                          pa_stream_success_cb_t cb, void *userdata)
{
    trace_info_z("Z %s\n", __func__);
    return NULL;
}

APULSE_EXPORT
pa_operation *
pa_stream_update_sample_rate(pa_stream *s, uint32_t rate,
                             pa_stream_success_cb_t cb, void *userdata)
{
    trace_info_z("Z %s\n", __func__);
    return NULL;
}

APULSE_EXPORT
pa_operation *
pa_stream_proplist_update(pa_stream *s, pa_update_mode_t mode, pa_proplist *p,
                          pa_stream_success_cb_t cb, void *userdata)
{
    trace_info_z("Z %s\n", __func__);
    return NULL;
}

APULSE_EXPORT
pa_operation *
pa_stream_proplist_remove(pa_stream *s, const char *const keys[],
                          pa_stream_success_cb_t cb, void *userdata)
{
    trace_info_z("Z %s\n", __func__);
    return NULL;
}

APULSE_EXPORT
int
pa_stream_set_monitor_stream(pa_stream *s, uint32_t sink_input_idx)
{
    trace_info_z("Z %s\n", __func__);
    return 0;
}

APULSE_EXPORT
uint32_t
pa_stream_get_monitor_stream(pa_stream *s)
{
    trace_info_z("Z %s\n", __func__);
    return 0;
}

APULSE_EXPORT
int
pa_stream_connect_upload(pa_stream *s, size_t length)
{
    trace_info_z("Z %s\n", __func__);
    return 0;
}

APULSE_EXPORT
int
pa_stream_finish_upload(pa_stream *s)
{
    trace_info_z("Z %s\n", __func__);
    return 0;
}
