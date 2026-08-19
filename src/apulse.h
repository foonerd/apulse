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

#pragma once

#define _GNU_SOURCE
#include "ringbuffer.h"
#include <alsa/asoundlib.h>
#include <glib.h>
#include <poll.h>
#include <pthread.h>
#include <pulse/pulseaudio.h>

#define APULSE_EXPORT __attribute__((visibility("default")))

struct pa_context {
    pa_context_state_t state;
    pa_context_state_t new_state;
    pa_context_notify_cb_t state_cb;
    void *state_cb_userdata;
    pa_mainloop_api *mainloop_api;
    char *name;
    int ref_cnt;
    int next_stream_idx;
    GHashTable *streams_ht;
    pa_volume_t source_volume[PA_CHANNELS_MAX];
};

struct pa_io_event {
    int fd;
    pa_io_event_flags_t events;
    pa_io_event_cb_t cb;
    void *cb_userdata;
    pa_io_event_destroy_cb_t destroy_cb;
    pa_mainloop *mainloop;
    struct pollfd *pollfd;
    snd_pcm_t *pcm;
};

struct pa_mainloop {
    pa_mainloop_api api;
    GQueue *deferred_events_queue;
    GQueue *timed_events_queue;
    GHashTable *events_ht;  ///< a set of (pa_io_event *)
    struct pollfd *fds;
    nfds_t nfds;
    int recreate_fds;  ///< 1 if fds array needs to be recreated from events_ht
    int timeout;
    int wakeup_pipe[2];
    int terminate;
    int retval;
    pa_poll_func poll_func;
    void *poll_func_userdata;
    int alsa_special_cnt;
};

struct pa_glib_mainloop {
    pa_mainloop_api api;
};

struct pa_threaded_mainloop {
    pa_mainloop *m;
    pthread_mutex_t lock;
    pthread_cond_t cond;
    pthread_t t;
    int running;
};

struct pa_proplist {
    GHashTable *ht;
};

struct pa_stream {
    pa_context *c;
    pa_stream_state_t state;
    pa_stream_direction_t direction;
    snd_pcm_t *ph;
    pa_sample_spec ss;
    pa_buffer_attr buffer_attr;
    int ref_cnt;
    int idx;
    pa_stream_notify_cb_t state_cb;
    void *state_cb_userdata;
    pa_stream_request_cb_t write_cb;
    void *write_cb_userdata;
    pa_stream_request_cb_t read_cb;
    void *read_cb_userdata;
    pa_stream_notify_cb_t latency_update_cb;
    void *latency_update_cb_userdata;
    char *name;
    pa_timing_info timing_info;
    pa_io_event **ioe;  ///< list of pa_io_events
    int nioe;           ///< count of pa_io_events
    ringbuffer_t *rb;
    void *peek_buffer;
    size_t peek_buffer_data_len;
    void *write_buffer;
    volatile int paused;
    pa_volume_t volume[PA_CHANNELS_MAX];
    // Flow control for the playback io events. POLLOUT is level-triggered, so a
    // wakeup with an empty ring is re-entered immediately and forever. These
    // track the registered event mask so POLLOUT can be dropped when there is
    // nothing to write and restored when the client writes again.
    pa_io_event_flags_t ioe_events;
    int out_enabled;
    // Hardware playback clock. hw_ptr read from /proc/asound on the real DAC,
    // not snd_pcm_delay, which through volumioswitch reports local + target and
    // so describes two stages rather than the one we feed.
    int64_t clock_origin_hw;
    int64_t clock_last_hw;
    pa_usec_t clock_frozen_usec;
    pa_usec_t clock_last_played;
    int clock_running;
    int clock_have_origin;
    int clock_have_path;
    int clock_logged;
    char clock_status_path[576];
    // Smoothed clock model. Real PulseAudio answers timing queries from a
    // fitted rate and offset rather than from a fresh measurement, so two reads
    // microseconds apart are consistent with each other by construction.
    // Soloist polls in bursts tens of microseconds apart, where a raw hardware
    // sample cannot be differenced meaningfully.
    struct timeval clock_model_at;    // when the model was last anchored
    int64_t clock_model_frames;       // hardware position at that anchor
    double clock_model_rate;          // frames per second, fitted
    int clock_model_valid;
    // Duration of the ALSA buffer actually opened, reported to the client as
    // configured_sink_usec so it knows what pipeline it is feeding.
    pa_usec_t configured_sink_usec;
    // Last cork request from the client. Uncork sets it; a failed reacquire
    // after a Volumio yield leaves it set so the backoff timer can keep trying.
    int want_running;
    pa_time_event *acquire_ev;
    int acquire_attempts;
    // Set when a reacquire on uncork found the chain owned by another source,
    // so the open error is logged once rather than on every attempt.
    int acquire_failed;
};

struct pa_operation {
    pa_operation_state_t state;
    pa_stream_success_cb_t stream_success_cb;
    pa_sink_input_info_cb_t sink_input_info_cb;
    pa_sink_info_cb_t sink_info_cb;
    pa_context_success_cb_t context_success_cb;
    pa_server_info_cb_t server_info_cb;
    pa_source_info_cb_t source_info_cb;
    void *cb_userdata;

    pa_mainloop_api *api;

    void (*mainloop_api_once_cb)(pa_mainloop_api *m, void *userdata);

    void (*handler)(pa_operation *op);

    int ref_cnt;
    int int_arg_1;
    char *char_ptr_arg_1;

    pa_cvolume pa_cvolume_arg_1;

    pa_stream *s;
    pa_context *c;
};

struct pa_defer_event {
    int enabled;
    pa_defer_event_cb_t cb;
    void *userdata;
    pa_mainloop *mainloop;
};

struct pa_time_event {
    int enabled;
    struct timeval when;
    pa_time_event_cb_t cb;
    void *userdata;
    pa_mainloop *mainloop;
    pa_time_event_destroy_cb_t destroy_cb;
};

struct pa_simple {
    pa_context *context;
    pa_threaded_mainloop *mainloop;
    pa_stream *stream;
    pa_stream_direction_t direction;
    int initialized;
};

pa_operation *
pa_operation_new(pa_mainloop_api *api, void (*handler)(pa_operation *op));

void
pa_operation_launch(pa_operation *op);

void
pa_operation_done(pa_operation *op);
