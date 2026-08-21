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

#include "trace.h"
#include "util.h"

#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

// APULSE_EXTERNAL_VOLUME=1: keep sink-input volume for the Pulse API, do
// not scale samples. Volumio's SoftMaster is the mixer; scaling here sits
// in front of peppyalsa.
static int
external_volume(void)
{
    static int cached = -1;

    if (cached < 0) {
        const char *e = getenv("APULSE_EXTERNAL_VOLUME");

        cached = (e && e[0] && e[0] != '0') ? 1 : 0;
    }
    return cached;
}

int
pa_format_to_alsa(pa_sample_format_t format)
{
    switch (format) {
    case PA_SAMPLE_U8:
        return SND_PCM_FORMAT_U8;
    case PA_SAMPLE_ALAW:
        return SND_PCM_FORMAT_A_LAW;
    case PA_SAMPLE_ULAW:
        return SND_PCM_FORMAT_MU_LAW;
    case PA_SAMPLE_S16LE:
        return SND_PCM_FORMAT_S16_LE;
    case PA_SAMPLE_S16BE:
        return SND_PCM_FORMAT_S16_BE;
    case PA_SAMPLE_FLOAT32LE:
        return SND_PCM_FORMAT_FLOAT_LE;
    case PA_SAMPLE_FLOAT32BE:
        return SND_PCM_FORMAT_FLOAT_BE;
    case PA_SAMPLE_S32LE:
        return SND_PCM_FORMAT_S32_LE;
    case PA_SAMPLE_S32BE:
        return SND_PCM_FORMAT_S32_BE;
    case PA_SAMPLE_S24LE:
        return SND_PCM_FORMAT_S24_3LE;
    case PA_SAMPLE_S24BE:
        return SND_PCM_FORMAT_S24_3BE;
    case PA_SAMPLE_S24_32LE:
        return SND_PCM_FORMAT_S24_LE;
    case PA_SAMPLE_S24_32BE:
        return SND_PCM_FORMAT_S24_BE;
    default:
        return SND_PCM_FORMAT_UNKNOWN;
    }
}

pa_sample_format_t
pa_sample_format_from_string(const char *str)
{
    if (!str)
        return 0;

    if (strcmp(str, "u8") == 0) {
        return PA_SAMPLE_U8;
    } else if (strcmp(str, "aLaw") == 0) {
        return PA_SAMPLE_ALAW;
    } else if (strcmp(str, "uLaw") == 0) {
        return PA_SAMPLE_ULAW;
    } else if (strcmp(str, "s16le") == 0) {
        return PA_SAMPLE_S16LE;
    } else if (strcmp(str, "s16be") == 0) {
        return PA_SAMPLE_S16BE;
    } else if (strcmp(str, "float32le") == 0) {
        return PA_SAMPLE_FLOAT32LE;
    } else if (strcmp(str, "float32be") == 0) {
        return PA_SAMPLE_FLOAT32BE;
    } else if (strcmp(str, "s32le") == 0) {
        return PA_SAMPLE_S32LE;
    } else if (strcmp(str, "s32be") == 0) {
        return PA_SAMPLE_S32BE;
    } else if (strcmp(str, "s24le") == 0) {
        return PA_SAMPLE_S24LE;
    } else if (strcmp(str, "s24be") == 0) {
        return PA_SAMPLE_S24BE;
    } else if (strcmp(str, "s24-32le") == 0) {
        return PA_SAMPLE_S24_32LE;
    } else if (strcmp(str, "s24-32be") == 0) {
        return PA_SAMPLE_S24_32BE;
    } else {
        return 0;
    }
}

size_t
pa_find_multiple_of(size_t number, size_t multiple_of,
                    int towards_larger_numbers)
{
    if (multiple_of == 0)
        return number;

    size_t n = towards_larger_numbers ? (number + multiple_of - 1) : number;
    return n - (n % multiple_of);
}

void
pa_apply_volume_multiplier(void *buf, size_t sz,
                           const pa_volume_t volume[PA_CHANNELS_MAX],
                           const pa_sample_spec *ss)
{
    char *p = buf;
    char *last = p + sz;
    float fvol[PA_CHANNELS_MAX];
    uint32_t channels = MIN(ss->channels, PA_CHANNELS_MAX);

    if (channels == 0) {
        // No channels — nothing to scale.
        return;
    }

    if (external_volume()) {
        return;
    }

    int all_normal = 1;
    for (uint32_t k = 0; k < channels; k++)
        all_normal = all_normal && (volume[k] == PA_VOLUME_NORM);

    if (all_normal) {
        // No scaling required.
        return;
    }

    for (uint32_t k = 0; k < channels; k++)
        fvol[k] = pa_sw_volume_to_linear(volume[k]);

    switch (ss->format) {
    case PA_SAMPLE_FLOAT32NE:
        while (p < last) {
            for (uint32_t k = 0; k < channels && p < last; k++) {
                float sample;
                memcpy(&sample, p, sizeof(sample));
                sample *= fvol[k];
                memcpy(p, &sample, sizeof(sample));
                p += sizeof(sample);
            }
        }
        break;

    case PA_SAMPLE_S16NE:
        while (p < last) {
            for (uint32_t k = 0; k < channels && p < last; k++) {
                int16_t sample;
                memcpy(&sample, p, sizeof(sample));
                float sample_scaled = sample * fvol[k];
                sample = CLAMP(sample_scaled, -32768.0, 32767.0);
                memcpy(p, &sample, sizeof(sample));
                p += sizeof(sample);
            }
        }
        break;

    case PA_SAMPLE_U8:
    case PA_SAMPLE_ALAW:
    case PA_SAMPLE_ULAW:
    case PA_SAMPLE_S16RE:
    case PA_SAMPLE_FLOAT32RE:
    case PA_SAMPLE_S32NE:
    case PA_SAMPLE_S32RE:
    case PA_SAMPLE_S24NE:
    case PA_SAMPLE_S24RE:
    case PA_SAMPLE_S24_32NE:
    case PA_SAMPLE_S24_32RE:
    default:
        trace_error("format %s is not implemented in %s\n",
                    pa_sample_format_to_string(ss->format), __func__);
        break;
    }
}

static float
output_trim_gain(void)
{
    static float gain = -1.f;

    if (gain < 0.f) {
        const char *e = getenv("APULSE_OUTPUT_TRIM_DB");
        char *end = NULL;
        long db = e ? strtol(e, &end, 10) : 0;

        if (!e || end == e || *end != '\0' || db == 0) {
            gain = 1.f;
        } else {
            if (db > 12)
                db = 12;
            if (db < -12)
                db = -12;
            gain = powf(10.f, (float)db / 20.f);
        }
    }
    return gain;
}

void
pa_apply_output_trim(void *buf, size_t sz, const pa_sample_spec *ss)
{
    char *p;
    char *last;
    uint32_t channels;
    float g;

    if (!buf || !ss || sz == 0)
        return;

    g = output_trim_gain();
    if (g == 1.f)
        return;

    channels = MIN(ss->channels, PA_CHANNELS_MAX);
    if (channels == 0)
        return;

    p = buf;
    last = p + sz;

    switch (ss->format) {
    case PA_SAMPLE_FLOAT32NE:
        while (p < last) {
            uint32_t k;

            for (k = 0; k < channels && p < last; k++) {
                float sample;

                memcpy(&sample, p, sizeof(sample));
                sample *= g;
                memcpy(p, &sample, sizeof(sample));
                p += sizeof(sample);
            }
        }
        break;

    case PA_SAMPLE_S16NE:
        while (p < last) {
            uint32_t k;

            for (k = 0; k < channels && p < last; k++) {
                int16_t sample;
                float sample_scaled;

                memcpy(&sample, p, sizeof(sample));
                sample_scaled = sample * g;
                sample = CLAMP(sample_scaled, -32768.0, 32767.0);
                memcpy(p, &sample, sizeof(sample));
                p += sizeof(sample);
            }
        }
        break;

    default:
        break;
    }
}

size_t
pa_alsa_frame_size(snd_pcm_format_t fmt, unsigned channels)
{
    int bits;

    if (channels == 0)
        return 0;
    bits = snd_pcm_format_physical_width(fmt);
    if (bits <= 0)
        return 0;
    return ((size_t)bits / 8) * (size_t)channels;
}

static float
clampf(float v, float lo, float hi)
{
    if (v < lo)
        return lo;
    if (v > hi)
        return hi;
    return v;
}

static int32_t
sample_to_s24(pa_sample_format_t fmt, const char *p)
{
    switch (fmt) {
    case PA_SAMPLE_FLOAT32NE: {
        float f;

        memcpy(&f, p, sizeof(f));
        return (int32_t)lrintf(clampf(f, -1.f, 1.f) * 8388607.f);
    }
    case PA_SAMPLE_S16NE: {
        int16_t s;

        memcpy(&s, p, sizeof(s));
        return (int32_t)s << 8;
    }
    default:
        return 0;
    }
}

static void
pack_s24(snd_pcm_format_t fmt, int32_t v, char *p)
{
    if (v > 8388607)
        v = 8388607;
    if (v < -8388608)
        v = -8388608;

    switch (fmt) {
    case SND_PCM_FORMAT_S24_3LE:
        p[0] = (char)(v & 0xff);
        p[1] = (char)((v >> 8) & 0xff);
        p[2] = (char)((v >> 16) & 0xff);
        break;
    case SND_PCM_FORMAT_S24_LE: {
        int32_t le = v;

        memcpy(p, &le, sizeof(le));
        break;
    }
    case SND_PCM_FORMAT_S16_LE: {
        int16_t s = (int16_t)(v >> 8);

        memcpy(p, &s, sizeof(s));
        break;
    }
    default:
        break;
    }
}

int
pa_convert_frames_to_alsa(const void *src, void *dst, size_t frames,
                          const pa_sample_spec *ss, snd_pcm_format_t fmt)
{
    const char *in;
    char *out;
    size_t src_width;
    size_t dst_width;
    size_t i;
    unsigned ch;
    unsigned channels;

    if (!src || !dst || !ss)
        return -1;
    if (frames == 0)
        return 0;
    if (pa_format_to_alsa(ss->format) == fmt) {
        size_t n = frames * pa_frame_size(ss);

        memcpy(dst, src, n);
        return 0;
    }
    if (ss->format != PA_SAMPLE_FLOAT32NE && ss->format != PA_SAMPLE_S16NE)
        return -1;
    if (fmt != SND_PCM_FORMAT_S24_3LE && fmt != SND_PCM_FORMAT_S24_LE &&
        fmt != SND_PCM_FORMAT_S16_LE)
        return -1;

    channels = ss->channels;
    if (channels == 0)
        return -1;
    src_width = pa_sample_size(ss);
    dst_width = (size_t)snd_pcm_format_physical_width(fmt) / 8;
    if (src_width == 0 || dst_width == 0)
        return -1;

    in = src;
    out = dst;
    for (i = 0; i < frames; i++) {
        for (ch = 0; ch < channels; ch++) {
            int32_t v = sample_to_s24(ss->format, in);

            pack_s24(fmt, v, out);
            in += src_width;
            out += dst_width;
        }
    }
    return 0;
}
