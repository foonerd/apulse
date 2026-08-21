#undef NDEBUG
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <alsa/asoundlib.h>
#include <pulse/pulseaudio.h>

size_t
pa_frame_size(const pa_sample_spec *ss)
{
    if (!ss)
        return 0;
    if (ss->format == PA_SAMPLE_FLOAT32NE)
        return 4u * ss->channels;
    if (ss->format == PA_SAMPLE_S16NE)
        return 2u * ss->channels;
    return 0;
}

size_t
pa_sample_size(const pa_sample_spec *ss)
{
    if (!ss)
        return 0;
    if (ss->format == PA_SAMPLE_FLOAT32NE)
        return 4;
    if (ss->format == PA_SAMPLE_S16NE)
        return 2;
    return 0;
}

float
pa_sw_volume_to_linear(pa_volume_t v)
{
    (void)v;
    return 1.f;
}

const char *
pa_sample_format_to_string(pa_sample_format_t f)
{
    (void)f;
    return "test";
}

void
trace_error(const char *fmt, ...)
{
    (void)fmt;
}

#include <src/util.c>

static void
test_float_to_s24_3le(void)
{
    pa_sample_spec ss = {
        .format = PA_SAMPLE_FLOAT32NE, .rate = 44100, .channels = 2,
    };
    float in[4] = {0.f, 1.f, -1.f, 0.5f};
    unsigned char out[12];
    int32_t v;

    assert(pa_convert_frames_to_alsa(in, out, 2, &ss, SND_PCM_FORMAT_S24_3LE) ==
           0);

    v = (int32_t)(out[0] | (out[1] << 8) | ((int8_t)out[2] << 16));
    assert(v == 0);

    v = (int32_t)(out[3] | (out[4] << 8) | ((int8_t)out[5] << 16));
    assert(v == 8388607);

    v = (int32_t)(out[6] | (out[7] << 8) | ((int8_t)out[8] << 16));
    assert(v == -8388607);

    v = (int32_t)(out[9] | (out[10] << 8) | ((int8_t)out[11] << 16));
    assert(v == 4194304 || v == 4194303);
}

static void
test_s16_to_s24_3le(void)
{
    pa_sample_spec ss = {
        .format = PA_SAMPLE_S16NE, .rate = 44100, .channels = 1,
    };
    int16_t in[1] = {256};
    unsigned char out[3];
    int32_t v;

    assert(pa_convert_frames_to_alsa(in, out, 1, &ss, SND_PCM_FORMAT_S24_3LE) ==
           0);
    v = (int32_t)(out[0] | (out[1] << 8) | ((int8_t)out[2] << 16));
    assert(v == (256 << 8));
}

static void
test_identity_s16(void)
{
    pa_sample_spec ss = {
        .format = PA_SAMPLE_S16NE, .rate = 44100, .channels = 2,
    };
    int16_t in[4] = {1, 2, 3, 4};
    int16_t out[4] = {0};

    assert(pa_convert_frames_to_alsa(in, out, 2, &ss, SND_PCM_FORMAT_S16_LE) ==
           0);
    assert(memcmp(in, out, sizeof(in)) == 0);
}

static void
test_frame_size(void)
{
    assert(pa_alsa_frame_size(SND_PCM_FORMAT_S24_3LE, 2) == 6);
    assert(pa_alsa_frame_size(SND_PCM_FORMAT_S24_LE, 2) == 8);
    assert(pa_alsa_frame_size(SND_PCM_FORMAT_S16_LE, 2) == 4);
}

int
main(void)
{
    test_frame_size();
    test_float_to_s24_3le();
    test_s16_to_s24_3le();
    test_identity_s16();
    printf("pass\n");
    return 0;
}
