#include "float_math.h"
#include "userdelfx.h"
#include "fixed_math.h"
#include "biquad.hpp"
#include "delayline.hpp"
#include "simplelfo.hpp"

static const uint32_t BUF_LEN = 16384;
// HPF around ~80hz
static const float HPF_COEFF = 0.00524f;
// LPF around ~3.2khz
static const float LPF_COEFF = 0.2126f;
// keep Q below 0.7071f or it will create unpleasent self amplifying resonant peaks
static const float LPF_Q = 0.5f;
// keep this between .25 and .33
static const float CLIP_COEFF = 0.25f;
static const float DELAY_MAX = 16382.0f;
static const float DELAY_MIN = 720.0f;// ~15ms at 48khz
static float s_time, s_drive, s_mix, s_depth, s_time_lin;
static dsp::BiQuad s_lpf_l, s_lpf_r, s_hpf_l, s_hpf_r;
static dsp::DualDelayLine s_delay;
static __sdram f32pair_t s_delay_buf[BUF_LEN];

void DELFX_INIT(uint32_t platform, uint32_t api) {
    s_lpf_l.mCoeffs.setSOLP(LPF_COEFF, LPF_Q);
    s_lpf_r.mCoeffs.setSOLP(LPF_COEFF, LPF_Q);
    s_lpf_l.flush();
    s_lpf_r.flush();
    s_hpf_l.mCoeffs.setFOHP(HPF_COEFF);
    s_hpf_r.mCoeffs.setFOHP(HPF_COEFF);
    s_hpf_l.flush();
    s_hpf_r.flush();
    s_delay.setMemory(s_delay_buf, BUF_LEN);
    s_time_lin = 4800.f;
    s_time = 4800.f;
    s_depth = 0.4f;
    s_drive = 1.0f;
    s_mix = 0.5f;
}

void DELFX_PROCESS(float *xn, uint32_t frames) {
    // This lets the compiler do optomization magic
    float * __restrict mxn = xn;
    // this is the pointer to the end of the block of samples
    const float * mxn_e = mxn + 2 * frames;
    // grab the value from the time knob
    const float t = s_time;
    // grab our previous interpolated time value
    float time_lin = s_time_lin;
    f32pair_t wet;
    f32pair_t fb_sat;
    float dry;
    // loop the samples until we get to the end
    for(; mxn != mxn_e;) {
        // do a linear interpolation on the knob value
        time_lin = linintf(0.001f, time_lin, t);
        // read out delayed taps
        const float tap_l = s_delay.readFrac(time_lin).a;
        const float tap_r = s_delay.readFrac(time_lin).b;

        // do wet filtering
        wet.a = s_lpf_l.process_so(s_hpf_l.process_fo(tap_l));
        wet.b = s_lpf_r.process_so(s_hpf_r.process_fo(tap_r));

        // left channel
        dry = *mxn;
        fb_sat.a = fx_softclipf(CLIP_COEFF, dry + s_depth * wet.a);
        *(mxn++) = dry * (1.0f - s_mix) + wet.a * s_mix; // mix and increment

        // repeat same steps for right channel
        dry = *mxn;
        fb_sat.b = fx_softclipf(CLIP_COEFF, dry + s_depth * wet.b);
        *(mxn++) = dry * (1.0f - s_mix) + wet.b * s_mix;

        // write delay line data back to it's buffer
        s_delay.write(fb_sat);
    }

    // store back our current interpolated time for the next block
    s_time_lin = time_lin;
}

void DELFX_PARAM(uint8_t index, int32_t value) {
    const float pval = q31_to_f32(value);
    switch (index) {
        case k_user_delfx_param_time: {
            // map time to an exponential curve from DELAY_MIN to DELAY_MAX samples
            s_time = DELAY_MIN + pval * pval * (DELAY_MAX - DELAY_MIN);
            break;
        }
        case k_user_delfx_param_depth: {
            s_depth = pval * (0.56f + 0.42f * pval);
            s_drive = 1.0f + pval * 1.0f;
            break;
        }
        case k_user_delfx_param_shift_depth: {
            // pval is already 0.0f - 1.0f which is already suitable for mix
            s_mix = pval;
            break;
        }
    }
}
