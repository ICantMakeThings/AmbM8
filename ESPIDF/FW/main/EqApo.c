// Equalizer in APO format for listening to music, getting a flat curve based on your headphones, or live monitoring if you wanna for some reason

#include "EqApo.h"

#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846 // im hungry writing this
#endif

#define EQ_MAX_FILTERS 16
#define EQ_LINE_MAX 160
#define EQ_MAX_TOKENS 24

typedef enum {
    EQ_TYPE_NONE,
    EQ_TYPE_PEAK,
    EQ_TYPE_LOWPASS,
    EQ_TYPE_HIGHPASS,
    EQ_TYPE_LOWSHELF,
    EQ_TYPE_HIGHSHELF,
    EQ_TYPE_NOTCH
} eq_type_t;

typedef struct {
    eq_type_t type;
    float fc;
    float gain_db;
    float q;
} eq_param_t;

typedef struct { float b0, b1, b2, a1, a2; } eq_biquad_t;

static eq_param_t s_params[EQ_MAX_FILTERS];
static int s_count;
static bool s_enabled;
static float s_preamp_gain = 1.0f;

static uint32_t s_coef_rate;
static eq_biquad_t s_coefs[2][EQ_MAX_FILTERS];
static float s_state[2][EQ_MAX_FILTERS][3];

// Marking coeffienet cache
static void eq_invalidate(void)
{
    s_coef_rate = 0;
    memset(s_state, 0, sizeof(s_state));
}

// clears the filter chain
void eq_reset(void)
{
    s_count = 0;
    s_preamp_gain = 1.0f;
    s_enabled = false;
    eq_invalidate();
}

int eq_filter_count(void) { return s_count; }
void eq_set_enabled(bool on) { s_enabled = on; }
bool eq_enabled(void) { return s_enabled; }

// helper for numbers and stuf
static bool token_matches(const char *tok, const char *word)
{
    while (*tok && *word) {
        if (tolower((unsigned char)*tok) != tolower((unsigned char)*word)) return false;
        tok++;
        word++;
    }
    return *tok == '\0' && *word == '\0';
}

static bool prefix_icase(const char *s, const char *pre)
{
    while (*pre) {
        if (tolower((unsigned char)*s) != tolower((unsigned char)*pre)) return false;
        s++;
        pre++;
    }
    return true;
}

static eq_type_t parse_type(const char *tok)
{
    if (token_matches(tok, "pk")) return EQ_TYPE_PEAK;
    if (token_matches(tok, "lp")) return EQ_TYPE_LOWPASS;
    if (token_matches(tok, "hp")) return EQ_TYPE_HIGHPASS;
    if (token_matches(tok, "ls") || token_matches(tok, "lsc")) return EQ_TYPE_LOWSHELF;
    if (token_matches(tok, "hs") || token_matches(tok, "hsc")) return EQ_TYPE_HIGHSHELF;
    if (token_matches(tok, "no")) return EQ_TYPE_NOTCH;
    return EQ_TYPE_NONE;
}

static float token_number(const char *tok)
{
    if (!tok) return 0.0f;

    return strtof(tok, NULL);
}

static void parse_filter_tokens(char **toks, int ntoks)
{
    bool on = true;
    eq_type_t type = EQ_TYPE_NONE;
    float fc = 0.0f, gain_db = 0.0f, q = -1.0f;

    for (int i = 0; i < ntoks; i++) {
        const char *t = toks[i];
        if (!t || !*t) continue;
        if (token_matches(t, "on")) { on = true; continue; }
        if (token_matches(t, "off")) { on = false; continue; }
        if (token_matches(t, "gain")) {
            if (t[4]) gain_db = token_number(t + 4);
            else if (i + 1 < ntoks) gain_db = token_number(toks[++i]);
            continue;
        }
        if (token_matches(t, "fc")) {
            if (t[2]) fc = token_number(t + 2);
            else if (i + 1 < ntoks) fc = token_number(toks[++i]);
            continue;
        }
        if (token_matches(t, "q")) {
            if (t[1]) q = token_number(t + 1);
            else if (i + 1 < ntoks) q = token_number(toks[++i]);
            continue;
        }
        eq_type_t type2 = parse_type(t);
        if (type == EQ_TYPE_NONE && type2 != EQ_TYPE_NONE) type = type2;
    }
    if (!on || type == EQ_TYPE_NONE || fc <= 10.0f) return;
    if (s_count >= EQ_MAX_FILTERS) return;

    s_params[s_count].type = type;
    s_params[s_count].fc = fc;
    s_params[s_count].gain_db = gain_db;
    s_params[s_count].q = q > 0.0f ? q : (type == EQ_TYPE_PEAK ? 1.4f : 0.707f);
    s_count++;
}

// routes pre-amp & filter entries
static void parse_line_tokens(char *body)
{
    char *toks[EQ_MAX_TOKENS];
    int n = 0;
    char *save = NULL;
    for (char *tok = strtok_r(body, " \t", &save); tok && n < EQ_MAX_TOKENS; tok = strtok_r(NULL, " \t", &save)) {
        toks[n++] = tok;
    }
    parse_filter_tokens(toks, n);
}

// parses open EQ
static bool parse_file(FILE *f)
{
    char line[EQ_LINE_MAX];
    bool ok = false;
    while (fgets(line, sizeof(line), f)) {

        char *hash = strchr(line, '#');
        if (hash) *hash = '\0';
        for (char *c = line; *c; c++) {
            if (*c == '\n' || *c == '\r') { *c = '\0'; break; }
        }

        char *p = line;
        while (*p == ' ' || *p == '\t') p++;
        if (!*p) continue;

        if (prefix_icase(p, "preamp")) {
            const char *numstr = p + 6;
            while (*numstr == ':' || *numstr == ' ' || *numstr == '\t') numstr++;
            float val = token_number(numstr);
            if (val != 0.0f) {
                s_preamp_gain = expf(val * 0.11512925464970228f);
            }
            continue;
        }
        if (prefix_icase(p, "filter")) {
            char *colon = strchr(p, ':');
            if (!colon) continue;
            parse_line_tokens(colon + 1);
            if (s_count > 0) ok = true;
        }

    }
    return ok;
}

// opens eq.txt from SD and does its black magic
bool eq_load_file(const char *path)
{
    FILE *f = fopen(path, "r");
    if (!f) return false;

    s_count = 0;
    s_preamp_gain = 1.0f;
    bool parsed = parse_file(f);
    fclose(f);

    eq_invalidate();
    (void)parsed;
    return s_count > 0;
}

// biquad per filter type'esque
static void design_biquad(const eq_param_t *p, uint32_t rate, eq_biquad_t *out)
{
    const float w = 2.0f * (float)M_PI * p->fc / (float)rate;
    float cw = cosf(w), sw = sinf(w);
    float alpha = sw / (2.0f * p->q);
    float a = powf(10.0f, p->gain_db * 0.025f);
    float b0, b1, b2, a0, a1, a2;

    switch (p->type) {
    case EQ_TYPE_PEAK:
        b0 = 1.0f + alpha * a;
        b1 = -2.0f * cw;
        b2 = 1.0f - alpha * a;
        a0 = 1.0f + alpha / a;
        a1 = -2.0f * cw;
        a2 = 1.0f - alpha / a;
        break;
    case EQ_TYPE_LOWPASS:
        b0 = (1.0f - cw) * 0.5f;
        b1 = 1.0f - cw;
        b2 = (1.0f - cw) * 0.5f;
        a0 = 1.0f + alpha;
        a1 = -2.0f * cw;
        a2 = 1.0f - alpha;
        break;
    case EQ_TYPE_HIGHPASS:
        b0 = (1.0f + cw) * 0.5f;
        b1 = -(1.0f + cw);
        b2 = (1.0f + cw) * 0.5f;
        a0 = 1.0f + alpha;
        a1 = -2.0f * cw;
        a2 = 1.0f - alpha;
        break;
    case EQ_TYPE_NOTCH:
        b0 = 1.0f;
        b1 = -2.0f * cw;
        b2 = 1.0f;
        a0 = 1.0f + alpha;
        a1 = -2.0f * cw;
        a2 = 1.0f - alpha;
        break;
    case EQ_TYPE_LOWSHELF: {
        float sq = sqrtf(a) * sqrtf(alpha);
        b0 = a * ((a + 1.0f) - (a - 1.0f) * cw + sq * sw);
        b1 = 2.0f * a * ((a - 1.0f) - (a + 1.0f) * cw);
        b2 = a * ((a + 1.0f) - (a - 1.0f) * cw - sq * sw);
        a0 = (a + 1.0f) + (a - 1.0f) * cw + sq * sw;
        a1 = -2.0f * ((a - 1.0f) + (a + 1.0f) * cw);
        a2 = (a + 1.0f) + (a - 1.0f) * cw - sq * sw;
        break;
    }
    case EQ_TYPE_HIGHSHELF: {
        float sq = sqrtf(a) * sqrtf(alpha);
        b0 = a * ((a + 1.0f) + (a - 1.0f) * cw + sq * sw);
        b1 = -2.0f * a * ((a - 1.0f) + (a + 1.0f) * cw);
        b2 = a * ((a + 1.0f) + (a - 1.0f) * cw - sq * sw);
        a0 = (a + 1.0f) - (a - 1.0f) * cw + sq * sw;
        a1 = 2.0f * ((a - 1.0f) - (a + 1.0f) * cw);
        a2 = (a + 1.0f) - (a - 1.0f) * cw - sq * sw;
        break;
    }
    default:
        b0 = a0 = 1.0f;
        b1 = b2 = a1 = a2 = 0.0f;
        break;
    }

    out->b0 = b0 / a0;
    out->b1 = b1 / a0;
    out->b2 = b2 / a0;
    out->a1 = a1 / a0;
    out->a2 = a2 / a0;
}

// recomputes ting ting
static void eq_ensure_coefs(uint32_t rate)
{
    if (rate < 8000u) rate = 8000u;
    if (s_coef_rate == rate) return;
    if (s_coef_rate != 0) memset(s_state, 0, sizeof(s_state));

    for (int ch = 0; ch < 2; ch++) {
        for (int i = 0; i < s_count; i++) {
            design_biquad(&s_params[i], rate, &s_coefs[ch][i]);
        }
    }
    s_coef_rate = rate;
}

// goes thru enabled filter chain
static inline float eq_run_sample(int channel, float x)
{
    for (int i = 0; i < s_count; i++) {
        const eq_biquad_t *c = &s_coefs[channel][i];
        float *st = s_state[channel][i];

        float y = c->b0 * x + st[0];
        st[0] = c->b1 * x - c->a1 * y + st[1];
        st[1] = c->b2 * x - c->a2 * y;
        x = y;
    }
    return x * s_preamp_gain;
}

// stereo int32 or mono float processing depending on input
void eq_apply_float(int channel, float *samples, size_t count, uint32_t rate)
{
    if (!s_enabled || s_count <= 0 || !samples || count == 0) return;
    if (channel < 0 || channel > 1) return;
    eq_ensure_coefs(rate);
    for (size_t i = 0; i < count; i++) samples[i] = eq_run_sample(channel, samples[i]);
}

void eq_apply_int32_stereo(int32_t *stereo, size_t frames, uint32_t rate)
{
    if (!s_enabled || s_count <= 0 || !stereo || frames == 0) return;
    eq_ensure_coefs(rate);
    for (size_t i = 0; i < frames; i++) {
        float l = (float)stereo[2 * i] / 2147483648.0f;
        float r = (float)stereo[2 * i + 1] / 2147483648.0f;
        l = eq_run_sample(0, l) * 2147483648.0f;
        r = eq_run_sample(1, r) * 2147483648.0f;
        if (l > 2147483520.0f) l = 2147483520.0f;
        if (l < -2147483648.0f) l = -2147483648.0f;
        if (r > 2147483520.0f) r = 2147483520.0f;
        if (r < -2147483648.0f) r = -2147483648.0f;
        stereo[2 * i] = (int32_t)l;
        stereo[2 * i + 1] = (int32_t)r;
    }
}
