#include "hevc_rps.h"

#include <stdlib.h>
#include <string.h>

/* ---- bit reader / writer over RBSP ------------------------------------ */

typedef struct { const uint8_t *b; size_t n, p; bool err; } BR;

static uint32_t rd(BR *r, int k)
{
    uint32_t v = 0;
    while (k-- > 0) {
        if (r->p >= r->n * 8) { r->err = true; return 0; }
        v = (v << 1) | ((r->b[r->p >> 3] >> (7 - (r->p & 7))) & 1u);
        r->p++;
    }
    return v;
}

static uint32_t rd_ue(BR *r)
{
    int z = 0;
    while (!rd(r, 1)) {
        if (r->err || ++z > 31) { r->err = true; return 0; }
    }
    return ((1u << z) - 1) + rd(r, z);
}

static int32_t rd_se(BR *r)
{
    uint32_t k = rd_ue(r);
    return (k & 1) ? (int32_t)((k + 1) / 2) : -(int32_t)(k / 2);
}

typedef struct { uint8_t *b; size_t cap, p; bool err; } BW;

static void wr(BW *w, uint32_t v, int k)
{
    while (k-- > 0) {
        if (w->p >= w->cap * 8) { w->err = true; return; }
        size_t i = w->p >> 3;
        if ((w->p & 7) == 0) w->b[i] = 0;
        if ((v >> k) & 1u) w->b[i] |= (uint8_t)(0x80u >> (w->p & 7));
        w->p++;
    }
}

static void wr_ue(BW *w, uint32_t v)
{
    uint32_t x = v + 1;
    int len = 0;
    while ((x >> len) > 1) len++;
    wr(w, 0, len);
    wr(w, x, len + 1);
}

/* copy bits [from, to) of r into w */
static void copy_bits(BW *w, const BR *r, size_t from, size_t to)
{
    BR c = { r->b, r->n, from, false };
    while (c.p < to && !w->err) {
        int k = (to - c.p) > 24 ? 24 : (int)(to - c.p);
        wr(w, rd(&c, k), k);
    }
}

/* ---- emulation prevention --------------------------------------------- */

static size_t unescape(const uint8_t *in, size_t n, uint8_t *out)
{
    size_t o = 0; int z = 0;
    for (size_t i = 0; i < n; i++) {
        if (z >= 2 && in[i] == 3) { z = 0; continue; }
        out[o++] = in[i];
        z = in[i] == 0 ? z + 1 : 0;
    }
    return o;
}

static size_t escape(const uint8_t *in, size_t n, uint8_t *out)
{
    size_t o = 0; int z = 0;
    for (size_t i = 0; i < n; i++) {
        if (z >= 2 && in[i] <= 3) { out[o++] = 3; z = 0; }
        out[o++] = in[i];
        z = in[i] == 0 ? z + 1 : 0;
    }
    if (z >= 2) out[o++] = 3; /* NAL must not end in 00 00 */
    return o;
}

/* ---- parameter sets ----------------------------------------------------- */

#define FAIL(why) do { r->last_reason = (why); return false; } while (0)

static bool parse_sps(HevcRps *r, BR *b)
{
    rd(b, 4);                                  /* vps id */
    if (rd(b, 3) != 0) FAIL("sps: sub-layers");
    rd(b, 1);
    rd(b, 32); rd(b, 32); rd(b, 32);           /* profile_tier_level, 96 bits */
    rd_ue(b);                                  /* sps id */
    uint32_t cf = rd_ue(b);
    if (cf == 3) FAIL("sps: 4:4:4");
    rd_ue(b); rd_ue(b);                        /* size */
    if (rd(b, 1)) { rd_ue(b); rd_ue(b); rd_ue(b); rd_ue(b); }
    rd_ue(b); rd_ue(b);                        /* bit depths */
    r->lsb_bits = (int)rd_ue(b) + 4;
    rd(b, 1);                                  /* sub_layer_ordering_info_present */
    rd_ue(b); rd_ue(b); rd_ue(b);
    for (int i = 0; i < 6; i++) rd_ue(b);      /* cb/tb sizes, hierarchy depths */
    if (rd(b, 1) && rd(b, 1)) FAIL("sps: scaling list data");
    rd(b, 1);                                  /* amp */
    r->sao = rd(b, 1);
    if (rd(b, 1)) { rd(b, 8); rd_ue(b); rd_ue(b); rd(b, 1); }  /* pcm */
    if (rd_ue(b) != 0) FAIL("sps: sps rps sets");
    r->long_term = rd(b, 1);
    r->temporal_mvp = rd(b, 1);
    if (b->err) FAIL("sps: truncated");
    if (r->long_term) FAIL("sps: long-term refs");
    if (r->temporal_mvp) FAIL("sps: temporal mvp");
    if (r->lsb_bits > 16) FAIL("sps: poc lsb");
    return true;
}

static bool parse_pps(HevcRps *r, BR *b)
{
    rd_ue(b); rd_ue(b);                        /* pps id, sps id */
    rd(b, 1);                                  /* dependent slices (first slice only) */
    r->output_flag = rd(b, 1);
    r->extra_bits = (int)rd(b, 3);
    rd(b, 1);                                  /* sign data hiding */
    r->cabac_init_present = rd(b, 1);
    rd_ue(b); rd_ue(b); rd_se(b);
    rd(b, 1); rd(b, 1);
    if (rd(b, 1)) rd_ue(b);                    /* cu_qp_delta */
    rd_se(b); rd_se(b);
    r->chroma_qp_offsets = rd(b, 1);
    if (rd(b, 1) || rd(b, 1)) FAIL("pps: weighted prediction");
    rd(b, 1);                                  /* transquant bypass */
    if (rd(b, 1) || rd(b, 1)) FAIL("pps: tiles/wpp");
    r->loop_filter_across = rd(b, 1);
    r->dbk_override = r->dbk_disabled = false;
    if (rd(b, 1)) {
        r->dbk_override = rd(b, 1);
        r->dbk_disabled = rd(b, 1);
        if (!r->dbk_disabled) { rd_se(b); rd_se(b); }
    }
    if (rd(b, 1)) FAIL("pps: scaling list data");
    if (rd(b, 1)) FAIL("pps: lists modification");
    rd_ue(b);                                  /* parallel merge level */
    if (rd(b, 1)) FAIL("pps: slice header extension");
    if (rd(b, 1)) FAIL("pps: extensions");
    if (b->err) FAIL("pps: truncated");
    return true;
}

/* ---- slice header rewrite ---------------------------------------------- */

/* Rewrites one slice NAL (RBSP in r->rbsp[0..n)).  *changed = false means
 * keep the original NAL.  Returns false if the header can't be handled. */
static bool rewrite_slice(HevcRps *r, size_t n, int type, BW *w, bool *changed)
{
    BR b = { r->rbsp, n, 16, false };
    *changed = false;
    if (!rd(&b, 1)) FAIL("slice: not first in picture");
    if (type >= 16 && type <= 23) rd(&b, 1);
    rd_ue(&b);                                 /* pps id */
    rd(&b, r->extra_bits);
    uint32_t st = rd_ue(&b);
    if (r->output_flag) rd(&b, 1);
    if (type == 19 || type == 20) return !b.err; /* IDR: no RPS */
    rd(&b, r->lsb_bits);
    size_t rps_start = b.p;
    if (rd(&b, 1)) FAIL("slice: sps rps");
    uint32_t nneg = rd_ue(&b), npos = rd_ue(&b);
    if (b.err || nneg > 16 || npos > 16) FAIL("slice: rps size");
    uint32_t dneg[16]; uint8_t uneg[16]; int last_used = -1;
    for (uint32_t i = 0; i < nneg; i++) {
        dneg[i] = rd_ue(&b); uneg[i] = (uint8_t)rd(&b, 1);
        if (uneg[i]) last_used = (int)i;
    }
    size_t pos_start = b.p;
    for (uint32_t i = 0; i < npos; i++) { rd_ue(&b); rd(&b, 1); }
    size_t rps_end = b.p;
    bool sao_l = false, sao_c = false;
    if (r->sao) { sao_l = rd(&b, 1); sao_c = rd(&b, 1); }
    if (st <= 1) {                             /* P or B */
        if (rd(&b, 1)) { rd_ue(&b); if (st == 0) rd_ue(&b); }
        if (st == 0) rd(&b, 1);                /* mvd_l1_zero */
        if (r->cabac_init_present) rd(&b, 1);
        rd_ue(&b);                             /* five_minus_max_num_merge_cand */
    }
    rd_se(&b);                                 /* slice_qp_delta */
    if (r->chroma_qp_offsets) { rd_se(&b); rd_se(&b); }
    bool dbk_disabled = r->dbk_disabled;
    if (r->dbk_override && rd(&b, 1)) {
        dbk_disabled = rd(&b, 1);
        if (!dbk_disabled) { rd_se(&b); rd_se(&b); }
    }
    if (r->loop_filter_across && (sao_l || sao_c || !dbk_disabled)) rd(&b, 1);
    size_t hdr_end = b.p;
    if (rd(&b, 1) != 1) FAIL("slice: alignment bit");
    while (b.p & 7) if (rd(&b, 1)) FAIL("slice: alignment zeros");
    if (b.err) FAIL("slice: truncated");

    uint32_t k = nneg;
    if ((uint32_t)r->keep < k) k = (uint32_t)r->keep;
    if ((uint32_t)(last_used + 1) > k) k = (uint32_t)(last_used + 1);
    if (k == nneg) return true;

    copy_bits(w, &b, 0, rps_start);
    wr(w, 0, 1);
    wr_ue(w, k);
    wr_ue(w, npos);
    for (uint32_t i = 0; i < k; i++) { wr_ue(w, dneg[i]); wr(w, uneg[i], 1); }
    copy_bits(w, &b, pos_start, rps_end);
    copy_bits(w, &b, rps_end, hdr_end);
    wr(w, 1, 1);
    while (w->p & 7) wr(w, 0, 1);
    size_t data = b.p >> 3, hb = w->p >> 3;
    if (w->err || hb + (n - data) > w->cap) FAIL("slice: buffer");
    memcpy(w->b + hb, r->rbsp + data, n - data);
    w->p = (hb + n - data) * 8;
    *changed = true;
    return true;
}

/* ---- access unit ---------------------------------------------------------- */

void hevc_rps_init(HevcRps *r, int keep)
{
    memset(r, 0, sizeof(*r));
    r->keep = keep;
}

void hevc_rps_fini(HevcRps *r)
{
    free(r->rbsp);
    free(r->out);
    memset(r, 0, sizeof(*r));
}

static bool grow(uint8_t **p, size_t *cap, size_t need)
{
    if (*cap >= need) return true;
    uint8_t *n = realloc(*p, need);
    if (!n) return false;
    *p = n; *cap = need;
    return true;
}

/* Finds the next NAL: returns payload start, sets *end (exclusive, trailing
 * zeros of the next 4-byte start code removed).  Returns len if none. */
static size_t next_nal(const uint8_t *in, size_t len, size_t from, size_t *end)
{
    size_t i = from, s = len;
    for (; i + 3 <= len; i++)
        if (in[i] == 0 && in[i + 1] == 0 && in[i + 2] == 1) { s = i + 3; break; }
    if (s >= len) return len;
    size_t j = s;
    for (; j + 3 <= len; j++)
        if (in[j] == 0 && in[j + 1] == 0 && in[j + 2] == 1) break;
    size_t e = (j + 3 <= len) ? j : len;
    while (e > s && e < len && in[e - 1] == 0) e--;
    *end = e;
    return s;
}

bool hevc_rps_process(HevcRps *r, const uint8_t *in, size_t len,
                      const uint8_t **out, size_t *out_len)
{
    *out = in; *out_len = len;
    if (r->keep <= 0) return true;
    if (!grow(&r->rbsp, &r->rbsp_cap, len + 16) ||
        !grow(&r->out, &r->out_cap, 3 * len + 64))
        return false;

    size_t o = 0, pos = 0, end = 0;
    bool any = false, failed = false;
    while ((pos = next_nal(in, len, pos, &end)) < len) {
        const uint8_t *nal = in + pos;
        size_t nlen = end - pos;
        int type = nlen ? (nal[0] >> 1) & 0x3f : -1;
        const uint8_t *src = nal; size_t slen = nlen;
        size_t rl = 0;
        if (type == 33 || type == 34) {
            rl = unescape(nal, nlen, r->rbsp);
            BR b = { r->rbsp, rl, 16, false };
            bool ok = type == 33 ? parse_sps(r, &b) : parse_pps(r, &b);
            if (type == 33) r->sps_ok = ok; else r->pps_ok = ok;
        } else if (type >= 0 && type <= 21 && r->sps_ok && r->pps_ok && !failed) {
            rl = unescape(nal, nlen, r->rbsp);
            uint8_t *tmp = r->out + 2 * len + 32;  /* scratch after output area */
            BW w = { tmp, len + 16, 0, false };
            bool changed = false;
            if (!rewrite_slice(r, rl, type, &w, &changed)) {
                failed = true;
            } else if (changed) {
                /* escape into the output directly after the start code */
                if (o + 4 + 2 * (w.p >> 3) + 2 > 2 * len + 32) { failed = true; }
                else {
                    memcpy(r->out + o, "\0\0\0\1", 4); o += 4;
                    o += escape(tmp, w.p >> 3, r->out + o);
                    any = true;
                    pos = end;
                    continue;
                }
            }
        }
        if (o + 4 + slen > 2 * len + 32) { failed = true; break; }
        memcpy(r->out + o, "\0\0\0\1", 4); o += 4;
        memcpy(r->out + o, src, slen); o += slen;
        pos = end;
    }
    if (failed) { r->failed++; return true; }
    if (!any) { r->passed++; return true; }
    r->rewritten++;
    *out = r->out; *out_len = o;
    return true;
}
