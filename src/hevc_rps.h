#ifndef HEVC_RPS_H
#define HEVC_RPS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * HEVC short-term RPS trimmer.
 *
 * The PS5 sends P slices whose explicit short-term RPS lists up to 9 previous
 * pictures, of which only the newest is used.  The webOS NDL decoder freezes on
 * that stream.  This rewrites each P/B slice header so the RPS keeps at most
 * `keep` negative entries (never fewer than the last "used" entry needs).
 * Slice data is not touched.  Any stream feature the parser does not support
 * disables the rewrite and the access unit is passed through unchanged.
 */
typedef struct HevcRps {
    int keep;            /* max negative refs kept; 0 = disabled */
    bool sps_ok, pps_ok; /* layout checks passed for the active SPS/PPS */
    int lsb_bits;        /* log2_max_pic_order_cnt_lsb */
    bool sao, temporal_mvp, long_term;
    int extra_bits;
    bool output_flag, cabac_init_present, chroma_qp_offsets, loop_filter_across;
    bool dbk_override, dbk_disabled;
    uint8_t *rbsp, *out;  /* work buffers */
    size_t rbsp_cap, out_cap;
    uint64_t rewritten, passed, failed;
    const char *last_reason;
} HevcRps;

void hevc_rps_init(HevcRps *r, int keep);
void hevc_rps_fini(HevcRps *r);

/* Process one Annex-B access unit.  On success *out and *out_len point to the
 * rewritten AU (owned by r, valid until the next call) or to the input
 * when nothing changed.  Returns false only on allocation failure. */
bool hevc_rps_process(HevcRps *r, const uint8_t *in, size_t len,
                      const uint8_t **out, size_t *out_len);

#endif
