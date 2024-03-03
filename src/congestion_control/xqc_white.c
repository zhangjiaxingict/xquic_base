/**
 * @copyright Copyright (c) 2022, Alibaba Group Holding Limited
 * 
 * white based on https://tools.ietf.org/html/rfc8312
 */

#include "src/congestion_control/xqc_white.h"
#include "src/common/xqc_config.h"
#include <math.h>


#define XQC_white_FAST_CONVERGENCE  1
#define XQC_white_MSS               1460
#define XQC_white_BETA              718     /* 718/1024=0.7 */
#define XQC_white_BETA_SCALE        1024
#define XQC_white_C                 410     /* 410/1024=0.4 */
#define XQC_CUBE_SCALE              40u     /* 2^40=1024 * 1024^3 */
#define XQC_white_TIME_SCALE        10u
#define XQC_white_MAX_SSTHRESH      0xFFFFFFFF

#define XQC_white_MIN_WIN           (4 * XQC_white_MSS)
#define XQC_white_MAX_INIT_WIN      (100 * XQC_white_MSS)
#define XQC_white_INIT_WIN          (32 * XQC_white_MSS)

const static uint64_t xqc_cube_factor =
    (1ull << XQC_CUBE_SCALE) / XQC_white_C / XQC_white_MSS;


static void
xqc_white_update(void *cong_ctl, uint32_t acked_bytes, xqc_usec_t now)
{
    return;
}


size_t
xqc_white_size()
{
    return sizeof(xqc_white_t);
}

static void
xqc_white_init(void *cong_ctl, xqc_send_ctl_t *ctl_ctx, xqc_cc_params_t cc_params)
{
    xqc_white_t *white = (xqc_white_t *)(cong_ctl);
    white->epoch_start = 0;
    white->cwnd = XQC_white_INIT_WIN;
    white->tcp_cwnd = XQC_white_INIT_WIN;
    white->tcp_cwnd_cnt = 0;
    white->last_max_cwnd = XQC_white_INIT_WIN;
    white->ssthresh = XQC_white_MAX_SSTHRESH;
    white->congestion_recovery_start_time = 0;

    if (cc_params.customize_on) {
        cc_params.init_cwnd *= XQC_white_MSS;
        white->init_cwnd =
                cc_params.init_cwnd >= XQC_white_MIN_WIN && cc_params.init_cwnd <= XQC_white_MAX_INIT_WIN ?
                cc_params.init_cwnd : XQC_white_INIT_WIN;
    }
}


static void
xqc_white_on_lost(void *cong_ctl, xqc_usec_t lost_sent_time)
{
    return;
}


static void
xqc_white_on_ack(void *cong_ctl, xqc_packet_out_t *po, xqc_usec_t now)
{
    return;
}

uint64_t
xqc_white_get_cwnd(void *cong_ctl)
{
    xqc_white_t *white = (xqc_white_t *)(cong_ctl);
    return white->cwnd;
}

void
xqc_white_reset_cwnd(void *cong_ctl)
{
    return;
}


void
xqc_white_restart_from_idle(void *cong_ctl, uint64_t arg)
{
    return;
}

static int
xqc_white_in_recovery(void *cong_ctl)
{
    return 0;
}

void xqc_white_renew_cwnd_srtt(void *cong_ctl, uint64_t cwnd, uint64_t pacing_rate, uint64_t bw, uint64_t queue_size, uint64_t srtt, xqc_sample_t *sampler)
{
    xqc_white_t *white = (xqc_white_t *)cong_ctl;
    white->cwnd = cwnd;
    white->tcp_cwnd = cwnd;
    printf("cwnd: %ld,white->cwnd: %ld",cwnd,white->cwnd);
}

const xqc_cong_ctrl_callback_t xqc_white_cb = {
    .xqc_cong_ctl_size              = xqc_white_size,
    .xqc_cong_ctl_init              = xqc_white_init,
    .xqc_cong_ctl_on_lost           = xqc_white_on_lost,
    .xqc_cong_ctl_on_ack            = xqc_white_on_ack,
    .xqc_cong_ctl_get_cwnd          = xqc_white_get_cwnd,
    .xqc_cong_ctl_reset_cwnd        = xqc_white_reset_cwnd,
    .xqc_cong_ctl_restart_from_idle = xqc_white_restart_from_idle,
    .xqc_cong_ctl_in_recovery       = xqc_white_in_recovery,
    .xqc_renew_cwnd_srtt            = xqc_white_renew_cwnd_srtt,
};
