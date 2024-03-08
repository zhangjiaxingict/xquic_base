/**
 * @copyright Copyright (c) 2022, Alibaba Group Holding Limited
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "src/congestion_control/xqc_bbr_white.h"
#include "src/congestion_control/xqc_sample.h"
#include "src/common/xqc_time.h"
#include "src/common/xqc_config.h"
#include "src/transport/xqc_send_ctl.h"
#include "src/transport/xqc_packet.h"

#define XQC_BBR_W_MAX_DATAGRAMSIZE    XQC_MSS
#define XQC_BBR_W_MIN_WINDOW          (4 * XQC_BBR_W_MAX_DATAGRAMSIZE)
#define XQC_BBR_W_MAX_WINDOW          (100 * XQC_BBR_W_MAX_DATAGRAMSIZE)

#define XQC_BBR_W_MAX_WINDOW_W          (1000000 * XQC_BBR_W_MAX_DATAGRAMSIZE)
/* The RECOMMENDED value is the minimum of 10 * kMaxDatagramSize and max(2* kMaxDatagramSize, 14720)) */
/* same init window as cubic */
/* 32 is too aggressive. we have observed heavy bufferbloat events from online deployment */
/* 1440 * 10 / 1200 = 12 */
#define XQC_BBR_W_INITIAL_WINDOW  (32 * XQC_BBR_W_MAX_DATAGRAMSIZE) 
/* Pacing gain cycle rounds */
#define XQC_BBR_W_CYCLE_LENGTH    8
#define XQC_BBR_W_INF             0x7fffffff
#define XQC_BBR_W_MAX_AI_SCALE    (~0U)


/* Size of window of bandwidth filter, in rtts */
const uint32_t xqc_bbr_w_bw_win_size = XQC_BBR_W_CYCLE_LENGTH + 2;
/* Window of min rtt filter, in sec */
const uint32_t xqc_bbr_w_minrtt_win_size = 10;
/* Minimum time spent in BBR_PROBE_RTT, in us*/
const uint32_t xqc_bbr_w_probertt_time_us = 100000;
/* Initial rtt before any samples are received, in ms  */
const uint64_t xqc_bbr_w_initial_rtt_ms = 100;
/* The gain of pacing rate for START_UP, 2/(ln2) */
const float xqc_bbr_w_high_gain = 2.885;
/* Gain in BBR_DRAIN */
const float xqc_bbr_w_drain_gain = 1.0 / 2.885;
/* Gain for cwnd in probe_bw, like slow start*/
const float xqc_bbr_w_cwnd_gain = 2.5;
/* Cycle of gains in PROBE_BW for pacing rate */
const float xqc_bbr_w_pacing_gain[] = {1.25, 0.75, 1, 1, 1, 1, 1, 1};
const float xqc_bbr_w_low_pacing_gain[] = {1.1, 0.9, 1, 1, 1, 1, 1, 1};
/* Minimum packets that need to ensure ack if there is delayed ack */
const uint32_t xqc_bbr_w_min_cwnd = 4 * XQC_BBR_W_MAX_DATAGRAMSIZE;
/* If bandwidth has increased by 1.25, there may be more bandwidth available */
const float xqc_bbr_w_fullbw_thresh = 1.1;
/* After 3 rounds bandwidth less than (1.25x), estimate the pipe is full */
const uint32_t xqc_bbr_w_fullbw_cnt = 3;
const float xqc_bbr_w_probe_rtt_gain = 0.75;
const uint32_t xqc_bbr_w_extra_ack_gain = 1;
const float xqc_bbr_w_max_extra_ack_time = 0.1;
const uint32_t xqc_bbr_w_ack_epoch_acked_reset_thresh = 1 << 20;
const float xqc_bbr_w_pacing_rate_margin_percent = 0;


/* BBRv2 parameters */
const float xqc_bbr2_w_drain_gain = 0.75;
const float xqc_bbr2_w_startup_cwnd_gain = 2.885 + 2;
/* keep minrtt valid for 10s if it has not been changed */
const uint32_t xqc_bbr2_w_minrtt_win_size_us = 3000000;
/* probe new minrtt in 2.5s*/
const uint32_t xqc_bbr2_w_probertt_win_size_us = 2500000;
const bool xqc_bbr2_w_extra_ack_in_startup = 1;
/* 10 packet-timed rtt */
const uint32_t xqc_bbr2_w_extra_ack_win_rtt = 5;
/* 2 packet-timed rtt */
const uint32_t xqc_bbr2_w_extra_ack_win_rtt_in_startup = 1; 
/* slow down */
const float xqc_bbr2_w_startup_pacing_gain_on_lost = 1.5;
const bool xqc_bbr2_w_slow_down_startup_on_lost = 0;

const float ewma_parameter = 0.5;

/* 5RTT */
#if XQC_BBR_W_RTTVAR_COMPENSATION_ENABLED
static const float xqc_bbr_w_windowed_max_rtt_win_size = 5;
static const float xqc_bbr_w_rtt_compensation_startup_thresh = 2;
static const float xqc_bbr_w_rtt_compensation_thresh = 1;
static const float xqc_bbr_w_rtt_compensation_cwnd_factor = 1;
#endif

size_t 
xqc_bbr_w_size()
{
    return sizeof(xqc_bbr_w_t);
}

static void 
xqc_bbr_enter_startup(xqc_bbr_w_t *bbr)
{
    bbr->mode = BBR_STARTUP;
    bbr->pacing_gain = xqc_bbr_w_high_gain;
    bbr->cwnd_gain = xqc_bbr2_w_startup_cwnd_gain;
#if XQC_BBR_W_RTTVAR_COMPENSATION_ENABLED
    bbr->rtt_compensation_thresh = xqc_bbr_rtt_compensation_startup_thresh;
#endif
}

static void 
xqc_bbr_w_init_pacing_rate(xqc_bbr_w_t *bbr, xqc_sample_t *sampler)
{
    uint64_t bandwidth;
    if (sampler->srtt) {
        bbr->has_srtt = 1;
    }
    bandwidth = bbr->congestion_window * (uint64_t)MSEC2SEC
        / (sampler->srtt ? sampler->srtt : 1000);
    bbr->pacing_rate = bbr->pacing_gain * bandwidth;
}

static void 
xqc_bbr_init(void *cong_ctl, xqc_sample_t *sampler, xqc_cc_params_t cc_params)
{
    xqc_bbr_w_t *bbr = (xqc_bbr_w_t *)(cong_ctl);
    uint64_t now = xqc_monotonic_timestamp();

    memset(bbr, 0, sizeof(*bbr));
    xqc_win_filter_reset(&bbr->bandwidth, 0, 0);
#if XQC_BBR_W_RTTVAR_COMPENSATION_ENABLED
    xqc_win_filter_reset(&bbr->max_rtt, 0, 0);
    bbr->max_rtt_win_len = xqc_bbr_windowed_max_rtt_win_size;
    if (cc_params.cc_optimization_flags & XQC_BBR_W_FLAG_RTTVAR_COMPENSATION) {
        bbr->rttvar_compensation_on = 1;
    }
#endif
    bbr->beyond_target_cwnd = 0;
    bbr->snd_cwnd_cnt_bytes = 0;
    bbr->ai_scale = 1;
    bbr->ai_scale_accumulated_bytes = 0;
    bbr->min_rtt = sampler->srtt ? sampler->srtt : XQC_BBR_W_INF;
    bbr->min_rtt_stamp = now;
    bbr->probe_rtt_min_us = sampler->srtt ? sampler->srtt : XQC_BBR_W_INF;
    bbr->probe_rtt_min_us_stamp = now;
    bbr->round_start = 0;
    bbr->round_cnt = 0;
    bbr->next_round_delivered = 0;
    bbr->probe_rtt_round_done = FALSE;
    bbr->probe_rtt_round_done_stamp = 0;
    bbr->packet_conservation = FALSE;
    bbr->prior_cwnd = 0;
    bbr->initial_congestion_window = XQC_BBR_W_MAX_WINDOW_W;
    bbr->congestion_window = XQC_BBR_W_MAX_WINDOW_W;
    bbr->has_srtt = 0;
    bbr->idle_restart = 0;
    bbr->packet_conservation = 0;
    bbr->recovery_mode = BBR_NOT_IN_RECOVERY;
    bbr->just_enter_recovery_mode = FALSE;
    bbr->just_exit_recovery_mode = FALSE;
    bbr->recovery_start_time = 0;
    bbr->extra_ack_stamp = now;
    bbr->epoch_ack = 0;
    bbr->extra_ack_round_rtt = 0;
    bbr->extra_ack_idx = 0;
    bbr->extra_ack[0] = 0;
    bbr->extra_ack[1] = 0;
    bbr->extra_ack_in_startup = xqc_bbr2_w_extra_ack_in_startup;
    bbr->extra_ack_win_len = xqc_bbr2_w_extra_ack_win_rtt;
    bbr->extra_ack_win_len_in_startup = xqc_bbr2_w_extra_ack_win_rtt_in_startup;
    bbr->full_bandwidth_cnt = 0;
    bbr->full_bandwidth_reached = FALSE;
    bbr->last_qlen_ratio = 0;

    if (cc_params.customize_on) {
        cc_params.init_cwnd *= XQC_BBR_W_MAX_DATAGRAMSIZE;
        bbr->initial_congestion_window =
            cc_params.init_cwnd >= XQC_BBR_W_MIN_WINDOW 
            && cc_params.init_cwnd <= XQC_BBR_W_MAX_WINDOW 
            ? cc_params.init_cwnd : XQC_BBR_W_INITIAL_WINDOW;

        if (cc_params.expect_bw > 0) {
            bbr->enable_expect_bw = TRUE;
            bbr->expect_bw = cc_params.expect_bw;
        }
        if (cc_params.max_expect_bw > 0) {
            bbr->enable_max_expect_bw = TRUE;
            bbr->max_expect_bw = cc_params.max_expect_bw;
        }
    }

    xqc_bbr_enter_startup(bbr);
    xqc_bbr_w_init_pacing_rate(bbr, sampler);
}

static uint32_t 
xqc_bbr_max_bw(xqc_bbr_w_t *bbr)
{
    return xqc_win_filter_get(&bbr->bandwidth);
}

static void 
xqc_bbr_update_bandwidth(xqc_bbr_w_t *bbr, xqc_sample_t *sampler, uint32_t bandwidth)
{
    bbr->round_start = FALSE;
    /* Check whether the data is legal */
    if (/*sampler->delivered < 0 ||*/ sampler->interval <= 0) {
        return;
    }

    /* 
     * check whether the next BBR cycle is reached
     * at the beginning of the cycle, the number of packets sent is less than or equal to
     * the maximum number of packets that have been sent when the current ack packet is sent.
     */
    if (bbr->next_round_delivered <= sampler->prior_delivered) {
        bbr->next_round_delivered = sampler->total_acked;
        bbr->round_cnt++;
        bbr->round_start = TRUE;
        bbr->packet_conservation = 0;
        xqc_log(sampler->send_ctl->ctl_conn->log, XQC_LOG_DEBUG, 
                "|BBRv1: RTT round update %ud -> %ud|",
                bbr->round_cnt - 1, bbr->round_cnt);
    }
    /* FIXED: It may reduce the est. bw due to network instability. */
    /*  if (sampler->lagest_ack_time > bbr->last_round_trip_time) {
        bbr->round_cnt++;
        bbr->last_round_trip_time = xqc_monotonic_timestamp();
    } */

    if (bbr->enable_max_expect_bw && bandwidth >= bbr->max_expect_bw) {
        bandwidth = bbr->max_expect_bw;
    }

    xqc_win_filter_max(&bbr->bandwidth, xqc_bbr_w_bw_win_size, 
                           bbr->round_cnt, bandwidth);
    xqc_log(sampler->send_ctl->ctl_conn->log, XQC_LOG_DEBUG, 
            "|BBR_white: BW filter updated (%ud) in round %ud|",
            bandwidth, bbr->round_cnt);
}

static uint32_t 
xqc_bbr_bdp(xqc_bbr_w_t *bbr)
{
    return bbr->min_rtt * xqc_win_filter_get(&bbr->bandwidth) / MSEC2SEC;
}

#if XQC_BBR_W_RTTVAR_COMPENSATION_ENABLED
static uint32_t
xqc_bbr_compensate_cwnd_for_rttvar(xqc_bbr_w_t *bbr, xqc_sample_t *sampler)
{
    xqc_usec_t srtt = sampler->srtt;
    xqc_usec_t recent_max_rtt = xqc_win_filter_get_u64(&bbr->max_rtt);
    xqc_usec_t compensation_thresh = (1 + bbr->rtt_compensation_thresh) *
                                     bbr->min_rtt;
    uint32_t cwnd_addition = 0;
    if (recent_max_rtt >= compensation_thresh) {
        if (srtt > bbr->min_rtt) {
            xqc_usec_t rtt_var = (srtt - bbr->min_rtt);
            cwnd_addition = (xqc_bbr_max_bw(bbr) * rtt_var / MSEC2SEC) * 
                                       xqc_bbr_rtt_compensation_cwnd_factor;

        } else {
            xqc_log(sampler->send_ctl->ctl_conn->log, XQC_LOG_WARN, 
                    "|rttvar compensation|weird things happened|"
                    "|srtt %ui <= min_rtt %ui|", 
                    srtt, bbr->min_rtt);
        }
    }
    return cwnd_addition;
}
#endif

static uint32_t 
xqc_bbr_w_target_cwnd(xqc_bbr_w_t *bbr, float gain)
{
    if (bbr->min_rtt == XQC_BBR_W_INF) {
        return bbr->initial_congestion_window;
    }
    uint32_t cwnd = gain * xqc_bbr_bdp(bbr);
    return xqc_max(cwnd, XQC_BBR_W_MIN_WINDOW);
}

static bool 
xqc_bbr_is_next_cycle_phase(xqc_bbr_w_t *bbr, xqc_sample_t *sampler)
{
    bool is_full_length = (sampler->now - bbr->last_cycle_start) > bbr->min_rtt;
    uint32_t inflight = sampler->prior_inflight;
    bool should_advance_gain_cycling = is_full_length;
    if (bbr->pacing_gain > 1.0) {
        should_advance_gain_cycling = is_full_length 
            && (sampler->loss 
                || inflight >= xqc_bbr_w_target_cwnd(bbr, bbr->pacing_gain));
    }
    /* Drain to target: 1xBDP */
    if (bbr->pacing_gain < 1.0) {
        should_advance_gain_cycling = is_full_length 
            || (inflight <= xqc_bbr_w_target_cwnd(bbr, 1.0));
    }
    return should_advance_gain_cycling;
}

static float 
xqc_bbr_get_pacing_gain(xqc_bbr_w_t *bbr, uint32_t cycle_idx)
{
    if (bbr->enable_expect_bw && xqc_bbr_max_bw(bbr) >= bbr->expect_bw) {
        return xqc_bbr_w_low_pacing_gain[cycle_idx];
    }
    return xqc_bbr_w_pacing_gain[cycle_idx];
}

static void 
xqc_bbr_update_cycle_phase(xqc_bbr_w_t *bbr, xqc_sample_t *sampler)
{
    if (bbr->mode == BBR_PROBE_BW 
        && xqc_bbr_is_next_cycle_phase(bbr, sampler))
    {
        bbr->cycle_idx = (bbr->cycle_idx + 1) % XQC_BBR_W_CYCLE_LENGTH;
        bbr->last_cycle_start = sampler->now;
        bbr->pacing_gain = xqc_bbr_get_pacing_gain(bbr, bbr->cycle_idx);
    }
}

static uint32_t 
xqc_bbr_extra_ack(xqc_bbr_w_t *bbr)
{
    return xqc_max(bbr->extra_ack[0], bbr->extra_ack[1]);
}

static uint32_t 
xqc_bbr_ack_aggregation_cwnd(xqc_bbr_w_t *bbr)
{
    uint32_t max_aggr_cwnd, aggr_cwnd = 0;
    if (xqc_bbr_w_extra_ack_gain 
        && (bbr->full_bandwidth_reached || bbr->extra_ack_in_startup))
    {
        max_aggr_cwnd = xqc_bbr_max_bw(bbr) * xqc_bbr_w_max_extra_ack_time;
        aggr_cwnd = xqc_bbr_w_extra_ack_gain * xqc_bbr_extra_ack(bbr);
        aggr_cwnd = xqc_min(aggr_cwnd, max_aggr_cwnd);
    }
    return aggr_cwnd;
}

static void 
xqc_update_ack_aggregation(xqc_bbr_w_t *bbr, xqc_sample_t *sampler)
{
    uint32_t epoch, expected_ack, extra_ack;
    uint32_t extra_ack_win_thresh = bbr->extra_ack_win_len;
    if (!xqc_bbr_w_extra_ack_gain || sampler->delivered <= 0 
        || sampler->interval <= 0 || sampler->acked <= 0) 
    {
        return;
    } 

    if (bbr->round_start) {
        bbr->extra_ack_round_rtt += 1;
        if (bbr->extra_ack_in_startup && !bbr->full_bandwidth_reached) {
            extra_ack_win_thresh = bbr->extra_ack_win_len_in_startup;
        }
        if (bbr->extra_ack_round_rtt >= extra_ack_win_thresh) {
            bbr->extra_ack_round_rtt = 0;
            bbr->extra_ack_idx = bbr->extra_ack_idx ? 0 : 1;
            bbr->extra_ack[bbr->extra_ack_idx] = 0;
        }
    }

    epoch = sampler->now - bbr->extra_ack_stamp;
    expected_ack = ((uint64_t)xqc_bbr_max_bw(bbr) * epoch) / MSEC2SEC;

    if (bbr->epoch_ack <= expected_ack 
        || (bbr->epoch_ack + sampler->acked 
            >= xqc_bbr_w_ack_epoch_acked_reset_thresh))
    {
        bbr->epoch_ack = 0;
        bbr->extra_ack_stamp = sampler->now;
        expected_ack = 0;
    }
    uint32_t cap = 0xFFFFFU * XQC_BBR_W_MAX_DATAGRAMSIZE;
    /* Compute excess data delivered, beyond what was expected. */
    bbr->epoch_ack = xqc_min(cap, bbr->epoch_ack + sampler->acked);
    extra_ack = bbr->epoch_ack - expected_ack;
    extra_ack = xqc_min(extra_ack, bbr->congestion_window);

    if (extra_ack > bbr->extra_ack[bbr->extra_ack_idx]) {
        bbr->extra_ack[bbr->extra_ack_idx] = extra_ack;
    }   
}

static void 
xqc_bbr_check_full_bw_reached(xqc_bbr_w_t *bbr, xqc_sample_t *sampler)
{
    /*
     * we MUST only check whether full bw is reached ONCE per RTT!!! 
     * Otherwise, startup may end too early due to multiple ACKs arrive in a RTT.
     */
    if (!bbr->round_start || bbr->full_bandwidth_reached 
        || sampler->is_app_limited)
    {
        return;
    }

    if (bbr->enable_expect_bw && xqc_bbr_max_bw(bbr) >= bbr->expect_bw) {
        bbr->full_bandwidth_reached = TRUE;
        return;
    }

    uint32_t bw_thresh = bbr->last_bandwidth * xqc_bbr_w_fullbw_thresh;
    if (xqc_bbr_max_bw(bbr) >= bw_thresh) {
        bbr->last_bandwidth = xqc_bbr_max_bw(bbr);
        bbr->full_bandwidth_cnt = 0;
        return;
    }
    ++bbr->full_bandwidth_cnt;
    bbr->full_bandwidth_reached = bbr->full_bandwidth_cnt >= xqc_bbr_w_fullbw_cnt;
}

static void 
xqc_bbr_enter_drain(xqc_bbr_w_t *bbr)
{
    bbr->mode = BBR_DRAIN;
    bbr->pacing_gain = xqc_bbr_w_drain_gain;
    bbr->cwnd_gain = xqc_bbr2_w_startup_cwnd_gain;
}

static void 
xqc_bbr_enter_probe_bw(xqc_bbr_w_t *bbr, xqc_sample_t *sampler)
{
    bbr->mode = BBR_PROBE_BW;
    bbr->cwnd_gain = xqc_bbr_w_cwnd_gain;
    bbr->cycle_idx = random() % (XQC_BBR_W_CYCLE_LENGTH - 1);
    bbr->cycle_idx = bbr->cycle_idx == 0 ? bbr->cycle_idx : bbr->cycle_idx + 1;
    bbr->pacing_gain = xqc_bbr_get_pacing_gain(bbr, bbr->cycle_idx);
    bbr->cycle_start_stamp = sampler->now;
    bbr->last_cycle_start = sampler->now;
}

static void 
xqc_bbr_check_drain(xqc_bbr_w_t *bbr, xqc_sample_t *sampler)
{
    if (bbr->mode == BBR_STARTUP && bbr->full_bandwidth_reached) {
        xqc_bbr_enter_drain(bbr);
    }
    if (bbr->mode == BBR_DRAIN 
        && sampler->bytes_inflight <= xqc_bbr_w_target_cwnd(bbr, 1.0)) {
#if XQC_BBR_W_RTTVAR_COMPENSATION_ENABLED
        bbr->rtt_compensation_thresh = xqc_bbr_rtt_compensation_thresh;
#endif
        xqc_bbr_enter_probe_bw(bbr, sampler);
    }
        
}

static void 
xqc_bbr_enter_probe_rtt(xqc_bbr_w_t *bbr)
{
    bbr->mode = BBR_PROBE_RTT;
    bbr->pacing_gain = 1;
    bbr->cwnd_gain = 1;
}

static void 
xqc_bbr_restore_cwnd(xqc_bbr_w_t *bbr)
{
    bbr->congestion_window = xqc_max(bbr->congestion_window, bbr->prior_cwnd);
}

static void 
xqc_bbr_exit_probe_rtt(xqc_bbr_w_t *bbr, xqc_sample_t *sampler)
{
    if (bbr->full_bandwidth_reached) {
        xqc_bbr_enter_probe_bw(bbr, sampler);

    } else {
        xqc_bbr_enter_startup(bbr);
    }
}

static void 
xqc_bbr_check_probe_rtt_done(xqc_bbr_w_t *bbr, xqc_sample_t *sampler)
{
    if (!bbr->probe_rtt_round_done_stamp 
        || sampler->now < bbr->probe_rtt_round_done_stamp) 
    {
        return;
    }
    /* schedule the next probeRTT round */
    bbr->probe_rtt_min_us_stamp = sampler->now;
    xqc_bbr_restore_cwnd(bbr);
    xqc_bbr_exit_probe_rtt(bbr, sampler);
}

static uint32_t 
xqc_bbr_probe_rtt_cwnd(xqc_bbr_w_t *bbr)
{
    if (xqc_bbr_w_probe_rtt_gain == 0) {
        return xqc_bbr_w_min_cwnd;
    }
    return xqc_max(xqc_bbr_w_min_cwnd, xqc_bbr_w_target_cwnd(bbr, xqc_bbr_w_probe_rtt_gain));
}

static void 
xqc_bbr_update_min_rtt(xqc_bbr_w_t *bbr, xqc_sample_t *sampler)
{
    bool probe_rtt_expired, min_rtt_expired;
    probe_rtt_expired = sampler->now > (bbr->probe_rtt_min_us_stamp +  
        xqc_bbr2_w_probertt_win_size_us);
    if (sampler->rtt <= bbr->probe_rtt_min_us || probe_rtt_expired) {
        xqc_log(sampler->send_ctl->ctl_conn->log, XQC_LOG_DEBUG, "|probertt expire|rtt:%ui, old_rtt:%ui|",
                sampler->rtt,
                bbr->probe_rtt_min_us);
        bbr->probe_rtt_min_us = sampler->rtt;
        bbr->probe_rtt_min_us_stamp = sampler->now;
    }
    min_rtt_expired = sampler->now > 
                      (bbr->min_rtt_stamp + xqc_bbr2_w_minrtt_win_size_us);
    bbr->min_rtt_expired = min_rtt_expired;
    if (bbr->probe_rtt_min_us <= bbr->min_rtt || min_rtt_expired) {
        xqc_log(sampler->send_ctl->ctl_conn->log, XQC_LOG_DEBUG, "|minrtt expire|rtt:%ui, old_rtt:%ui|",
                bbr->probe_rtt_min_us,
                bbr->min_rtt);
        if (bbr->probe_rtt_min_us_stamp != bbr->min_rtt_stamp
            || min_rtt_expired)
        {
            /*
             * We should remove additional cwnd if background buffer-fillers 
             * have gone away or we have increased our target_cwnd by increasing
             * min_rtt.
             */
            bbr->snd_cwnd_cnt_bytes = 0;
            bbr->beyond_target_cwnd = 0;
            /* If we have increased min_rtt, we should reset our accelerating factor. */
            if (min_rtt_expired) {
                bbr->ai_scale = 1;
                bbr->ai_scale_accumulated_bytes = 0;
            }
        }
        bbr->min_rtt = bbr->probe_rtt_min_us;
        bbr->min_rtt_stamp = bbr->probe_rtt_min_us_stamp;
    }

    /* Restart after idle ends only once we process a new S/ACK for data */
    if (sampler->delivered > 0) {
        bbr->idle_restart = 0;
    }
}

static uint64_t 
xqc_bbr_get_min_rtt(xqc_bbr_w_t *bbr)
{
    return bbr->min_rtt == 0 ? xqc_bbr_w_initial_rtt_ms * 1000 : bbr->min_rtt;
}

static void 
_xqc_bbr_set_pacing_rate_helper(xqc_bbr_w_t *bbr, float pacing_gain)
{
    return;
}

static void 
xqc_bbr_set_pacing_rate(xqc_bbr_w_t *bbr, xqc_sample_t *sampler)
{
    if (!bbr->has_srtt && sampler->srtt) {
        xqc_bbr_w_init_pacing_rate(bbr, sampler);
    }
    _xqc_bbr_set_pacing_rate_helper(bbr, bbr->pacing_gain);
    if (bbr->pacing_rate == 0) {
        xqc_bbr_w_init_pacing_rate(bbr, sampler);
        xqc_log(sampler->send_ctl->ctl_conn->log, XQC_LOG_WARN,
                "|rate reached 0|reset pacing_rate:%ud|", bbr->pacing_rate);
    }
}

static void 
xqc_bbr_modulate_cwnd_for_recovery(xqc_bbr_w_t *bbr, xqc_sample_t *sampler)
{
    xqc_log(sampler->send_ctl->ctl_conn->log, XQC_LOG_DEBUG, 
            "|before ModulateCwndForRecovery|cwnd:%ud"
            "|packet_lost:%ud|acked:%ud|po_sent_time:%ui"
            "|recovery:%ud|recovery_start:%ui|packet_conservation:%ud|"
            "next_round_delivered:%ud|",
            bbr->congestion_window, sampler->loss, sampler->acked, 
            sampler->po_sent_time, bbr->recovery_mode, bbr->recovery_start_time,
            bbr->packet_conservation, bbr->next_round_delivered);
    if (sampler->loss > 0) {
        /* to avoid underflow of unsigned numbers */
        if (bbr->congestion_window 
            > (sampler->loss * XQC_BBR_W_MAX_DATAGRAMSIZE)) 
        {
            bbr->congestion_window -= sampler->loss * XQC_BBR_W_MAX_DATAGRAMSIZE;

        } else {
            bbr->congestion_window = 0;
        }
        bbr->congestion_window = xqc_max(bbr->congestion_window, 
            XQC_BBR_W_MAX_DATAGRAMSIZE);
    }
    if (bbr->just_enter_recovery_mode) {
        bbr->just_enter_recovery_mode = FALSE;
        bbr->packet_conservation = 1;
        bbr->next_round_delivered = sampler->total_acked;
        bbr->congestion_window = sampler->send_ctl->ctl_bytes_in_flight + 
                                 xqc_max(sampler->acked, 
                                         XQC_BBR_W_MAX_DATAGRAMSIZE);

    } else if (bbr->just_exit_recovery_mode) {
        /* 
         * exit recovery mode once any packet sent
         * during the recovery epoch is acked.
         */
        bbr->just_exit_recovery_mode = FALSE;
        bbr->packet_conservation = 0;
        xqc_bbr_restore_cwnd(bbr);
    }
    if (bbr->packet_conservation) {
        bbr->congestion_window = xqc_max(bbr->congestion_window, 
                                 sampler->send_ctl->ctl_bytes_in_flight + 
                                 sampler->acked);
    }
    xqc_log(sampler->send_ctl->ctl_conn->log, XQC_LOG_DEBUG, 
            "|after ModulateCwndForRecovery|cwnd:%ud"
            "|packet_lost:%ud|acked:%ud|po_sent_time:%ui"
            "|recovery:%ud|recovery_start:%ui|packet_conservation:%ud|"
            "next_round_delivered:%ud|",
            bbr->congestion_window, sampler->loss, sampler->acked, 
            sampler->po_sent_time, bbr->recovery_mode, bbr->recovery_start_time,
            bbr->packet_conservation, bbr->next_round_delivered);
}

static void 
xqc_bbr_reset_cwnd(void *cong_ctl)
{
    return;
}


static void
xqc_bbr_cong_avoid_ai(xqc_bbr_w_t *bbr, uint32_t cwnd, uint32_t acked)
{
    /* growing 2x cwnd at maximum per RTT */
    uint32_t cwnd_thresh = xqc_max(XQC_BBR_W_MAX_DATAGRAMSIZE, 
                                   cwnd / bbr->ai_scale);
    if (bbr->snd_cwnd_cnt_bytes >= cwnd_thresh) {
        bbr->beyond_target_cwnd += XQC_BBR_W_MAX_DATAGRAMSIZE;
        bbr->snd_cwnd_cnt_bytes = 0;
    }
    bbr->snd_cwnd_cnt_bytes += acked;
    if (bbr->snd_cwnd_cnt_bytes >= cwnd_thresh) {
        uint32_t delta = bbr->snd_cwnd_cnt_bytes / cwnd_thresh;
        bbr->snd_cwnd_cnt_bytes -= delta * cwnd_thresh;
        bbr->beyond_target_cwnd += delta * XQC_BBR_W_MAX_DATAGRAMSIZE;
    }
    /* update ai_scale: we want to double ai_scale when enough data is acked. */
    bbr->ai_scale_accumulated_bytes += acked;
    if (bbr->ai_scale_accumulated_bytes >= cwnd_thresh) {
        uint32_t delta = bbr->ai_scale_accumulated_bytes / cwnd_thresh;
        bbr->ai_scale = xqc_min(bbr->ai_scale + delta, XQC_BBR_W_MAX_AI_SCALE);
        bbr->ai_scale_accumulated_bytes -= delta * cwnd_thresh;
    }
}

static void 
xqc_bbr_set_cwnd(xqc_bbr_w_t *bbr, xqc_sample_t *sampler)
{
    if (sampler->acked != 0) {
        xqc_send_ctl_t *send_ctl = sampler->send_ctl;

        uint32_t target_cwnd, extra_cwnd;
        target_cwnd = xqc_bbr_w_target_cwnd(bbr, bbr->cwnd_gain);
        extra_cwnd = xqc_bbr_ack_aggregation_cwnd(bbr);
        xqc_log(send_ctl->ctl_conn->log, XQC_LOG_DEBUG,
                "|xqc_bbr_set_cwnd|target_cwnd:%ud|extra_cwnd:%ud|", 
                target_cwnd, extra_cwnd);
        target_cwnd += extra_cwnd;

#if XQC_BBR_W_RTTVAR_COMPENSATION_ENABLED
        if (bbr->rttvar_compensation_on) {
            uint32_t cwnd_for_rttvar;
            cwnd_for_rttvar = xqc_bbr_compensate_cwnd_for_rttvar(bbr, sampler);
            target_cwnd += cwnd_for_rttvar;
            xqc_log(send_ctl->ctl_conn->log, XQC_LOG_DEBUG,
                    "|rttvar compensation|cwnd_for_rttvar: %ud|", cwnd_for_rttvar);
        }
#endif

        xqc_bbr_modulate_cwnd_for_recovery(bbr, sampler);
        if (!bbr->packet_conservation) {
            if (bbr->full_bandwidth_reached) {
                if ((bbr->congestion_window + sampler->acked 
                    >= (target_cwnd + bbr->beyond_target_cwnd))
                    && sampler->send_ctl->ctl_is_cwnd_limited)
                {
                    /* We are limited by target_cwnd */
                    xqc_bbr_cong_avoid_ai(bbr, xqc_bbr_w_target_cwnd(bbr, 1.0), sampler->acked);   
                }
                /* additive increasing target_cwnd */
                target_cwnd += bbr->beyond_target_cwnd;
                xqc_conn_log(sampler->send_ctl->ctl_conn, XQC_LOG_DEBUG, 
                            "|cwnd: %ud|target_cwnd: %ud|acked: %ud"
                            "|cwnd_cnt: %ud|beyond_target_cwnd: %ud|",
                            bbr->congestion_window,
                            target_cwnd,
                            sampler->acked,
                            bbr->snd_cwnd_cnt_bytes,
                            bbr->beyond_target_cwnd);
                bbr->congestion_window = xqc_min(target_cwnd, 
                                                bbr->congestion_window + 
                                                sampler->acked);

            } else if (bbr->congestion_window < target_cwnd 
                    || send_ctl->ctl_delivered < bbr->initial_congestion_window)
            {
                bbr->congestion_window += sampler->acked;
            }
        }
        bbr->congestion_window = xqc_max(bbr->congestion_window, xqc_bbr_w_min_cwnd);
    }
    if (bbr->mode == BBR_PROBE_RTT) {
        bbr->congestion_window = xqc_min(bbr->congestion_window, 
                                         xqc_bbr_probe_rtt_cwnd(bbr));
    }
}

static void 
xqc_bbr_on_lost(void *cong_ctl, xqc_usec_t lost_sent_time)
{
    xqc_bbr_w_t *bbr = (xqc_bbr_w_t *)cong_ctl;
    /* 
     * Unlike the definition of "recovery epoch" for loss-based CCs, 
     * for the sake of resistance to losses, we MUST refresh the end of a 
     * recovery epoch if further losses happen in the epoch. Otherwise, the
     * ability of BBR to sustain network where high loss rate presents 
     * is hampered because of frequently entering packet conservation state. 
     */
    bbr->recovery_start_time = xqc_monotonic_timestamp();
    /* If losses happened, we do not increase cwnd beyond target_cwnd. */
    bbr->snd_cwnd_cnt_bytes = 0;
    bbr->beyond_target_cwnd = 0;
    bbr->ai_scale = 1;
    bbr->ai_scale_accumulated_bytes = 0;
}

static void 
xqc_bbr_set_or_restore_pacing_gain_in_startup(void *cong_ctl)
{
    xqc_bbr_w_t *bbr = (xqc_bbr_w_t *)cong_ctl;
    if (bbr->mode == BBR_STARTUP) {
        if (bbr->recovery_mode == BBR_IN_RECOVERY) {
            bbr->pacing_gain = xqc_bbr2_w_startup_pacing_gain_on_lost;
        }
        if (bbr->recovery_mode == BBR_NOT_IN_RECOVERY) {
            bbr->pacing_gain = xqc_bbr_w_high_gain;
        }
    }
}

static void
xqc_bbr_update_recovery_mode(void *cong_ctl, xqc_sample_t *sampler)
{
    xqc_bbr_w_t *bbr = (xqc_bbr_w_t *)cong_ctl;
    if (sampler->po_sent_time <= bbr->recovery_start_time 
        && bbr->recovery_mode == BBR_NOT_IN_RECOVERY)
    {
        bbr->just_enter_recovery_mode = TRUE;
        bbr->recovery_mode = BBR_IN_RECOVERY;

    } 
    else if (sampler->po_sent_time > bbr->recovery_start_time 
             && bbr->recovery_mode == BBR_IN_RECOVERY)
    {
        /* exit recovery mode once any packet sent during the recovery epoch is acked. */
        bbr->recovery_mode = BBR_NOT_IN_RECOVERY;
        bbr->just_exit_recovery_mode = TRUE;
        bbr->recovery_start_time = 0;
    }
}

static void 
xqc_bbr_on_ack(void *cong_ctl, xqc_sample_t *sampler)
{
    xqc_bbr_w_t *bbr = (xqc_bbr_w_t *)(cong_ctl);
#if XQC_BBR_W_RTTVAR_COMPENSATION_ENABLED
    /* maintain windowed max rtt here */
    if (bbr->rttvar_compensation_on) {
        if (sampler->rtt >= 0) {
            xqc_usec_t last_max_rtt = xqc_win_filter_get_u64(&bbr->max_rtt);
            xqc_win_filter_max_u64(&bbr->max_rtt, bbr->max_rtt_win_len,
                                   bbr->round_cnt, sampler->rtt);
            xqc_log(sampler->send_ctl->ctl_conn->log, XQC_LOG_DEBUG, 
                    "|rttvar_compensation|windowed max rtt info|"
                    "rtt %ui, last_max %ui, max %ui|",
                    sampler->rtt, last_max_rtt, 
                    xqc_win_filter_get_u64(&bbr->max_rtt));
        }
    }
#endif
    /* Update model and state */
    //xqc_bbr_update_bandwidth(bbr, sampler);
    //xqc_update_ack_aggregation(bbr, sampler);
    //xqc_bbr_update_cycle_phase(bbr, sampler);
    //xqc_bbr_check_full_bw_reached(bbr, sampler);
    //xqc_bbr_check_drain(bbr, sampler);
    xqc_bbr_update_min_rtt(bbr, sampler);

    //xqc_bbr_update_recovery_mode(bbr, sampler);
    // if (xqc_bbr2_slow_down_startup_on_lost) {
    //    xqc_bbr_set_or_restore_pacing_gain_in_startup(bbr);
    //}
    /* Update control parameter */
    //xqc_bbr_set_pacing_rate(bbr, sampler);
    //xqc_bbr_set_cwnd(bbr, sampler);
}

static uint64_t 
xqc_bbr_w_get_cwnd(void *cong_ctl)
{
    xqc_bbr_w_t *bbr = (xqc_bbr_w_t *)(cong_ctl);
    return bbr->congestion_window;
}

static uint32_t 
xqc_bbr_w_get_pacing_rate(void *cong_ctl)
{
    xqc_bbr_w_t *bbr = (xqc_bbr_w_t *)(cong_ctl);
    return bbr->pacing_rate;
}

static uint32_t 
xqc_bbr_w_get_bandwidth(void *cong_ctl)
{
    xqc_bbr_w_t *bbr = (xqc_bbr_w_t *)(cong_ctl);
    return xqc_bbr_max_bw(bbr);
}

static void 
xqc_bbr_restart_from_idle(void *cong_ctl, uint64_t conn_delivered)
{
    xqc_bbr_w_t *bbr = (xqc_bbr_w_t *)(cong_ctl);
    uint32_t rate;
    uint64_t now = xqc_monotonic_timestamp();
    bbr->idle_restart = 1;
    bbr->extra_ack_stamp = now;
    bbr->epoch_ack = 0;
    xqc_sample_t sampler = {.now = now, .total_acked = conn_delivered};

    if (bbr->mode == BBR_PROBE_BW) {
        _xqc_bbr_set_pacing_rate_helper(bbr, 1.0);
        if (bbr->pacing_rate == 0) {
            xqc_bbr_w_init_pacing_rate(bbr, &sampler);
        }
    } else if (bbr->mode == BBR_PROBE_RTT) {
        xqc_bbr_check_probe_rtt_done(bbr, &sampler);
    }
}

/* These functions are mainly for debug */
static uint8_t 
xqc_bbr_info_mode(void *cong)
{
    xqc_bbr_w_t *bbr = (xqc_bbr_w_t *)cong;
    return bbr->mode;
}

static uint64_t 
xqc_bbr_info_min_rtt(void *cong)
{
    xqc_bbr_w_t *bbr = (xqc_bbr_w_t *)cong;
    return bbr->min_rtt;
}

static uint8_t 
xqc_bbr_info_idle_restart(void *cong)
{
    xqc_bbr_w_t *bbr = (xqc_bbr_w_t *)cong;
    return bbr->idle_restart;
}

static uint8_t 
xqc_bbr_info_full_bw_reached(void *cong)
{
    xqc_bbr_w_t *bbr = (xqc_bbr_w_t *)cong;
    return bbr->full_bandwidth_reached;
}

static uint8_t 
xqc_bbr_info_recovery_mode(void *cong)
{
    xqc_bbr_w_t *bbr = (xqc_bbr_w_t *)cong;
    return bbr->recovery_mode;
}

static uint64_t 
xqc_bbr_info_recovery_start_time(void *cong)
{
    xqc_bbr_w_t *bbr = (xqc_bbr_w_t *)cong;
    return bbr->recovery_start_time;
}

static uint8_t 
xqc_bbr_info_packet_conservation(void *cong)
{
    xqc_bbr_w_t *bbr = (xqc_bbr_w_t *)cong;
    return bbr->packet_conservation;
}

static uint8_t 
xqc_bbr_info_round_start(void *cong)
{
    xqc_bbr_w_t *bbr = (xqc_bbr_w_t *)cong;
    return bbr->round_start;
}

static float 
xqc_bbr_info_pacing_gain(void *cong)
{
    xqc_bbr_w_t *bbr = (xqc_bbr_w_t *)cong;
    return bbr->pacing_gain;
}

static float 
xqc_bbr_info_cwnd_gain(void *cong)
{
    xqc_bbr_w_t *bbr = (xqc_bbr_w_t *)cong;
    return bbr->cwnd_gain;
}

static int
xqc_bbr_in_recovery(void *cong) {
    xqc_bbr_w_t *bbr = (xqc_bbr_w_t *)cong;
    return bbr->recovery_start_time > 0;
}

void xqc_bbr_w_renew_cwnd_srtt(void *cong_ctl, uint64_t cwnd, uint64_t pacing_rate, uint64_t bw, uint64_t queue_size, uint64_t srtt, xqc_sample_t *sampler)
{
    xqc_bbr_w_t *bbr = (xqc_bbr_w_t *)cong_ctl;
    uint64_t pacing_rate_new = 0;
    /*
    if((bw > 0 && ((double)(queue_size) + sampler->bytes_inflight )/((double)(bbr->min_rtt * bw / MSEC2SEC)) > 0.5))
    {
        pacing_rate_new = pacing_rate * 0.75;
    }
    else
    {
        //pacing_rate_new = (uint64_t)(pacing_rate * (1.0 - ((double)(queue_size * 1000000))/((double)(bbr->min_rtt * pacing_rate))));
        pacing_rate_new = pacing_rate * 1.05;
    }*/
    //double qlen_ratio = ((double)queue_size + sampler->bytes_inflight)  / (double)(bbr->min_rtt * bw / MSEC2SEC); 
    //double qlen_ratio = (double)queue_size / (double)(srtt * bw / MSEC2SEC); 
    //double qlen_ratio = (sampler->bytes_inflight - (double)queue_size) / (double)(srtt * bw / MSEC2SEC);
    
    double qlen_ratio = (double)queue_size / (double)(bbr->min_rtt * bw / MSEC2SEC);
    //double qlen_ratio = ((double) queue_size + pacing_rate * bbr->min_rtt / MSEC2SEC) / (double)(bbr->min_rtt * bw / MSEC2SEC);
    //double ratio = (qlen_ratio > 1.1) ? qlen_ratio : (1 / 0.95);
    double ratio = xqc_max((1 - qlen_ratio*0.8), 0.5);
    bbr->last_qlen_ratio = bbr->last_qlen_ratio * ewma_parameter + (1 - ewma_parameter) * ratio;
    if (pacing_rate > 2*bw)
    {
        bbr->last_qlen_ratio = 1.2;
    }
    //bbr->last_qlen_ratio = xqc_min(bbr->last_qlen_ratio, 5);
    //pacing_rate_new = xqc_max((1 - 0.8*bbr->last_qlen_ratio), 0.5) * pacing_rate;
    pacing_rate_new = pacing_rate * bbr->last_qlen_ratio;
    
    //bbr->pacing_rate = ((pacing_rate / bw) > 1.5) ? pacing_rate : pacing_rate_new;
    bbr->pacing_rate = pacing_rate_new;
    //bbr->pacing_rate = pacing_rate;
    bbr->congestion_window = cwnd * bbr->min_rtt/srtt;
    xqc_bbr_update_bandwidth(bbr, sampler, pacing_rate_new);
    bbr->cycle_idx =2;
    //xqc_bbr_set_cwnd(bbr, sampler);
    //printf("server pacing_rate: %ld, cwnd:%ld, srtt:%ld, queue_size:%ld, pacing_rate_new:%lu\n",pacing_rate, cwnd, srtt, queue_size, pacing_rate_new);
    xqc_log(sampler->send_ctl->ctl_conn->log, XQC_LOG_TEST, "server pacing_rate: %ud cwnd: %ud srtt: %ud queue_size: %ud pacing_rate_new: %ud bw: %ud qlen_ratio: %.4f inflight: %.4f",
            pacing_rate, srtt, bbr->min_rtt, queue_size, bbr->pacing_rate, bw, bbr->last_qlen_ratio, ratio);
    //bbr->bandwidth = pacing_rate;
    //bbr->congestion_window = cwnd;
}

static xqc_bbr_info_interface_t xqc_bbr_info_cb = {
    .mode                 = xqc_bbr_info_mode,
    .min_rtt              = xqc_bbr_info_min_rtt,
    .idle_restart         = xqc_bbr_info_idle_restart,
    .full_bw_reached      = xqc_bbr_info_full_bw_reached,
    .recovery_mode        = xqc_bbr_info_recovery_mode,
    .recovery_start_time  = xqc_bbr_info_recovery_start_time,
    .packet_conservation  = xqc_bbr_info_packet_conservation,
    .round_start          = xqc_bbr_info_round_start,
    .pacing_gain          = xqc_bbr_info_pacing_gain,
    .cwnd_gain            = xqc_bbr_info_cwnd_gain,
};

const xqc_cong_ctrl_callback_t xqc_bbr_w_cb = {
    .xqc_cong_ctl_size                    = xqc_bbr_w_size,
    .xqc_cong_ctl_init_bbr                = xqc_bbr_init,
    .xqc_cong_ctl_on_ack_multiple_pkts    = xqc_bbr_on_ack,
    .xqc_cong_ctl_get_cwnd                = xqc_bbr_w_get_cwnd,
    .xqc_cong_ctl_get_pacing_rate         = xqc_bbr_w_get_pacing_rate,
    .xqc_cong_ctl_get_bandwidth_estimate  = xqc_bbr_w_get_bandwidth,
    .xqc_cong_ctl_restart_from_idle       = xqc_bbr_restart_from_idle,
    .xqc_cong_ctl_on_lost                 = xqc_bbr_on_lost,
    .xqc_cong_ctl_reset_cwnd              = xqc_bbr_reset_cwnd,
    .xqc_cong_ctl_info_cb                 = &xqc_bbr_info_cb,
    .xqc_cong_ctl_in_recovery             = xqc_bbr_in_recovery,
    .xqc_renew_cwnd_srtt                  = xqc_bbr_w_renew_cwnd_srtt,
};
