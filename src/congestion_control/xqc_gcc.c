#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <xquic/xquic.h>
#include "src/common/xqc_config.h"
#include "src/common/xqc_time.h"
#include "src/congestion_control/xqc_gcc.h"
#include "src/transport/xqc_send_ctl.h"

#define XQC_GCC_MSS                   (XQC_MSS)
#define XQC_GCC_MIN_WIN               (4 * XQC_GCC_MSS)
#define XQC_GCC_MAX_INIT_WIN          (100 * XQC_GCC_MSS)
#define XQC_GCC_INIT_WIN              (32 * XQC_GCC_MSS)
#define XQC_GCC_RTT_MIN_WINDOW        (10000000) /* 10s */
#define XQC_GCC_RTT_MAX_WINDOW        (4) /* 4 RTTs */
#define XQC_GCC_RTT_STA_WINDOW        (0.5) /* 0.5 RTT */
/* mode switching threshold: 5 RTTs */
#define XQC_GCC_MS_THRESHOLD          (5) 
/* the default value recommended by Facebook */
#define XQC_GCC_INF_U64               (~0ULL)
#define XQC_GCC_INIT_VELOCITY         (1.0)
#define XQC_GCC_MAX_RATE              (1.0 * (~0ULL))
#define XQC_GCC_USEC2SEC              (1000000)

#define XQC_GCC_LOG_STATE(comment_str, log_level) \
do { \
    xqc_log(gcc->ctl_ctx->ctl_conn->log, log_level,  \
            "|gcc|comment_str|init_cwnd_bytes:%ui|cwnd_bytes:%ui|" \
            "rtt_min:%ui|rtt_max:%ui|rtt_standing:%ui|" \
            "velocity:%.4f|curr_dir:%d|prev_dir:%d|same_dir_cnt:%d|mode:%d|" \
            "t_last_delay_min:%ui|slow_start:%d|pacing_rate:%ui|" \
            "recovery_start_time:%ui|round_cnt:%ud|next_round_delivered:%ui|" \
            "round_start:%d|" \
            "last_round_cwnd_bytes:%ui|cwnd_adjustment_accumulated:%i|", \
            gcc->init_cwnd_bytes, gcc->cwnd_bytes, \
            xqc_win_filter_get(&gcc->rtt_min), \
            xqc_win_filter_get(&gcc->rtt_max), \
            xqc_win_filter_get(&gcc->rtt_standing), \
            gcc->v, gcc->curr_dir, gcc->prev_dir, \
            gcc->same_dir_cnt, gcc->mode, gcc->t_last_delay_min, \
            gcc->in_slow_start, gcc->pacing_rate, gcc->recovery_start_time, \
            gcc->round_cnt, gcc->next_round_delivered,  \
            gcc->round_start, \
            gcc->last_round_cwnd_bytes, gcc->cwnd_adjustment_accumulated \
        ); \
} while (0)



static void 
xqc_gcc_set_pacing_rate(xqc_gcc_t *gcc)
{
    /* 2*cwnd / rtt_standing */
    xqc_usec_t rtt_standing = xqc_win_filter_get(&gcc->rtt_standing);
    if (rtt_standing == XQC_GCC_INF_U64) {
        /* initialization */
        rtt_standing = gcc->ctl_ctx->ctl_srtt;
    }
    if (rtt_standing == 0) {
        xqc_log(gcc->ctl_ctx->ctl_conn->log, XQC_LOG_WARN, 
                "|gcc|rtt_standing_error:%ui|", rtt_standing);
        /* initialization */
        rtt_standing = XQC_kInitialRtt * 1000;
        xqc_win_filter_reset(&gcc->rtt_standing, 0, XQC_GCC_INF_U64);
    }
    gcc->pacing_rate = ((gcc->cwnd_bytes) * XQC_GCC_USEC2SEC) << 1;
    gcc->pacing_rate /= rtt_standing;
    gcc->pacing_rate = xqc_max(gcc->pacing_rate, XQC_GCC_MSS);
}

static size_t
xqc_gcc_size()
{
    return sizeof(xqc_gcc_t);
}

static void
xqc_gcc_init(void *cong, xqc_send_ctl_t *ctl_ctx, xqc_cc_params_t cc_params)
{
    xqc_gcc_t *gcc = (xqc_gcc_t*)cong;
    gcc->init_cwnd_bytes = XQC_GCC_INIT_WIN;
    xqc_win_filter_reset(&gcc->rtt_min, 0, XQC_GCC_INF_U64);
    xqc_win_filter_reset(&gcc->rtt_max, 0, 0);
    xqc_win_filter_reset(&gcc->rtt_standing, 0, XQC_GCC_INF_U64);
    gcc->v = XQC_GCC_INIT_VELOCITY;
    gcc->curr_dir = GCC_UNDEF; /* slow start */
    gcc->prev_dir = GCC_UNDEF;
    gcc->same_dir_cnt = 0;
    gcc->mode = GCC_DELAY_MODE;
    gcc->t_last_delay_min = 0;
    gcc->in_slow_start = XQC_TRUE;
    gcc->ctl_ctx = ctl_ctx;
    if (cc_params.customize_on) {
        gcc->init_cwnd_bytes = xqc_clamp(cc_params.init_cwnd * XQC_GCC_MSS,
                                          XQC_GCC_MIN_WIN,
                                          XQC_GCC_MAX_INIT_WIN);
        
    }
    gcc->cwnd_bytes = gcc->init_cwnd_bytes;
    gcc->last_round_cwnd_bytes = 0; 
    xqc_gcc_set_pacing_rate(gcc);
    gcc->recovery_start_time = 0;
    gcc->next_round_delivered = 0;
    gcc->round_cnt = 0;
    gcc->round_start = XQC_FALSE;
    gcc->cwnd_adjustment_accumulated = 0;

    XQC_GCC_LOG_STATE("initialization", XQC_LOG_DEBUG);
    return;
}

static void
xqc_gcc_on_lost(void *cong, xqc_usec_t lost_sent_time)
{
    xqc_gcc_t *gcc = (xqc_gcc_t*)cong;

    xqc_usec_t min_rtt = xqc_win_filter_get(&gcc->rtt_min);

    if(gcc->lost_time > lost_sent_time){

        gcc->pacing_rate *= 0.7;

        /* 更新拥塞窗口 */
        gcc->cwnd_bytes = gcc->pacing_rate * xqc_win_filter_get(&gcc->rtt_min) / XQC_GCC_USEC2SEC;
        gcc->lost_time = lost_sent_time + min_rtt;
    }

    return;
}

static inline void
xqc_gcc_handle_sudden_direction_change(xqc_gcc_t *gcc, 
    xqc_gcc_direction_t new_dir)
{
    gcc->v = XQC_GCC_INIT_VELOCITY;
    gcc->same_dir_cnt = 0;
    gcc->last_round_cwnd_bytes = gcc->cwnd_bytes;
    gcc->prev_dir = gcc->curr_dir;
    gcc->curr_dir = new_dir;
    xqc_log(gcc->ctl_ctx->ctl_conn->log, XQC_LOG_DEBUG, 
            "|gcc|handle_sudden_direction_change|cwnd_bytes:%ui|"
            "last_round_cwnd_bytes:%ui|curr_dir:%d|prev_dir:%d|",
            gcc->cwnd_bytes, gcc->last_round_cwnd_bytes,
            gcc->curr_dir, gcc->prev_dir);
}

static void
xqc_gcc_on_ack(void *cong, xqc_sample_t *sampler)
{
    xqc_gcc_t *gcc = (xqc_gcc_t*)cong;
    xqc_usec_t  largest_pkt_sent_time = sampler->po_sent_time;
    uint32_t    newly_acked_bytes = sampler->acked;
    xqc_usec_t  ack_recv_time = sampler->now;
    xqc_usec_t  latest_rtt = gcc->ctl_ctx->ctl_latest_rtt;
    xqc_usec_t  srtt = gcc->ctl_ctx->ctl_srtt;
    xqc_usec_t  rtt_min_win = XQC_GCC_RTT_MIN_WINDOW;
    xqc_usec_t  rtt_sta_win = XQC_GCC_RTT_STA_WINDOW * srtt;
    xqc_usec_t  rtt_max_win = XQC_GCC_RTT_MAX_WINDOW;


    xqc_win_filter_min(&gcc->rtt_min, XQC_GCC_RTT_MIN_WINDOW, sampler->now, sampler->rtt);

    xqc_usec_t min_rtt = xqc_win_filter_get(&gcc->rtt_min);
    xqc_usec_t current_rtt = sampler->rtt;
    
    //if(gcc->rate_time >ack_recv_time){
        /* 延迟趋势判断 */
        if (current_rtt - min_rtt > min_rtt * 0.3) {
            /* RTT 增加：减小 pacing rate */
            gcc->pacing_rate *= 0.8;
        } else {
            /* RTT 减少：增加 pacing rate */
            gcc->pacing_rate *= 1.2;
        }

        gcc->rate_time = ack_recv_time + min_rtt;
    //}
    

    /* 调整 pacing rate */
    gcc->pacing_rate = xqc_min(gcc->pacing_rate, XQC_GCC_MAX_RATE);

    /* 调整拥塞窗口 */
    gcc->cwnd_bytes = gcc->pacing_rate * min_rtt / XQC_GCC_USEC2SEC;



on_ack_end:
    XQC_GCC_LOG_STATE("after_on_ack", XQC_LOG_DEBUG);
    return;
}

static uint64_t
xqc_gcc_get_cwnd(void *cong)
{
    xqc_gcc_t *gcc = (xqc_gcc_t*)cong;
    return gcc->cwnd_bytes;
}

static void
xqc_gcc_reset_cwnd(void *cong)
{
    xqc_gcc_t *gcc = (xqc_gcc_t*)cong;
    /* 
     * @NOTE: We reinitialize Copa and do slow start again. After recovering 
     * from a persistent congestion event, the network path may have changed 
     * significantly. Therefore, the safest way to do congestion control is to 
     * cut the cwnd to the minimal value and re-probe the network path by 
     * slow start.
     */
    xqc_win_filter_reset(&gcc->rtt_min, 0, XQC_GCC_INF_U64);
    xqc_win_filter_reset(&gcc->rtt_max, 0, 0);
    xqc_win_filter_reset(&gcc->rtt_standing, 0, XQC_GCC_INF_U64);
    gcc->v = XQC_GCC_INIT_VELOCITY;
    gcc->curr_dir = GCC_UNDEF; /* slow start */
    gcc->prev_dir = GCC_UNDEF;
    gcc->same_dir_cnt = 0;
    gcc->mode = GCC_DELAY_MODE;
    gcc->t_last_delay_min = 0;
    gcc->in_slow_start = XQC_TRUE;
    gcc->recovery_start_time = 0;
    gcc->cwnd_adjustment_accumulated = 0;
    gcc->cwnd_bytes = XQC_GCC_MIN_WIN;
    xqc_gcc_set_pacing_rate(gcc);

    XQC_GCC_LOG_STATE("persistent_congestion", XQC_LOG_DEBUG);
    return;
}

static int
xqc_gcc_in_slow_start(void *cong)
{
    xqc_gcc_t *gcc = (xqc_gcc_t*)cong;
    return gcc->in_slow_start;
}

static void
xqc_gcc_restart_from_idle(void *cong, uint64_t arg)
{
    /* 
     * @TODO: may do something here in the future, 
     * e.g. resetting congestion state and restarting from slow start.
     */
    return;
}

static int
xqc_gcc_in_recovery(void *cong)
{
    xqc_gcc_t *gcc = (xqc_gcc_t*)cong;
    return gcc->recovery_start_time > 0;
}

/* @TODO: use u64 for pacing rate all the time */
static uint32_t
xqc_gcc_get_pacing_rate(void *cong)
{
    xqc_gcc_t *gcc = (xqc_gcc_t*)cong;
    return gcc->pacing_rate;
}

const xqc_cong_ctrl_callback_t xqc_gcc_cb = {
    .xqc_cong_ctl_size                 = xqc_gcc_size,
    .xqc_cong_ctl_init                 = xqc_gcc_init,
    .xqc_cong_ctl_on_lost              = xqc_gcc_on_lost,
    /* @TODO: rename this callback interface */
    .xqc_cong_ctl_on_ack_multiple_pkts = xqc_gcc_on_ack,
    .xqc_cong_ctl_get_cwnd             = xqc_gcc_get_cwnd,
    .xqc_cong_ctl_reset_cwnd           = xqc_gcc_reset_cwnd,
    .xqc_cong_ctl_in_slow_start        = xqc_gcc_in_slow_start,
    .xqc_cong_ctl_restart_from_idle    = xqc_gcc_restart_from_idle,
    .xqc_cong_ctl_in_recovery          = xqc_gcc_in_recovery,
    .xqc_cong_ctl_get_pacing_rate      = xqc_gcc_get_pacing_rate,
};