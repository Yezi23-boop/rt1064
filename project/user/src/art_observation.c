#include "art_observation.h"
#include "executor.h"
#include "motion_math.h"
#include "openart_uart.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI (3.1415926f)
#endif

void art_center_batch_reset(art_center_batch_struct *batch)
{
    if(0 != batch)
    {
        memset(batch, 0, sizeof(*batch));
    }
}

uint8 art_center_batch_collect(art_center_batch_struct *batch)
{
    uint16 col_q;
    uint16 row_q;
    uint16 yaw_q;
    uint8 yaw_valid;

    if(0 == batch)
    {
        return 0u;
    }
    while(0u != openart_get_requested_center_sample(&col_q, &row_q,
                                                     &yaw_q, &yaw_valid))
    {
        if(batch->count < ART_CENTER_SAMPLE_COUNT)
        {
            batch->col_q[batch->count] = col_q;
            batch->row_q[batch->count] = row_q;
            batch->yaw_q[batch->count] = yaw_q;
            batch->yaw_valid[batch->count] = yaw_valid;
            batch->count++;
        }
    }
    return (batch->count >= ART_CENTER_SAMPLE_COUNT) ? 1u : 0u;
}

uint8 art_center_batch_get_median(const art_center_batch_struct *batch,
                                  uint16 *col_q,
                                  uint16 *row_q)
{
    if((0 == batch) || (0 == col_q) || (0 == row_q) ||
       (batch->count < ART_CENTER_SAMPLE_COUNT))
    {
        return 0u;
    }
    *col_q = median_u16_values(batch->col_q, ART_CENTER_SAMPLE_COUNT);
    *row_q = median_u16_values(batch->row_q, ART_CENTER_SAMPLE_COUNT);
    return 1u;
}

uint8 art_center_batch_get_yaw_deg(const art_center_batch_struct *batch,
                                   float *yaw_deg,
                                   uint8 *accepted_count)
{
    uint8 candidate;
    uint8 sample;
    uint8 reference_index = 0u;
    uint8 valid_count = 0u;
    uint8 accepted = 0u;
    float best_score = 0.0f;
    float sin_sum = 0.0f;
    float cos_sum = 0.0f;

    if((0 == batch) || (0 == yaw_deg) ||
       (batch->count < ART_CENTER_SAMPLE_COUNT))
    {
        return 0u;
    }

    for(candidate = 0u; candidate < ART_CENTER_SAMPLE_COUNT; candidate++)
    {
        float score = 0.0f;

        if(0u == batch->yaw_valid[candidate])
        {
            continue;
        }
        valid_count++;
        for(sample = 0u; sample < ART_CENTER_SAMPLE_COUNT; sample++)
        {
            if(0u != batch->yaw_valid[sample])
            {
                score += fabsf(shortest_angle_error(
                    (float)batch->yaw_q[sample] * 0.01f,
                    (float)batch->yaw_q[candidate] * 0.01f));
            }
        }
        if((1u == valid_count) || (score < best_score))
        {
            best_score = score;
            reference_index = candidate;
        }
    }

    if(valid_count < ART_LAUNCH_YAW_MIN_VALID_SAMPLES)
    {
        return 0u;
    }

    for(sample = 0u; sample < ART_CENTER_SAMPLE_COUNT; sample++)
    {
        float sample_deg;
        float sample_rad;

        if(0u == batch->yaw_valid[sample])
        {
            continue;
        }
        sample_deg = (float)batch->yaw_q[sample] * 0.01f;
        if(fabsf(shortest_angle_error(
               sample_deg,
               (float)batch->yaw_q[reference_index] * 0.01f)) >
           ART_LAUNCH_YAW_OUTLIER_DEG)
        {
            continue;
        }
        sample_rad = sample_deg * (float)M_PI / 180.0f;
        sin_sum += sinf(sample_rad);
        cos_sum += cosf(sample_rad);
        accepted++;
    }

    if(accepted < ART_LAUNCH_YAW_MIN_VALID_SAMPLES)
    {
        return 0u;
    }

    *yaw_deg = atan2f(sin_sum, cos_sum) * 180.0f / (float)M_PI;
    if(*yaw_deg < 0.0f)
    {
        *yaw_deg += 360.0f;
    }
    if(0 != accepted_count)
    {
        *accepted_count = accepted;
    }
    return 1u;
}

uint8 art_center_batch_apply_to_executor(const art_center_batch_struct *batch)
{
    uint8 index;
    uint8 ready = 0u;

    if((0 == batch) || (batch->count < ART_CENTER_SAMPLE_COUNT))
    {
        return 0u;
    }
    for(index = 0u; index < ART_CENTER_SAMPLE_COUNT; index++)
    {
        if(0u != executor_apply_art_player_center(batch->col_q[index],
                                                  batch->row_q[index],
                                                  (uint32)(index + 1u)))
        {
            ready = 1u;
        }
    }
    return ready;
}

void art_box_observation_session_reset(art_box_observation_session_struct *session)
{
    if(0 != session)
    {
        memset(session, 0, sizeof(*session));
    }
}

void art_box_observation_session_request(art_box_observation_session_struct *session,
                                         uint8 box_row,
                                         uint8 box_col)
{
    if(0 == session)
    {
        return;
    }
    executor_reset_art_box_observation_samples();
    openart_request_observation(box_row, box_col);
    session->active = 1u;
    session->ready = 0u;
    session->box_row = box_row;
    session->box_col = box_col;
}

uint8 art_box_observation_session_collect(art_box_observation_session_struct *session)
{
    openart_observation_sample_struct sample;

    if(0 == session)
    {
        return 0u;
    }
    while(0u != openart_get_observation_sample(&sample))
    {
        if(0u != executor_apply_art_box_observation(sample.car_col_q,
                                                    sample.car_row_q,
                                                    sample.box_col_q,
                                                    sample.box_row_q))
        {
            session->active = 0u;
            session->ready = 1u;
        }
    }
    return session->ready;
}
