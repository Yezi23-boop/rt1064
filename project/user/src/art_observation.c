#include "art_observation.h"
#include "executor.h"
#include "motion_math.h"
#include "openart_uart.h"
#include <string.h>

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

    if(0 == batch)
    {
        return 0u;
    }
    while(0u != openart_get_requested_center_sample(&col_q, &row_q))
    {
        if(batch->count < ART_CENTER_SAMPLE_COUNT)
        {
            batch->col_q[batch->count] = col_q;
            batch->row_q[batch->count] = row_q;
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
