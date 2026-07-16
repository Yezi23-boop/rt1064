#ifndef _art_observation_h_
#define _art_observation_h_

#include "drive_config.h"
#include "zf_common_typedef.h"

typedef struct
{
    uint16 col_q[ART_CENTER_SAMPLE_COUNT];
    uint16 row_q[ART_CENTER_SAMPLE_COUNT];
    uint16 yaw_q[ART_CENTER_SAMPLE_COUNT];
    uint8 yaw_valid[ART_CENTER_SAMPLE_COUNT];
    uint8 count;
} art_center_batch_struct;

typedef struct
{
    uint8 active;
    uint8 ready;
    uint8 box_row;
    uint8 box_col;
} art_box_observation_session_struct;

void art_center_batch_reset(art_center_batch_struct *batch);
uint8 art_center_batch_collect(art_center_batch_struct *batch);
uint8 art_center_batch_get_median(const art_center_batch_struct *batch,
                                  uint16 *col_q,
                                  uint16 *row_q);
uint8 art_center_batch_get_yaw_deg(const art_center_batch_struct *batch,
                                   float *yaw_deg,
                                   uint8 *accepted_count);
uint8 art_center_batch_apply_to_executor(const art_center_batch_struct *batch);

void art_box_observation_session_reset(art_box_observation_session_struct *session);
void art_box_observation_session_request(art_box_observation_session_struct *session,
                                         uint8 box_row,
                                         uint8 box_col);
uint8 art_box_observation_session_collect(art_box_observation_session_struct *session);

#endif
