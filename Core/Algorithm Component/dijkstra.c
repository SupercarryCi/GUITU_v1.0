/*
 * LoRa indoor map and return navigation core.
 *
 * Input line format:
 *   x_mm,y_mm,z_mm,yaw_cdeg
 *   R
 *
 * This file is intentionally self-contained for MCU integration. It uses fixed
 * arrays, no malloc, no file system, and no GUI code.
 */
#include "dijkstra.h"

#include <stdlib.h>
#include <string.h>
#include <math.h>

static float nav_absf(float value)
{
    return value >= 0.0f ? value : -value;
}

static float nav_maxf(float a, float b)
{
    return a > b ? a : b;
}

static float nav_clampf(float value, float lo, float hi)
{
    if (value < lo)
    {
        return lo;
    }
    if (value > hi)
    {
        return hi;
    }
    return value;
}

static float nav_distance_xy(float ax, float ay, float bx, float by)
{
    const float dx = ax - bx;
    const float dy = ay - by;
    return sqrtf(dx * dx + dy * dy);
}

static float nav_distance_point(const LoraNavPoint *a, const LoraNavPoint *b)
{
    return nav_distance_xy(a->x_m, a->y_m, b->x_m, b->y_m);
}

static float nav_normalize_deg(float deg)
{
    float out = fmodf(deg + 180.0f, 360.0f);
    if (out < 0.0f)
    {
        out += 360.0f;
    }
    return out - 180.0f;
}

static float nav_angle_delta_deg(float start_deg, float end_deg)
{
    return nav_normalize_deg(end_deg - start_deg);
}

static float nav_bearing_deg(float from_x, float from_y, float to_x, float to_y)
{
    const float dx = to_x - from_x;
    const float dy = to_y - from_y;
    return nav_normalize_deg(atan2f(dx, dy) * 180.0f / LORA_NAV_PI);
}

static int32_t nav_meters_to_mm(float value_m)
{
    return (int32_t)(value_m * 1000.0f + (value_m >= 0.0f ? 0.5f : -0.5f));
}

static int16_t nav_deg_to_cdeg(float value_deg)
{
    const float normalized = nav_normalize_deg(value_deg);
    return (int16_t)(normalized * 100.0f + (normalized >= 0.0f ? 0.5f : -0.5f));
}

static float nav_turn_angle(const LoraNavPoint *prev, const LoraNavPoint *cur, const LoraNavPoint *next)
{
    const float ax = cur->x_m - prev->x_m;
    const float ay = cur->y_m - prev->y_m;
    const float bx = next->x_m - cur->x_m;
    const float by = next->y_m - cur->y_m;
    const float len_a = sqrtf(ax * ax + ay * ay);
    const float len_b = sqrtf(bx * bx + by * by);
    float cos_value;

    if (len_a < LORA_NAV_EPS || len_b < LORA_NAV_EPS)
    {
        return 0.0f;
    }

    cos_value = (ax * bx + ay * by) / (len_a * len_b);
    cos_value = nav_clampf(cos_value, -1.0f, 1.0f);
    return acosf(cos_value) * 180.0f / LORA_NAV_PI;
}

static bool nav_direction_matches(float move_dx, float move_dy,
                                  const LoraNavPoint *start,
                                  const LoraNavPoint *end,
                                  float threshold_deg)
{
    const float seg_dx = end->x_m - start->x_m;
    const float seg_dy = end->y_m - start->y_m;
    const float seg_len = sqrtf(seg_dx * seg_dx + seg_dy * seg_dy);
    const float move_len = sqrtf(move_dx * move_dx + move_dy * move_dy);
    float cos_value;
    float angle;

    if (seg_len < LORA_NAV_EPS || move_len < LORA_NAV_EPS)
    {
        return false;
    }

    cos_value = (move_dx * seg_dx + move_dy * seg_dy) / (move_len * seg_len);
    cos_value = nav_clampf(cos_value, -1.0f, 1.0f);
    angle = acosf(cos_value) * 180.0f / LORA_NAV_PI;
    return angle <= threshold_deg;
}

static bool nav_direction_matches_bidir(float move_dx, float move_dy,
                                        const LoraNavPoint *a,
                                        const LoraNavPoint *b,
                                        float threshold_deg)
{
    return nav_direction_matches(move_dx, move_dy, a, b, threshold_deg) ||
           nav_direction_matches(move_dx, move_dy, b, a, threshold_deg);
}

static bool nav_project_on_segment(const LoraNavPoint *point,
                                   const LoraNavPoint *a,
                                   const LoraNavPoint *b,
                                   float *proj_x,
                                   float *proj_y,
                                   float *pos,
                                   float *distance_m)
{
    const float dx = b->x_m - a->x_m;
    const float dy = b->y_m - a->y_m;
    const float len_sq = dx * dx + dy * dy;
    float p;
    float x;
    float y;

    if (len_sq < 1.0e-12f)
    {
        return false;
    }

    p = ((point->x_m - a->x_m) * dx + (point->y_m - a->y_m) * dy) / len_sq;
    p = nav_clampf(p, 0.0f, 1.0f);
    x = a->x_m + p * dx;
    y = a->y_m + p * dy;

    if (proj_x != NULL)
    {
        *proj_x = x;
    }
    if (proj_y != NULL)
    {
        *proj_y = y;
    }
    if (pos != NULL)
    {
        *pos = p;
    }
    if (distance_m != NULL)
    {
        *distance_m = nav_distance_xy(point->x_m, point->y_m, x, y);
    }
    return true;
}

static float nav_cross(float ax, float ay, float bx, float by)
{
    return ax * by - ay * bx;
}

static bool nav_segment_intersection(const LoraNavPoint *a,
                                     const LoraNavPoint *b,
                                     const LoraNavPoint *c,
                                     const LoraNavPoint *d,
                                     float *out_x,
                                     float *out_y,
                                     float *out_pos_ab,
                                     float *out_pos_cd)
{
    const float rx = b->x_m - a->x_m;
    const float ry = b->y_m - a->y_m;
    const float sx = d->x_m - c->x_m;
    const float sy = d->y_m - c->y_m;
    const float denominator = nav_cross(rx, ry, sx, sy);
    const float qpx = c->x_m - a->x_m;
    const float qpy = c->y_m - a->y_m;
    float pos_ab;
    float pos_cd;

    if (nav_absf(denominator) < 1.0e-9f)
    {
        return false;
    }

    pos_ab = nav_cross(qpx, qpy, sx, sy) / denominator;
    pos_cd = nav_cross(qpx, qpy, rx, ry) / denominator;
    if (pos_ab < -1.0e-6f || pos_ab > 1.0f + 1.0e-6f ||
        pos_cd < -1.0e-6f || pos_cd > 1.0f + 1.0e-6f)
    {
        return false;
    }

    pos_ab = nav_clampf(pos_ab, 0.0f, 1.0f);
    pos_cd = nav_clampf(pos_cd, 0.0f, 1.0f);

    *out_x = a->x_m + pos_ab * rx;
    *out_y = a->y_m + pos_ab * ry;
    *out_pos_ab = pos_ab;
    *out_pos_cd = pos_cd;
    return true;
}

static bool nav_parse_int(const char **cursor, int32_t *out)
{
    char *end_ptr;
    long value;

    while (**cursor == ' ' || **cursor == '\t')
    {
        (*cursor)++;
    }

    value = strtol(*cursor, &end_ptr, 10);
    if (end_ptr == *cursor)
    {
        return false;
    }

    *out = (int32_t)value;
    *cursor = end_ptr;
    return true;
}

static bool nav_parse_comma(const char **cursor)
{
    while (**cursor == ' ' || **cursor == '\t')
    {
        (*cursor)++;
    }
    if (**cursor != ',')
    {
        return false;
    }
    (*cursor)++;
    return true;
}

void LoraNav_DefaultConfig(LoraNavConfig *cfg)
{
    if (cfg == NULL)
    {
        return;
    }

    cfg->turn_angle_deg = 60.0f;
    cfg->min_segment_m = 0.5f;
    cfg->merge_distance_m = 3.0f;
    cfg->snap_distance_m = 3.0f;
    cfg->direction_match_deg = 45.0f;
    cfg->target_arrive_m = 0.5f;
    cfg->forward_snap_enable = true;
    cfg->return_snap_enable = true;
}

static void nav_init_edges(LoraNavContext *ctx)
{
    uint16_t i;
    uint16_t j;

    for (i = 0u; i < LORA_NAV_MAX_NODES; i++)
    {
        for (j = 0u; j < LORA_NAV_MAX_NODES; j++)
        {
            ctx->edges[i][j] = LORA_NAV_NO_EDGE;
        }
        ctx->edges[i][i] = 0.0f;
    }
}

void LoraNav_Init(LoraNavContext *ctx, const LoraNavConfig *cfg)
{
    LoraNavConfig defaults;

    if (ctx == NULL)
    {
        return;
    }

    memset(ctx, 0, sizeof(*ctx));
    if (cfg == NULL)
    {
        LoraNav_DefaultConfig(&defaults);
        ctx->cfg = defaults;
    }
    else
    {
        ctx->cfg = *cfg;
    }

    ctx->home_node_id = UINT16_MAX;
    ctx->last_key_node_id = UINT16_MAX;
    ctx->forward_edge_a = UINT16_MAX;
    ctx->forward_edge_b = UINT16_MAX;
    nav_init_edges(ctx);
}

static LoraNavParseResult nav_parse_line(const char *line, LoraNavPoint *point)
{
    const char *cursor = line;
    int32_t x_mm;
    int32_t y_mm;
    int32_t z_mm;
    int32_t yaw_cdeg;

    if (line == NULL)
    {
        return LORA_NAV_PARSE_ERROR;
    }

    while (*cursor == ' ' || *cursor == '\t')
    {
        cursor++;
    }

    if (*cursor == 'R' || *cursor == 'r')
    {
        cursor++;
        while (*cursor == ' ' || *cursor == '\t')
        {
            cursor++;
        }
        if (*cursor == '\0' || *cursor == '\r' || *cursor == '\n' || *cursor == '[')
        {
            return LORA_NAV_PARSE_RETURN_MARKER;
        }
    }

    cursor = line;
    if (!nav_parse_int(&cursor, &x_mm) ||
        !nav_parse_comma(&cursor) ||
        !nav_parse_int(&cursor, &y_mm) ||
        !nav_parse_comma(&cursor) ||
        !nav_parse_int(&cursor, &z_mm) ||
        !nav_parse_comma(&cursor) ||
        !nav_parse_int(&cursor, &yaw_cdeg))
    {
        return LORA_NAV_PARSE_IGNORED;
    }

    point->x_m = (float)x_mm / 1000.0f;
    point->y_m = (float)y_mm / 1000.0f;
    point->z_m = (float)z_mm / 1000.0f;
    point->yaw_deg = (float)yaw_cdeg / 100.0f;
    return LORA_NAV_PARSE_POINT;
}

static void nav_add_edge(LoraNavContext *ctx, uint16_t a, uint16_t b, float weight)
{
    if (a >= ctx->node_count || b >= ctx->node_count || a == b)
    {
        return;
    }
    if (weight < LORA_NAV_EPS)
    {
        weight = 0.0f;
    }
    if (ctx->edges[a][b] >= LORA_NAV_NO_EDGE || weight < ctx->edges[a][b])
    {
        ctx->edges[a][b] = weight;
        ctx->edges[b][a] = weight;
    }
}

static void nav_remove_edge(LoraNavContext *ctx, uint16_t a, uint16_t b)
{
    if (a >= LORA_NAV_MAX_NODES || b >= LORA_NAV_MAX_NODES)
    {
        return;
    }
    ctx->edges[a][b] = LORA_NAV_NO_EDGE;
    ctx->edges[b][a] = LORA_NAV_NO_EDGE;
    if (a == b)
    {
        ctx->edges[a][a] = 0.0f;
    }
}

static bool nav_find_or_add_node(LoraNavContext *ctx, const LoraNavPoint *point, uint16_t *out_id)
{
    uint16_t i;

    for (i = 0u; i < ctx->node_count; i++)
    {
        if (nav_distance_point(&ctx->nodes[i], point) <= ctx->cfg.merge_distance_m)
        {
            *out_id = i;
            return true;
        }
    }

    if (ctx->node_count >= LORA_NAV_MAX_NODES)
    {
        return false;
    }

    ctx->nodes[ctx->node_count] = *point;
    *out_id = ctx->node_count;
    ctx->node_count++;
    return true;
}

static uint16_t nav_intersection_node_id(LoraNavContext *ctx,
                                         uint16_t a,
                                         uint16_t b,
                                         float pos_new,
                                         uint16_t c,
                                         uint16_t d,
                                         float pos_old,
                                         float x_m,
                                         float y_m,
                                         bool *ok)
{
    LoraNavPoint point;
    uint16_t id;

    *ok = true;

    if (pos_new <= 1.0e-6f)
    {
        return a;
    }
    if (pos_new >= 1.0f - 1.0e-6f)
    {
        return b;
    }
    if (pos_old <= 1.0e-6f)
    {
        return c;
    }
    if (pos_old >= 1.0f - 1.0e-6f)
    {
        return d;
    }

    point.x_m = x_m;
    point.y_m = y_m;
    point.z_m = 0.0f;
    point.yaw_deg = 0.0f;
    if (!nav_find_or_add_node(ctx, &point, &id))
    {
        *ok = false;
        return UINT16_MAX;
    }

    ctx->nodes[id] = point;
    return id;
}

static void nav_sort_segment_points(LoraNavSegmentPoint *items, uint16_t count)
{
    uint16_t i;
    for (i = 1u; i < count; i++)
    {
        LoraNavSegmentPoint key = items[i];
        int16_t j = (int16_t)i - 1;
        while (j >= 0 && items[j].pos > key.pos)
        {
            items[j + 1] = items[j];
            j--;
        }
        items[j + 1] = key;
    }
}

static void nav_add_ordered_edges(LoraNavContext *ctx, LoraNavSegmentPoint *items, uint16_t count)
{
    uint16_t i;
    uint16_t prev_id;

    if (count < 2u)
    {
        return;
    }

    nav_sort_segment_points(items, count);
    prev_id = items[0].id;
    for (i = 1u; i < count; i++)
    {
        const uint16_t id = items[i].id;
        if (id != prev_id)
        {
            const float weight = nav_distance_point(&ctx->nodes[prev_id], &ctx->nodes[id]);
            if (weight > LORA_NAV_EPS)
            {
                nav_add_edge(ctx, prev_id, id, weight);
            }
        }
        prev_id = id;
    }
}

static void nav_refresh_node_weights(LoraNavContext *ctx, uint16_t node_id)
{
    uint16_t j;
    if (node_id >= ctx->node_count)
    {
        return;
    }

    for (j = 0u; j < ctx->node_count; j++)
    {
        if (j == node_id)
        {
            ctx->edges[node_id][j] = 0.0f;
        }
        else if (ctx->edges[node_id][j] < LORA_NAV_NO_EDGE)
        {
            const float weight = nav_distance_point(&ctx->nodes[node_id], &ctx->nodes[j]);
            ctx->edges[node_id][j] = weight;
            ctx->edges[j][node_id] = weight;
        }
    }
}

static void nav_cluster_intersection_area(LoraNavContext *ctx, uint16_t center_id)
{
    uint16_t i;
    uint16_t j;
    LoraNavPoint center;

    if (center_id >= ctx->node_count || ctx->cfg.merge_distance_m <= 0.0f)
    {
        return;
    }

    center = ctx->nodes[center_id];
    for (i = 0u; i < ctx->node_count; i++)
    {
        if (nav_distance_point(&ctx->nodes[i], &center) <= ctx->cfg.merge_distance_m)
        {
            ctx->nodes[i] = center;
            if (i != center_id)
            {
                nav_add_edge(ctx, center_id, i, 0.0f);
            }
        }
    }

    for (i = 0u; i < ctx->node_count; i++)
    {
        if (nav_distance_point(&ctx->nodes[i], &center) <= LORA_NAV_EPS)
        {
            for (j = 0u; j < ctx->node_count; j++)
            {
                if (ctx->edges[i][j] < LORA_NAV_NO_EDGE)
                {
                    ctx->edges[i][j] = nav_distance_point(&ctx->nodes[i], &ctx->nodes[j]);
                    ctx->edges[j][i] = ctx->edges[i][j];
                }
            }
        }
    }
}

static LoraNavStatus nav_add_incremental_segment(LoraNavContext *ctx, uint16_t a_id, uint16_t b_id)
{
    LoraNavSegmentPoint old_points[3];
    uint16_t new_count = 0u;
    uint16_t c;
    uint16_t intersection_ids[LORA_NAV_MAX_NODES];
    uint16_t intersection_count = 0u;

    if (a_id == b_id || a_id >= ctx->node_count || b_id >= ctx->node_count)
    {
        return LORA_NAV_OK;
    }
    if (nav_distance_point(&ctx->nodes[a_id], &ctx->nodes[b_id]) < LORA_NAV_EPS)
    {
        return LORA_NAV_OK;
    }

    ctx->scratch_points[new_count].pos = 0.0f;
    ctx->scratch_points[new_count].id = a_id;
    new_count++;
    ctx->scratch_points[new_count].pos = 1.0f;
    ctx->scratch_points[new_count].id = b_id;
    new_count++;

    for (c = 0u; c < ctx->node_count; c++)
    {
        uint16_t d;
        for (d = (uint16_t)(c + 1u); d < ctx->node_count; d++)
        {
            float x_m;
            float y_m;
            float pos_new;
            float pos_old;
            uint16_t node_id;
            bool ok;

            if (ctx->edges[c][d] >= LORA_NAV_NO_EDGE)
            {
                continue;
            }
            if (a_id == c || a_id == d || b_id == c || b_id == d)
            {
                continue;
            }
            if (!nav_segment_intersection(&ctx->nodes[a_id], &ctx->nodes[b_id],
                                          &ctx->nodes[c], &ctx->nodes[d],
                                          &x_m, &y_m, &pos_new, &pos_old))
            {
                continue;
            }

            node_id = nav_intersection_node_id(ctx, a_id, b_id, pos_new,
                                               c, d, pos_old, x_m, y_m, &ok);
            if (!ok)
            {
                return LORA_NAV_FULL;
            }

            if (new_count >= LORA_NAV_MAX_NODES)
            {
                return LORA_NAV_FULL;
            }
            ctx->scratch_points[new_count].pos = pos_new;
            ctx->scratch_points[new_count].id = node_id;
            new_count++;

            old_points[0].pos = 0.0f;
            old_points[0].id = c;
            old_points[1].pos = pos_old;
            old_points[1].id = node_id;
            old_points[2].pos = 1.0f;
            old_points[2].id = d;
            nav_remove_edge(ctx, c, d);
            nav_add_ordered_edges(ctx, old_points, 3u);

            if (intersection_count < LORA_NAV_MAX_NODES)
            {
                intersection_ids[intersection_count++] = node_id;
            }
        }
    }

    nav_add_ordered_edges(ctx, ctx->scratch_points, new_count);

    for (c = 0u; c < intersection_count; c++)
    {
        nav_cluster_intersection_area(ctx, intersection_ids[c]);
        nav_refresh_node_weights(ctx, intersection_ids[c]);
    }

    return LORA_NAV_OK;
}

static LoraNavStatus nav_commit_key_point(LoraNavContext *ctx, const LoraNavPoint *point)
{
    uint16_t node_id;
    LoraNavStatus status;

    if (!nav_find_or_add_node(ctx, point, &node_id))
    {
        return LORA_NAV_FULL;
    }

    if (!ctx->has_last_key)
    {
        ctx->home_node_id = node_id;
        ctx->last_key_node_id = node_id;
        ctx->has_home = true;
        ctx->has_last_key = true;
        ctx->key_count = 0u;
        ctx->key_node_ids[ctx->key_count++] = node_id;
        return LORA_NAV_OK;
    }

    if (node_id != ctx->last_key_node_id)
    {
        status = nav_add_incremental_segment(ctx, ctx->last_key_node_id, node_id);
        if (status != LORA_NAV_OK)
        {
            return status;
        }

        ctx->last_key_node_id = node_id;
        if (ctx->key_count < LORA_NAV_MAX_NODES)
        {
            ctx->key_node_ids[ctx->key_count++] = node_id;
        }
    }

    return LORA_NAV_OK;
}

static void nav_append_map_point(LoraNavContext *ctx, const LoraNavPoint *point)
{
    if (ctx->point_count < LORA_NAV_MAX_POINTS)
    {
        ctx->points[ctx->point_count++] = *point;
    }
    else
    {
        memmove(&ctx->points[0], &ctx->points[1], sizeof(ctx->points[0]) * (LORA_NAV_MAX_POINTS - 1u));
        ctx->points[LORA_NAV_MAX_POINTS - 1u] = *point;
    }
    ctx->total_point_count++;
}

static LoraNavStatus nav_update_incremental_map(LoraNavContext *ctx)
{
    LoraNavPoint prev_key;
    LoraNavPoint current;
    LoraNavPoint next;
    float prev_len;
    float next_len;
    float angle;

    if (ctx->point_count == 0u)
    {
        return LORA_NAV_OK;
    }

    if (!ctx->has_home)
    {
        return nav_commit_key_point(ctx, &ctx->points[ctx->point_count - 1u]);
    }

    if (ctx->point_count < 3u)
    {
        return LORA_NAV_OK;
    }

    prev_key = ctx->nodes[ctx->last_key_node_id];
    current = ctx->points[ctx->point_count - 2u];
    next = ctx->points[ctx->point_count - 1u];
    prev_len = nav_distance_point(&prev_key, &current);
    next_len = nav_distance_point(&current, &next);
    if (prev_len < nav_maxf(ctx->cfg.min_segment_m, LORA_NAV_EPS) || next_len < LORA_NAV_EPS)
    {
        return LORA_NAV_OK;
    }

    angle = nav_turn_angle(&prev_key, &current, &next);
    if (angle >= ctx->cfg.turn_angle_deg)
    {
        return nav_commit_key_point(ctx, &current);
    }

    return LORA_NAV_OK;
}

static bool nav_find_forward_snap_edge(LoraNavContext *ctx,
                                       const LoraNavPoint *predicted,
                                       float move_dx,
                                       float move_dy,
                                       uint16_t *out_a,
                                       uint16_t *out_b,
                                       float *out_x,
                                       float *out_y)
{
    float best_score = LORA_NAV_NO_EDGE;
    bool found = false;
    uint16_t a;

    for (a = 0u; a < ctx->node_count; a++)
    {
        uint16_t b;
        for (b = (uint16_t)(a + 1u); b < ctx->node_count; b++)
        {
            float proj_x;
            float proj_y;
            float pos;
            float distance_m;
            float score;

            if (ctx->edges[a][b] >= LORA_NAV_NO_EDGE)
            {
                continue;
            }
            if (!nav_direction_matches_bidir(move_dx, move_dy, &ctx->nodes[a], &ctx->nodes[b], ctx->cfg.direction_match_deg))
            {
                continue;
            }
            if (!nav_project_on_segment(predicted, &ctx->nodes[a], &ctx->nodes[b], &proj_x, &proj_y, &pos, &distance_m))
            {
                continue;
            }
            if (distance_m > ctx->cfg.snap_distance_m)
            {
                continue;
            }

            score = distance_m;
            if (!ctx->forward_has_edge || ctx->forward_edge_a != a || ctx->forward_edge_b != b)
            {
                score += 0.2f;
            }

            if (score < best_score)
            {
                best_score = score;
                *out_a = a;
                *out_b = b;
                *out_x = proj_x;
                *out_y = proj_y;
                found = true;
            }
        }
    }

    return found;
}

static LoraNavPoint nav_snap_forward_point(LoraNavContext *ctx, const LoraNavPoint *point)
{
    LoraNavPoint predicted = *point;
    float move_dx;
    float move_dy;
    uint16_t a;
    uint16_t b;
    float proj_x;
    float proj_y;

    if (!ctx->cfg.forward_snap_enable || ctx->node_count < 2u)
    {
        ctx->forward_has_edge = false;
        return *point;
    }

    if (ctx->forward_has_snap && ctx->forward_has_prev && ctx->forward_has_curr)
    {
        move_dx = ctx->forward_raw_curr.x_m - ctx->forward_raw_prev.x_m;
        move_dy = ctx->forward_raw_curr.y_m - ctx->forward_raw_prev.y_m;
        predicted.x_m = ctx->forward_snap_last.x_m + move_dx;
        predicted.y_m = ctx->forward_snap_last.y_m + move_dy;
        predicted.z_m = point->z_m;
        predicted.yaw_deg = point->yaw_deg;
    }
    else
    {
        ctx->forward_snap_last = *point;
        ctx->forward_has_snap = true;
        return *point;
    }

    if (sqrtf(move_dx * move_dx + move_dy * move_dy) < LORA_NAV_EPS)
    {
        ctx->forward_snap_last = predicted;
        return predicted;
    }

    if (nav_find_forward_snap_edge(ctx, &predicted, move_dx, move_dy, &a, &b, &proj_x, &proj_y))
    {
        predicted.x_m = proj_x;
        predicted.y_m = proj_y;
        ctx->forward_edge_a = a;
        ctx->forward_edge_b = b;
        ctx->forward_has_edge = true;
    }

    ctx->forward_snap_last = predicted;
    ctx->forward_has_snap = true;
    return predicted;
}

static void nav_push_forward_raw(LoraNavContext *ctx, const LoraNavPoint *point)
{
    if (ctx->forward_has_curr)
    {
        ctx->forward_raw_prev = ctx->forward_raw_curr;
        ctx->forward_has_prev = true;
    }
    ctx->forward_raw_curr = *point;
    ctx->forward_has_curr = true;
}

static void nav_push_return_raw(LoraNavContext *ctx, const LoraNavPoint *point)
{
    if (ctx->return_has_curr)
    {
        ctx->return_raw_prev = ctx->return_raw_curr;
        ctx->return_has_prev = true;
    }
    ctx->return_raw_curr = *point;
    ctx->return_has_curr = true;
}

static bool nav_dijkstra(LoraNavContext *ctx, uint16_t start_id, uint16_t home_id)
{
    uint16_t i;
    uint16_t route_tmp[LORA_NAV_MAX_NODES];
    uint16_t route_len = 0u;
    uint16_t cur;

    if (start_id >= ctx->node_count || home_id >= ctx->node_count)
    {
        return false;
    }

    for (i = 0u; i < ctx->node_count; i++)
    {
        ctx->dijkstra_dist[i] = LORA_NAV_NO_EDGE;
        ctx->dijkstra_prev[i] = -1;
        ctx->dijkstra_used[i] = false;
    }
    ctx->dijkstra_dist[start_id] = 0.0f;

    for (;;)
    {
        uint16_t best = UINT16_MAX;
        float best_dist = LORA_NAV_NO_EDGE;
        uint16_t j;

        for (i = 0u; i < ctx->node_count; i++)
        {
            if (!ctx->dijkstra_used[i] && ctx->dijkstra_dist[i] < best_dist)
            {
                best = i;
                best_dist = ctx->dijkstra_dist[i];
            }
        }

        if (best == UINT16_MAX)
        {
            break;
        }
        if (best == home_id)
        {
            break;
        }

        ctx->dijkstra_used[best] = true;
        for (j = 0u; j < ctx->node_count; j++)
        {
            const float weight = ctx->edges[best][j];
            const float new_dist = ctx->dijkstra_dist[best] + weight;
            if (weight >= LORA_NAV_NO_EDGE || j == best)
            {
                continue;
            }
            if (new_dist < ctx->dijkstra_dist[j])
            {
                ctx->dijkstra_dist[j] = new_dist;
                ctx->dijkstra_prev[j] = (int16_t)best;
            }
        }
    }

    if (ctx->dijkstra_dist[home_id] >= LORA_NAV_NO_EDGE)
    {
        return false;
    }

    cur = home_id;
    while (route_len < LORA_NAV_MAX_NODES)
    {
        route_tmp[route_len++] = cur;
        if (cur == start_id)
        {
            break;
        }
        if (ctx->dijkstra_prev[cur] < 0)
        {
            return false;
        }
        cur = (uint16_t)ctx->dijkstra_prev[cur];
    }

    if (route_len == 0u || route_tmp[route_len - 1u] != start_id)
    {
        return false;
    }

    ctx->route_count = route_len;
    for (i = 0u; i < route_len; i++)
    {
        const uint16_t node_id = route_tmp[route_len - 1u - i];
        ctx->route_ids[i] = node_id;
        ctx->route_points[i] = ctx->nodes[node_id];
    }

    ctx->snap_segment_index = 0u;
    ctx->route_valid = route_len > 0u;
    return ctx->route_valid;
}

static LoraNavStatus nav_start_return(LoraNavContext *ctx)
{
    uint16_t start_id;
    LoraNavStatus status;

    if (ctx->point_count == 0u || !ctx->has_home)
    {
        ctx->route_valid = false;
        return LORA_NAV_NO_ROUTE;
    }

    status = nav_commit_key_point(ctx, &ctx->points[ctx->point_count - 1u]);
    if (status != LORA_NAV_OK)
    {
        return status;
    }
    start_id = ctx->last_key_node_id;

    if (!nav_dijkstra(ctx, start_id, ctx->home_node_id))
    {
        ctx->route_valid = false;
        return LORA_NAV_NO_ROUTE;
    }

    ctx->return_mode = true;
    ctx->return_has_prev = false;
    ctx->return_has_curr = false;
    ctx->return_has_snap = false;
    return LORA_NAV_OK;
}

LoraNavStatus LoraNav_EnterReturnMode(LoraNavContext *ctx)
{
    if (ctx == NULL)
    {
        return LORA_NAV_BAD_ARG;
    }
    return nav_start_return(ctx);
}

void LoraNav_ExitReturnMode(LoraNavContext *ctx)
{
    if (ctx == NULL)
    {
        return;
    }

    ctx->return_mode = false;
    ctx->route_valid = false;
    ctx->route_count = 0u;
    ctx->snap_segment_index = 0u;
    ctx->return_has_prev = false;
    ctx->return_has_curr = false;
    ctx->return_has_snap = false;
}

static bool nav_find_route_snap_segment(LoraNavContext *ctx,
                                        const LoraNavPoint *predicted,
                                        float move_dx,
                                        float move_dy,
                                        uint16_t *out_seg,
                                        float *out_x,
                                        float *out_y)
{
    uint16_t last_seg;
    uint16_t pass;
    float best_score = LORA_NAV_NO_EDGE;
    bool found = false;

    if (!ctx->route_valid || ctx->route_count < 2u)
    {
        return false;
    }

    last_seg = (uint16_t)(ctx->route_count - 2u);
    for (pass = 0u; pass < 2u; pass++)
    {
        uint16_t start = 0u;
        uint16_t end = last_seg;
        uint16_t seg;

        if (pass == 0u)
        {
            uint16_t local_end;
            start = ctx->snap_segment_index > 0u ? (uint16_t)(ctx->snap_segment_index - 1u) : 0u;
            local_end = (uint16_t)(ctx->snap_segment_index + 2u);
            end = local_end < last_seg ? local_end : last_seg;
        }

        for (seg = start; seg <= end; seg++)
        {
            float proj_x;
            float proj_y;
            float pos;
            float distance_m;
            float score;

            if (!nav_direction_matches(move_dx, move_dy,
                                       &ctx->route_points[seg],
                                       &ctx->route_points[seg + 1u],
                                       ctx->cfg.direction_match_deg))
            {
                continue;
            }
            if (!nav_project_on_segment(predicted, &ctx->route_points[seg], &ctx->route_points[seg + 1u],
                                        &proj_x, &proj_y, &pos, &distance_m))
            {
                continue;
            }
            if (distance_m > ctx->cfg.snap_distance_m)
            {
                continue;
            }

            score = distance_m + nav_absf((float)seg - (float)ctx->snap_segment_index) * 0.2f;
            if (score < best_score)
            {
                best_score = score;
                *out_seg = seg;
                *out_x = proj_x;
                *out_y = proj_y;
                found = true;
            }

        }

        if (pass == 0u && found)
        {
            break;
        }
    }

    return found;
}

static LoraNavPoint nav_snap_return_point(LoraNavContext *ctx, const LoraNavPoint *point)
{
    LoraNavPoint predicted = *point;
    float move_dx;
    float move_dy;
    uint16_t seg;
    float proj_x;
    float proj_y;

    if (!ctx->cfg.return_snap_enable || !ctx->route_valid || ctx->route_count < 2u)
    {
        return *point;
    }

    if (ctx->return_has_snap && ctx->return_has_prev && ctx->return_has_curr)
    {
        move_dx = ctx->return_raw_curr.x_m - ctx->return_raw_prev.x_m;
        move_dy = ctx->return_raw_curr.y_m - ctx->return_raw_prev.y_m;
        predicted.x_m = ctx->return_snap_last.x_m + move_dx;
        predicted.y_m = ctx->return_snap_last.y_m + move_dy;
        predicted.z_m = point->z_m;
        predicted.yaw_deg = point->yaw_deg;
    }
    else
    {
        ctx->return_snap_last = *point;
        ctx->return_has_snap = true;
        return *point;
    }

    if (sqrtf(move_dx * move_dx + move_dy * move_dy) < LORA_NAV_EPS)
    {
        ctx->return_snap_last = predicted;
        return predicted;
    }

    if (nav_find_route_snap_segment(ctx, &predicted, move_dx, move_dy, &seg, &proj_x, &proj_y))
    {
        predicted.x_m = proj_x;
        predicted.y_m = proj_y;
        ctx->snap_segment_index = seg;
    }

    ctx->return_snap_last = predicted;
    ctx->return_has_snap = true;
    return predicted;
}

static void nav_advance_target(LoraNavContext *ctx, const LoraNavPoint *point)
{
    while (ctx->route_valid && ctx->route_count >= 2u && ctx->snap_segment_index + 1u < ctx->route_count)
    {
        const LoraNavPoint *target = &ctx->route_points[ctx->snap_segment_index + 1u];
        if (nav_distance_point(point, target) > ctx->cfg.target_arrive_m)
        {
            break;
        }
        if (ctx->snap_segment_index + 2u >= ctx->route_count)
        {
            break;
        }
        ctx->snap_segment_index++;
    }
}

static void nav_fill_return_output(LoraNavContext *ctx, const LoraNavPoint *point, LoraNavOutput *out)
{
    const LoraNavPoint *target;

    memset(out, 0, sizeof(*out));
    out->valid = true;
    out->return_mode = ctx->return_mode;
    out->route_valid = ctx->route_valid;
    out->corrected_point = *point;

    if (!ctx->route_valid || ctx->route_count == 0u)
    {
        return;
    }

    nav_advance_target(ctx, point);
    if (ctx->snap_segment_index + 1u >= ctx->route_count)
    {
        out->arrived_home = true;
        out->next_key_point = ctx->route_points[ctx->route_count - 1u];
        return;
    }

    target = &ctx->route_points[ctx->snap_segment_index + 1u];
    out->next_key_point = *target;
    out->next_route_index = (uint16_t)(ctx->snap_segment_index + 1u);
    out->distance_to_next_m = nav_distance_point(point, target);
    out->bearing_to_next_deg = nav_bearing_deg(point->x_m, point->y_m, target->x_m, target->y_m);
    out->relative_bearing_deg = nav_angle_delta_deg(point->yaw_deg, out->bearing_to_next_deg);
    out->distance_to_next_mm = nav_meters_to_mm(out->distance_to_next_m);
    out->bearing_to_next_cdeg = nav_deg_to_cdeg(out->bearing_to_next_deg);
    out->relative_bearing_cdeg = nav_deg_to_cdeg(out->relative_bearing_deg);

    if (out->next_route_index + 1u < ctx->route_count)
    {
        const LoraNavPoint *segment_start = &ctx->route_points[ctx->snap_segment_index];
        const LoraNavPoint *following = &ctx->route_points[out->next_route_index + 1u];
        const float segment_bearing = nav_bearing_deg(segment_start->x_m,
                                                      segment_start->y_m,
                                                      target->x_m,
                                                      target->y_m);
        const float following_bearing = nav_bearing_deg(target->x_m,
                                                        target->y_m,
                                                        following->x_m,
                                                        following->y_m);
        const float turn_deg = nav_angle_delta_deg(segment_bearing, following_bearing);

        if (turn_deg > LORA_NAV_EPS)
        {
            out->turn_after_next = 1;
        }
        else if (turn_deg < -LORA_NAV_EPS)
        {
            out->turn_after_next = -1;
        }
    }
    out->arrived_home = (out->next_route_index == ctx->route_count - 1u &&
                         out->distance_to_next_m <= ctx->cfg.target_arrive_m);
}

LoraNavStatus LoraNav_ProcessPoint(LoraNavContext *ctx, const LoraNavPoint *raw_point, LoraNavOutput *out)
{
    LoraNavPoint point;
    LoraNavPoint map_point;
    LoraNavStatus status;

    if (ctx == NULL || raw_point == NULL || out == NULL)
    {
        return LORA_NAV_BAD_ARG;
    }

    point = *raw_point;

    if (ctx->return_mode)
    {
        LoraNavPoint snapped;
        nav_push_return_raw(ctx, &point);
        snapped = nav_snap_return_point(ctx, &point);
        nav_fill_return_output(ctx, &snapped, out);
        return ctx->route_valid ? LORA_NAV_OK : LORA_NAV_NO_ROUTE;
    }

    nav_push_forward_raw(ctx, &point);
    map_point = nav_snap_forward_point(ctx, &point);
    nav_append_map_point(ctx, &map_point);
    status = nav_update_incremental_map(ctx);

    memset(out, 0, sizeof(*out));
    out->valid = true;
    out->return_mode = false;
    out->corrected_point = map_point;
    return status;
}

LoraNavStatus LoraNav_ProcessLine(LoraNavContext *ctx, const char *line, LoraNavOutput *out)
{
    LoraNavPoint point;
    LoraNavParseResult parse_result;

    if (ctx == NULL || out == NULL)
    {
        return LORA_NAV_BAD_ARG;
    }

    memset(out, 0, sizeof(*out));
    parse_result = nav_parse_line(line, &point);
    if (parse_result == LORA_NAV_PARSE_RETURN_MARKER)
    {
        LoraNavStatus status = nav_start_return(ctx);
        out->valid = true;
        out->return_mode = ctx->return_mode;
        out->route_valid = ctx->route_valid;
        return status;
    }
    if (parse_result == LORA_NAV_PARSE_POINT)
    {
        return LoraNav_ProcessPoint(ctx, &point, out);
    }
    if (parse_result == LORA_NAV_PARSE_ERROR)
    {
        return LORA_NAV_BAD_ARG;
    }

    return LORA_NAV_OK;
}

uint16_t LoraNav_GetNodeCount(const LoraNavContext *ctx)
{
    return ctx != NULL ? ctx->node_count : 0u;
}

uint16_t LoraNav_GetRouteCount(const LoraNavContext *ctx)
{
    return ctx != NULL ? ctx->route_count : 0u;
}

bool LoraNav_IsReturnMode(const LoraNavContext *ctx)
{
    return ctx != NULL && ctx->return_mode;
}

int32_t LoraNav_MetersToMm(float value_m)
{
    return nav_meters_to_mm(value_m);
}

int16_t LoraNav_DegToCdeg(float value_deg)
{
    return nav_deg_to_cdeg(value_deg);
}
