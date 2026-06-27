#ifndef DIJKSTRA_H
#define DIJKSTRA_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef LORA_NAV_MAX_POINTS
#define LORA_NAV_MAX_POINTS          50u
#endif

#ifndef LORA_NAV_MAX_NODES
#define LORA_NAV_MAX_NODES           96u
#endif

#ifndef LORA_NAV_NO_EDGE
#define LORA_NAV_NO_EDGE             1.0e20f
#endif

#ifndef LORA_NAV_EPS
#define LORA_NAV_EPS                 1.0e-6f
#endif

#ifndef LORA_NAV_PI
#define LORA_NAV_PI                  3.14159265358979323846f
#endif

typedef enum
{
    LORA_NAV_PARSE_IGNORED = 0,
    LORA_NAV_PARSE_POINT,
    LORA_NAV_PARSE_RETURN_MARKER,
    LORA_NAV_PARSE_ERROR
} LoraNavParseResult;

typedef enum
{
    LORA_NAV_OK = 0,
    LORA_NAV_FULL,
    LORA_NAV_NO_ROUTE,
    LORA_NAV_BAD_ARG
} LoraNavStatus;

typedef struct
{
    float turn_angle_deg;
    float min_segment_m;
    float merge_distance_m;
    float snap_distance_m;
    float direction_match_deg;
    float target_arrive_m;

    bool forward_snap_enable;
    bool return_snap_enable;
} LoraNavConfig;

typedef struct
{
    float x_m;
    float y_m;
    float z_m;
    float yaw_deg;
} LoraNavPoint;

typedef struct
{
    bool valid;
    bool return_mode;
    bool route_valid;
    bool arrived_home;

    LoraNavPoint corrected_point;
    LoraNavPoint next_key_point;
    float distance_to_next_m;
    float bearing_to_next_deg;
    float relative_bearing_deg;
    int32_t distance_to_next_mm;
    int16_t bearing_to_next_cdeg;
    int16_t relative_bearing_cdeg;
    uint16_t next_route_index;
    int8_t turn_after_next; /* -1: left, 0: no following turn, 1: right */
} LoraNavOutput;

typedef struct
{
    float pos;
    uint16_t id;
} LoraNavSegmentPoint;

typedef struct
{
    LoraNavConfig cfg;

    LoraNavPoint points[LORA_NAV_MAX_POINTS];
    uint16_t point_count;
    uint32_t total_point_count;

    LoraNavPoint nodes[LORA_NAV_MAX_NODES];
    float edges[LORA_NAV_MAX_NODES][LORA_NAV_MAX_NODES];
    uint16_t node_count;
    uint16_t key_node_ids[LORA_NAV_MAX_NODES];
    uint16_t key_count;
    uint16_t home_node_id;
    uint16_t last_key_node_id;
    bool has_home;
    bool has_last_key;

    bool return_mode;
    bool route_valid;
    uint16_t route_ids[LORA_NAV_MAX_NODES];
    LoraNavPoint route_points[LORA_NAV_MAX_NODES];
    uint16_t route_count;
    uint16_t snap_segment_index;

    LoraNavPoint return_raw_prev;
    LoraNavPoint return_raw_curr;
    bool return_has_prev;
    bool return_has_curr;
    LoraNavPoint return_snap_last;
    bool return_has_snap;

    LoraNavPoint forward_raw_prev;
    LoraNavPoint forward_raw_curr;
    bool forward_has_prev;
    bool forward_has_curr;
    LoraNavPoint forward_snap_last;
    bool forward_has_snap;
    uint16_t forward_edge_a;
    uint16_t forward_edge_b;
    bool forward_has_edge;

    float dijkstra_dist[LORA_NAV_MAX_NODES];
    int16_t dijkstra_prev[LORA_NAV_MAX_NODES];
    bool dijkstra_used[LORA_NAV_MAX_NODES];
    LoraNavSegmentPoint scratch_points[LORA_NAV_MAX_NODES];
} LoraNavContext;

void LoraNav_DefaultConfig(LoraNavConfig *cfg);
void LoraNav_Init(LoraNavContext *ctx, const LoraNavConfig *cfg);
LoraNavStatus LoraNav_EnterReturnMode(LoraNavContext *ctx);
void LoraNav_ExitReturnMode(LoraNavContext *ctx);
LoraNavStatus LoraNav_ProcessPoint(LoraNavContext *ctx, const LoraNavPoint *raw_point, LoraNavOutput *out);
LoraNavStatus LoraNav_ProcessLine(LoraNavContext *ctx, const char *line, LoraNavOutput *out);
uint16_t LoraNav_GetNodeCount(const LoraNavContext *ctx);
uint16_t LoraNav_GetRouteCount(const LoraNavContext *ctx);
bool LoraNav_IsReturnMode(const LoraNavContext *ctx);
int32_t LoraNav_MetersToMm(float value_m);
int16_t LoraNav_DegToCdeg(float value_deg);

#ifdef __cplusplus
}
#endif

#endif
