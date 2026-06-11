#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <stdint.h>

/* Include the embedded implementation directly so the DLL tests the same code. */
#include "dijkstra.c"

#define NAV_EXPORT __declspec(dllexport)

typedef struct
{
    int32_t status;
    int32_t valid;
    int32_t return_mode;
    int32_t route_valid;
    int32_t arrived_home;
    int32_t corrected_x_mm;
    int32_t corrected_y_mm;
    int32_t corrected_z_mm;
    int32_t corrected_yaw_cdeg;
    int32_t next_x_mm;
    int32_t next_y_mm;
    int32_t distance_to_next_mm;
    int32_t bearing_to_next_cdeg;
    int32_t relative_bearing_cdeg;
    int32_t next_route_index;
} DijkstraTestOutput;

typedef struct
{
    uint32_t process_count;
    uint32_t last_process_us;
    uint32_t max_process_us;
    uint32_t checked_edge_count;
    uint32_t intersection_count;
    uint32_t merged_node_count;
    uint32_t node_count;
    uint32_t key_count;
    uint32_t edge_count;
    uint32_t route_count;
    uint32_t point_count;
    uint32_t total_point_count;
    int32_t last_status;
    int32_t home_node_id;
    int32_t last_key_node_id;
    int32_t return_mode;
    int32_t route_valid;
} DijkstraTestDebug;

/* The embedded module uses one persistent navigation context. */
static LoraNavContext g_ctx;
static DijkstraTestDebug g_debug;
static LARGE_INTEGER g_counter_frequency;

static uint32_t test_elapsed_us(LARGE_INTEGER start, LARGE_INTEGER end)
{
    uint64_t ticks;
    if (g_counter_frequency.QuadPart <= 0)
    {
        return 0u;
    }
    ticks = (uint64_t)(end.QuadPart - start.QuadPart);
    return (uint32_t)((ticks * 1000000ull) / (uint64_t)g_counter_frequency.QuadPart);
}

static uint32_t test_edge_count(void)
{
    uint32_t count = 0u;
    uint16_t a;
    for (a = 0u; a < g_ctx.node_count; a++)
    {
        uint16_t b;
        for (b = (uint16_t)(a + 1u); b < g_ctx.node_count; b++)
        {
            if (g_ctx.edges[a][b] < LORA_NAV_NO_EDGE)
            {
                count++;
            }
        }
    }
    return count;
}

static void test_fill_output(const LoraNavOutput *source, LoraNavStatus status, DijkstraTestOutput *out)
{
    if (out == NULL)
    {
        return;
    }

    ZeroMemory(out, sizeof(*out));
    out->status = (int32_t)status;
    if (source == NULL)
    {
        return;
    }

    out->valid = source->valid ? 1 : 0;
    out->return_mode = source->return_mode ? 1 : 0;
    out->route_valid = source->route_valid ? 1 : 0;
    out->arrived_home = source->arrived_home ? 1 : 0;
    out->corrected_x_mm = nav_meters_to_mm(source->corrected_point.x_m);
    out->corrected_y_mm = nav_meters_to_mm(source->corrected_point.y_m);
    out->corrected_z_mm = nav_meters_to_mm(source->corrected_point.z_m);
    out->corrected_yaw_cdeg = nav_deg_to_cdeg(source->corrected_point.yaw_deg);
    out->next_x_mm = nav_meters_to_mm(source->next_key_point.x_m);
    out->next_y_mm = nav_meters_to_mm(source->next_key_point.y_m);
    out->distance_to_next_mm = source->distance_to_next_mm;
    out->bearing_to_next_cdeg = source->bearing_to_next_cdeg;
    out->relative_bearing_cdeg = source->relative_bearing_cdeg;
    out->next_route_index = source->next_route_index;
}

static void test_update_debug(LoraNavStatus status, uint32_t elapsed_us)
{
    g_debug.process_count++;
    g_debug.last_process_us = elapsed_us;
    if (elapsed_us > g_debug.max_process_us)
    {
        g_debug.max_process_us = elapsed_us;
    }
    g_debug.checked_edge_count = g_ctx.debug_checked_edge_count;
    g_debug.intersection_count = g_ctx.debug_intersection_count;
    g_debug.merged_node_count = g_ctx.debug_merged_node_count;
    g_debug.node_count = g_ctx.node_count;
    g_debug.key_count = g_ctx.key_count;
    g_debug.edge_count = test_edge_count();
    g_debug.route_count = g_ctx.route_count;
    g_debug.point_count = g_ctx.point_count;
    g_debug.total_point_count = g_ctx.total_point_count;
    g_debug.last_status = (int32_t)status;
    g_debug.home_node_id = g_ctx.has_home ? (int32_t)g_ctx.home_node_id : -1;
    g_debug.last_key_node_id = g_ctx.has_last_key ? (int32_t)g_ctx.last_key_node_id : -1;
    g_debug.return_mode = g_ctx.return_mode ? 1 : 0;
    g_debug.route_valid = g_ctx.route_valid ? 1 : 0;
}

NAV_EXPORT int32_t DijkstraTest_Init(float turn_angle_deg,
                                     float min_segment_m,
                                     float merge_distance_m,
                                     float snap_distance_m,
                                     float direction_match_deg,
                                     int32_t forward_snap_enable,
                                     int32_t return_snap_enable)
{
    LoraNavConfig cfg;

    QueryPerformanceFrequency(&g_counter_frequency);
    LoraNav_DefaultConfig(&cfg);
    cfg.turn_angle_deg = turn_angle_deg;
    cfg.min_segment_m = min_segment_m;
    cfg.merge_distance_m = merge_distance_m;
    cfg.snap_distance_m = snap_distance_m;
    cfg.direction_match_deg = direction_match_deg;
    cfg.forward_snap_enable = forward_snap_enable != 0;
    cfg.return_snap_enable = return_snap_enable != 0;
    LoraNav_Init(&g_ctx, &cfg);
    ZeroMemory(&g_debug, sizeof(g_debug));
    g_debug.last_status = LORA_NAV_OK;
    g_debug.home_node_id = -1;
    g_debug.last_key_node_id = -1;
    return LORA_NAV_OK;
}

NAV_EXPORT int32_t DijkstraTest_InputPoint(int32_t x_mm,
                                           int32_t y_mm,
                                           int32_t z_mm,
                                           int32_t yaw_cdeg,
                                           DijkstraTestOutput *out)
{
    LoraNavPoint point;
    LoraNavOutput nav_out;
    LoraNavStatus status;
    LARGE_INTEGER start;
    LARGE_INTEGER end;

    point.x_m = (float)x_mm / 1000.0f;
    point.y_m = (float)y_mm / 1000.0f;
    point.z_m = (float)z_mm / 1000.0f;
    point.yaw_deg = (float)yaw_cdeg / 100.0f;

    QueryPerformanceCounter(&start);
    status = LoraNav_ProcessPoint(&g_ctx, &point, &nav_out);
    QueryPerformanceCounter(&end);
    test_fill_output(&nav_out, status, out);
    test_update_debug(status, test_elapsed_us(start, end));
    return (int32_t)status;
}

NAV_EXPORT int32_t DijkstraTest_EnterReturn(DijkstraTestOutput *out)
{
    LoraNavOutput nav_out;
    LoraNavStatus status;
    LARGE_INTEGER start;
    LARGE_INTEGER end;

    ZeroMemory(&nav_out, sizeof(nav_out));
    QueryPerformanceCounter(&start);
    status = LoraNav_EnterReturnMode(&g_ctx);
    QueryPerformanceCounter(&end);
    nav_out.valid = true;
    nav_out.return_mode = g_ctx.return_mode;
    nav_out.route_valid = g_ctx.route_valid;
    test_fill_output(&nav_out, status, out);
    test_update_debug(status, test_elapsed_us(start, end));
    return (int32_t)status;
}

NAV_EXPORT int32_t DijkstraTest_GetNode(int32_t index, int32_t *x_mm, int32_t *y_mm)
{
    if (index < 0 || index >= (int32_t)g_ctx.node_count || x_mm == NULL || y_mm == NULL)
    {
        return 0;
    }
    *x_mm = nav_meters_to_mm(g_ctx.nodes[index].x_m);
    *y_mm = nav_meters_to_mm(g_ctx.nodes[index].y_m);
    return 1;
}

NAV_EXPORT int32_t DijkstraTest_GetEdge(int32_t index, int32_t *node_a, int32_t *node_b)
{
    int32_t current = 0;
    uint16_t a;
    if (index < 0 || node_a == NULL || node_b == NULL)
    {
        return 0;
    }

    for (a = 0u; a < g_ctx.node_count; a++)
    {
        uint16_t b;
        for (b = (uint16_t)(a + 1u); b < g_ctx.node_count; b++)
        {
            if (g_ctx.edges[a][b] < LORA_NAV_NO_EDGE)
            {
                if (current == index)
                {
                    *node_a = a;
                    *node_b = b;
                    return 1;
                }
                current++;
            }
        }
    }
    return 0;
}

NAV_EXPORT int32_t DijkstraTest_GetRoutePoint(int32_t index, int32_t *x_mm, int32_t *y_mm)
{
    if (index < 0 || index >= (int32_t)g_ctx.route_count || x_mm == NULL || y_mm == NULL)
    {
        return 0;
    }
    *x_mm = nav_meters_to_mm(g_ctx.route_points[index].x_m);
    *y_mm = nav_meters_to_mm(g_ctx.route_points[index].y_m);
    return 1;
}

NAV_EXPORT int32_t DijkstraTest_GetDebug(DijkstraTestDebug *debug)
{
    if (debug == NULL)
    {
        return 0;
    }
    *debug = g_debug;
    return 1;
}

NAV_EXPORT uint32_t DijkstraTest_GetContextSize(void)
{
    return (uint32_t)sizeof(g_ctx);
}
