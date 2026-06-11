import ctypes
import os


# ctypes structures use fixed-width fields to match the Windows test wrapper.

STATUS_NAMES = {
    0: "OK",
    1: "FULL",
    2: "NO_ROUTE",
    3: "BAD_ARG",
}


class _Output(ctypes.Structure):
    _fields_ = [
        ("status", ctypes.c_int32),
        ("valid", ctypes.c_int32),
        ("return_mode", ctypes.c_int32),
        ("route_valid", ctypes.c_int32),
        ("arrived_home", ctypes.c_int32),
        ("corrected_x_mm", ctypes.c_int32),
        ("corrected_y_mm", ctypes.c_int32),
        ("corrected_z_mm", ctypes.c_int32),
        ("corrected_yaw_cdeg", ctypes.c_int32),
        ("next_x_mm", ctypes.c_int32),
        ("next_y_mm", ctypes.c_int32),
        ("distance_to_next_mm", ctypes.c_int32),
        ("bearing_to_next_cdeg", ctypes.c_int32),
        ("relative_bearing_cdeg", ctypes.c_int32),
        ("next_route_index", ctypes.c_int32),
    ]


class _Debug(ctypes.Structure):
    _fields_ = [
        ("process_count", ctypes.c_uint32),
        ("last_process_us", ctypes.c_uint32),
        ("max_process_us", ctypes.c_uint32),
        ("checked_edge_count", ctypes.c_uint32),
        ("intersection_count", ctypes.c_uint32),
        ("merged_node_count", ctypes.c_uint32),
        ("node_count", ctypes.c_uint32),
        ("key_count", ctypes.c_uint32),
        ("edge_count", ctypes.c_uint32),
        ("route_count", ctypes.c_uint32),
        ("point_count", ctypes.c_uint32),
        ("total_point_count", ctypes.c_uint32),
        ("last_status", ctypes.c_int32),
        ("home_node_id", ctypes.c_int32),
        ("last_key_node_id", ctypes.c_int32),
        ("return_mode", ctypes.c_int32),
        ("route_valid", ctypes.c_int32),
    ]


class CNavEngine:
    """Small Python adapter around the embedded navigation test DLL."""

    def __init__(self, dll_path=None):
        if dll_path is None:
            dll_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "dijkstra.dll")
        self.dll_path = dll_path
        self.dll = ctypes.CDLL(dll_path)
        self._bind()

    def _bind(self):
        dll = self.dll
        dll.DijkstraTest_Init.argtypes = [
            ctypes.c_float,
            ctypes.c_float,
            ctypes.c_float,
            ctypes.c_float,
            ctypes.c_float,
            ctypes.c_int32,
            ctypes.c_int32,
        ]
        dll.DijkstraTest_Init.restype = ctypes.c_int32

        dll.DijkstraTest_InputPoint.argtypes = [
            ctypes.c_int32,
            ctypes.c_int32,
            ctypes.c_int32,
            ctypes.c_int32,
            ctypes.POINTER(_Output),
        ]
        dll.DijkstraTest_InputPoint.restype = ctypes.c_int32
        dll.DijkstraTest_EnterReturn.argtypes = [ctypes.POINTER(_Output)]
        dll.DijkstraTest_EnterReturn.restype = ctypes.c_int32
        dll.DijkstraTest_GetNode.argtypes = [
            ctypes.c_int32,
            ctypes.POINTER(ctypes.c_int32),
            ctypes.POINTER(ctypes.c_int32),
        ]
        dll.DijkstraTest_GetNode.restype = ctypes.c_int32
        dll.DijkstraTest_GetEdge.argtypes = [
            ctypes.c_int32,
            ctypes.POINTER(ctypes.c_int32),
            ctypes.POINTER(ctypes.c_int32),
        ]
        dll.DijkstraTest_GetEdge.restype = ctypes.c_int32
        dll.DijkstraTest_GetRoutePoint.argtypes = [
            ctypes.c_int32,
            ctypes.POINTER(ctypes.c_int32),
            ctypes.POINTER(ctypes.c_int32),
        ]
        dll.DijkstraTest_GetRoutePoint.restype = ctypes.c_int32
        dll.DijkstraTest_GetDebug.argtypes = [ctypes.POINTER(_Debug)]
        dll.DijkstraTest_GetDebug.restype = ctypes.c_int32
        dll.DijkstraTest_GetContextSize.argtypes = []
        dll.DijkstraTest_GetContextSize.restype = ctypes.c_uint32

    def initialize(self, turn_angle_deg, min_segment_m, merge_distance_m,
                   snap_distance_m, direction_match_deg,
                   forward_snap=False, return_snap=True):
        return int(self.dll.DijkstraTest_Init(
            float(turn_angle_deg),
            float(min_segment_m),
            float(merge_distance_m),
            float(snap_distance_m),
            float(direction_match_deg),
            int(bool(forward_snap)),
            int(bool(return_snap)),
        ))

    def input_point(self, point):
        x_m, y_m, z_m, yaw_deg = point
        out = _Output()
        self.dll.DijkstraTest_InputPoint(
            round(x_m * 1000.0),
            round(y_m * 1000.0),
            round(z_m * 1000.0),
            round(yaw_deg * 100.0),
            ctypes.byref(out),
        )
        return self._output_dict(out)

    def enter_return(self):
        out = _Output()
        self.dll.DijkstraTest_EnterReturn(ctypes.byref(out))
        return self._output_dict(out)

    def debug(self):
        value = _Debug()
        if not self.dll.DijkstraTest_GetDebug(ctypes.byref(value)):
            return {}
        return {name: getattr(value, name) for name, _ctype in value._fields_}

    def nodes(self):
        debug = self.debug()
        result = []
        for index in range(debug.get("node_count", 0)):
            x_mm = ctypes.c_int32()
            y_mm = ctypes.c_int32()
            if self.dll.DijkstraTest_GetNode(index, ctypes.byref(x_mm), ctypes.byref(y_mm)):
                result.append((x_mm.value / 1000.0, y_mm.value / 1000.0))
        return result

    def edges(self):
        debug = self.debug()
        result = []
        for index in range(debug.get("edge_count", 0)):
            node_a = ctypes.c_int32()
            node_b = ctypes.c_int32()
            if self.dll.DijkstraTest_GetEdge(index, ctypes.byref(node_a), ctypes.byref(node_b)):
                result.append((node_a.value, node_b.value))
        return result

    def route(self):
        debug = self.debug()
        result = []
        for index in range(debug.get("route_count", 0)):
            x_mm = ctypes.c_int32()
            y_mm = ctypes.c_int32()
            if self.dll.DijkstraTest_GetRoutePoint(index, ctypes.byref(x_mm), ctypes.byref(y_mm)):
                result.append((x_mm.value / 1000.0, y_mm.value / 1000.0))
        return result

    def context_size(self):
        return int(self.dll.DijkstraTest_GetContextSize())

    @staticmethod
    def _output_dict(out):
        return {
            "status": out.status,
            "status_name": STATUS_NAMES.get(out.status, str(out.status)),
            "valid": bool(out.valid),
            "return_mode": bool(out.return_mode),
            "route_valid": bool(out.route_valid),
            "arrived_home": bool(out.arrived_home),
            "corrected_point": (
                out.corrected_x_mm / 1000.0,
                out.corrected_y_mm / 1000.0,
                out.corrected_z_mm / 1000.0,
                out.corrected_yaw_cdeg / 100.0,
            ),
            "next_point": (out.next_x_mm / 1000.0, out.next_y_mm / 1000.0),
            "distance_to_next_m": out.distance_to_next_mm / 1000.0,
            "bearing_to_next_deg": out.bearing_to_next_cdeg / 100.0,
            "relative_bearing_deg": out.relative_bearing_cdeg / 100.0,
            "next_route_index": out.next_route_index,
        }


if __name__ == "__main__":
    engine = CNavEngine()
    engine.initialize(60.0, 0.5, 3.0, 3.0, 45.0, False, True)
    for point in ((0.0, 0.0, 0.0, 0.0), (0.0, 1.0, 0.0, 0.0), (1.0, 1.0, 0.0, 90.0)):
        print(engine.input_point(point))
    print(engine.debug())
    print("context_size", engine.context_size())
