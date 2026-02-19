import xml.etree.ElementTree as ET
from typing import List
import math
import networkx as nx

from src.rsa.RSA import RSA
from src.PhysicalTopology import PhysicalTopology
from src.VirtualTopology import VirtualTopology
from src.ControlPlaneForRSA import ControlPlaneForRSA
from src.TrafficGenerator import TrafficGenerator
from src.Flow import Flow
from src.Slot import Slot
from src.PCycle import PCycle
from src.LightPath import LightPath
from src.ProtectingLightPath import ProtectingLightPath


class BfsRSA(RSA):
    def __init__(self):
        self.pt: PhysicalTopology = None
        self.vt: VirtualTopology = None
        self.cp: ControlPlaneForRSA = None
        self.graph = None

    def simulation_interface(self, xml: ET.Element, pt: PhysicalTopology,
                             vt: VirtualTopology, cp: ControlPlaneForRSA,
                             traffic: TrafficGenerator):
        self.pt = pt
        self.vt = vt
        self.cp = cp
        self.graph = pt.get_weighted_graph()

    # ================== BFS UTILS ==================

    def bfs_path(self, graph: nx.Graph, src: int, dst: int, banned_edges=None):
        from collections import deque

        if banned_edges is None:
            banned_edges = set()

        if src not in graph or dst not in graph:
            return None

        queue = deque([src])
        parent = {src: None}

        while queue:
            u = queue.popleft()
            if u == dst:
                break

            for v in graph.neighbors(u):
                if (u, v) in banned_edges or (v, u) in banned_edges:
                    continue

                if v not in parent:
                    parent[v] = u
                    queue.append(v)

        if dst not in parent:
            return None

        path = []
        cur = dst
        while cur is not None:
            path.append(cur)
            cur = parent[cur]

        return list(reversed(path))


    def get_two_shortest_disjoint_paths(self, flow: Flow, search_graph: nx.Graph):

    # Path 1
        path1 = self.bfs_path(
            search_graph,
            flow.get_source(),
            flow.get_destination()
        )

        if path1 is None:
            return None, None, None, None

    # Mark edges of path1 as banned
        banned_edges = set()
        for i in range(len(path1) - 1):
            u = path1[i]
            v = path1[i + 1]
            banned_edges.add((u, v))

    # Path 2 (avoid edges of path1)
        path2 = self.bfs_path(
            search_graph,
            flow.get_source(),
            flow.get_destination(),
            banned_edges=banned_edges
        )

        if path2 is None:
            return None, None, None, None

        return len(path1), path1, len(path2), path2


    # ================== REMOVE USED EDGES ==================
    # FIX 1 + FIX 3: Không xóa cạnh nữa

    def remove_used_edges(self, graph: nx.Graph):
        # Không xóa cạnh LP
        # Không xóa cạnh p-cycle
        # Tất cả thông tin chiếm dụng đã nằm trong spectrum
        return

    # ================== FIND WORKING PATH ==================

    def find_working_path(self, flow: Flow, demand_in_slots: int):
        new_graph = self.pt.get_graph().copy()

        # attach spectrum info
        for u, v in new_graph.edges():
            edge_spectrum = self.pt.get_spectrum(u, v)
            new_graph[u][v]['spectrum'] = edge_spectrum

        # FIX: remove_used_edges không còn xóa cạnh
        self.remove_used_edges(new_graph)

        best_path = None
        best_slot_index = None
        weight_best_path = float("inf")

        for i in range(0, self.pt.get_num_slots() - demand_in_slots + 1):
            search_graph = nx.Graph()

            for u, v, edge_data in new_graph.edges(data=True):
                spectrum = edge_data['spectrum']
                if all(spectrum[0][i:i + demand_in_slots]):
                    search_graph.add_edge(u, v)

            path = self.bfs_path(search_graph, flow.get_source(), flow.get_destination())
            if path is not None:
                length = len(path) - 1
                if length < weight_best_path:
                    best_path = path
                    best_slot_index = i
                    weight_best_path = length

        return best_path, best_slot_index, weight_best_path

    # ================== CREATE P-CYCLE ==================

    def create_p_cycle(self, flow: Flow, working_path: List[int],
                   slot_index: int, demand_in_slots: int):

        new_graph = self.pt.get_graph().copy()

    # attach spectrum info
        for u, v in new_graph.edges():
            edge_spectrum = self.pt.get_spectrum(u, v)
            new_graph[u][v]['spectrum'] = edge_spectrum

    # FIX: không xóa cạnh LP/p-cycle
        self.remove_used_edges(new_graph)

    # ============================
    # HÀM CHECK DISJOINT
    # ============================
        def is_disjoint(path, wp):
            wp_edges = {(wp[i], wp[i+1]) for i in range(len(wp)-1)}
            p_edges = {(path[i], path[i+1]) for i in range(len(path)-1)}
            return wp_edges.isdisjoint(p_edges)

        best_path_1 = None
        best_path_2 = None
        best_slot_index = None
        weight_best_path = float("inf")

    # ============================
    # SLOT LOOP
    # ============================
        for j in range(0, self.pt.get_num_slots() - demand_in_slots + 1):

            search_graph = nx.Graph()

        # Build graph with available spectrum
            for u, v, edge_data in new_graph.edges(data=True):
                spectrum = edge_data['spectrum'][0]  # adjust if needed
                if all(spectrum[j:j + demand_in_slots]):
                    search_graph.add_edge(u, v)

        # Find two edge-disjoint paths
            length_1, path1, length_2, path2 = \
                self.get_two_shortest_disjoint_paths(flow, search_graph)

            if not (path1 and path2):
                continue

        # ============================
        # NEW LOGIC:
        # Chỉ cần 1 path disjoint với working path
        # ============================
            disjoint1 = is_disjoint(path1, working_path)
            disjoint2 = is_disjoint(path2, working_path)

            if not (disjoint1 or disjoint2):
                continue

        # Choose best p-cycle
            if weight_best_path > length_1 + length_2:
                best_path_1 = path1
                best_path_2 = path2
                best_slot_index = j
                weight_best_path = length_1 + length_2

        return best_path_1, best_path_2, best_slot_index

    # ================== FIPP-FLEX LOGIC ==================

    def convert_slot(self, index: int, demand: int):
        return [Slot(0, s_idx) for s_idx in range(index, index + demand)]

    def fippflexai(self, flow: Flow, demand_in_slots: int):
        working_path, slot_index, weight_best_path = self.find_working_path(flow, demand_in_slots)

        if working_path and weight_best_path != float("inf"):
            wp_slot_list = self.convert_slot(slot_index, demand_in_slots)

            wp_links = [
                self.pt.get_link_id(working_path[j], working_path[j + 1])
                for j in range(len(working_path) - 1)
            ]

            # Try reuse p-cycle
            for pcycle in self.vt.get_p_cycles():
                if pcycle.p_cycle_contains_flow(flow.get_source(), flow.get_destination()) \
                    and pcycle.get_reserved_slots() >= demand_in_slots:

                    if pcycle.can_reuse_with_slots(wp_links, wp_slot_list):
                        return pcycle, wp_links, wp_slot_list, True


            # Create new p-cycle
            path_1_pcycle, path_2_pcycle, slot_index_pcycle = self.create_p_cycle(
                flow, working_path, slot_index, demand_in_slots
            )

            if path_1_pcycle and path_2_pcycle:
                links_1 = [
                    self.pt.get_link_id(path_1_pcycle[i], path_1_pcycle[i + 1])
                    for i in range(len(path_1_pcycle) - 1)
                ]
                links_2 = [
                    self.pt.get_link_id(path_2_pcycle[i], path_2_pcycle[i + 1])
                    for i in range(len(path_2_pcycle) - 1)
                ]

                p_cycle_links = links_1 + links_2
                p_cycle_nodes = list(set(path_1_pcycle) | set(path_2_pcycle))

                new_p_cycle = PCycle(
                    cycle_links=p_cycle_links,
                    nodes=p_cycle_nodes,
                    slot_list=self.convert_slot(slot_index_pcycle, demand_in_slots),
                    reserved_slots=demand_in_slots
                )

                self.vt.add_p_cycles(new_p_cycle)

                return new_p_cycle, wp_links, wp_slot_list, False

        return None, None, None, False

    # ================== FLOW ARRIVAL ==================

    def flow_arrival(self, flow: Flow) -> None:
        demand_in_slots = math.ceil(flow.get_rate() / self.pt.get_slot_capacity())

        p_cycle, wp_links, wp_slot_list, pcycle_reuse = self.fippflexai(flow, demand_in_slots)

        if p_cycle is not None:
            establish, lp_id = self.establish_connection(
                links=wp_links,
                slot_list=wp_slot_list,
                flow=flow,
                pcycle=p_cycle,
                reused=pcycle_reuse
            )

            protected_lp = ProtectingLightPath(
                id=lp_id,
                src=flow.get_source(),
                dst=flow.get_destination(),
                links_id=wp_links,
                fss=demand_in_slots
            )

            for edge in wp_links:
                self.pt.reserve_slots(
                    self.pt.get_src_link(edge),
                    self.pt.get_dst_link(edge),
                    wp_slot_list
                )

            if not pcycle_reuse:
                for p_link in p_cycle.get_cycle_links():
                    self.pt.reserve_slots(
                        self.pt.get_src_link(p_link),
                        self.pt.get_dst_link(p_link),
                        p_cycle.get_slot_list()
                    )

            p_cycle.add_protected_lightpath(protected_lp)
            return

        self.cp.block_flow(flow.get_id())

    # ================== ESTABLISH CONNECTION ==================

    def establish_connection(self, links: List[int], slot_list: List[Slot],
                             flow: Flow, pcycle: PCycle, reused: bool):
        lp_id = self.vt.create_light_path(flow, links, slot_list, 0, pcycle)
        if lp_id >= 0:
            lps = self.vt.get_light_path(lp_id)
            flow.set_links(links)
            flow.set_slot_list(slot_list)
            self.cp.accept_flow(flow.get_id(), lps, reused)
            return True, lp_id
        return False, None

    # ================== FLOW DEPARTURE ==================

    def flow_departure(self, flow: Flow) -> None:
        if flow is None:
            return

        src = flow.get_source()
        dst = flow.get_destination()
        flow_links = flow.get_links()
        flow_slots = flow.get_slot_list()

        if not flow_links or not flow_slots:
            return

        lps_to_remove = []

        for u, v, key, data in list(self.vt.g_lightpath.edges(data=True, keys=True)):

            if "lightpath" not in data:
                continue

            lp = data["lightpath"]
            if lp is None:
                continue

            if lp.get_source() == src and lp.get_destination() == dst:
                if lp.get_links() == flow_links and lp.get_slot_list() == flow_slots:
                    lps_to_remove.append(lp)

        for lp in lps_to_remove:
            self.vt.remove_lp_p_cycle(lp)
            self.vt.remove_light_path(lp.get_id())

        if hasattr(self.cp,"flow_departure"):
            self.cp.flow_departure(flow.get_id())
            
