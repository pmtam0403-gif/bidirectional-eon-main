import xml.etree.ElementTree as ET
import math
import networkx as nx
from typing import List, Optional

from src.rsa.RSA import RSA
from src.PhysicalTopology import PhysicalTopology
from src.VirtualTopology import VirtualTopology
from src.ControlPlaneForRSA import ControlPlaneForRSA
from src.TrafficGenerator import TrafficGenerator
from src.Flow import Flow
from src.Slot import Slot
from src.PCycle import PCycle
from src.ProtectingLightPath import ProtectingLightPath


class FIPPBFS(RSA):

    def __init__(self):
        self.pt: Optional[PhysicalTopology] = None
        self.vt: Optional[VirtualTopology] = None
        self.cp: Optional[ControlPlaneForRSA] = None
        self.graph = None

    def simulation_interface(self, xml: ET.Element, pt: PhysicalTopology,
                             vt: VirtualTopology, cp: ControlPlaneForRSA,
                             traffic: TrafficGenerator):
        self.pt = pt
        self.vt = vt
        self.cp = cp
        self.graph = pt.get_weighted_graph()


    def flow_arrival(self, flow: Flow) -> None:
        demand = math.ceil(flow.get_rate() / self.pt.get_slot_capacity())

        # 1) Try reuse P-cycle
        for pcycle in self.vt.get_p_cycles():
            if pcycle.p_cycle_contains_flow(flow.get_source(), flow.get_destination()):
                ok, slot_list = self.try_extend_pcycle(pcycle, demand)
                if ok:
                    ok2, links, slots, backups = self.find_working_path(flow, demand)
                    if ok2:
                        self.create_lightpath(flow, links, slots, pcycle, backups, reused=True)
                        return

        # 2) Try create new P-cycle
        ok, wp_links, wp_slots, backup_paths, p_links, p_nodes, p_slots = \
            self.initialize_fipp(flow, demand)

        if ok:
            pcycle = self.establish_pcycle(p_links, p_nodes, p_slots, demand)

            self.vt.add_p_cycles(pcycle)
            self.create_lightpath(flow, wp_links, wp_slots, pcycle, backup_paths, reused=False)
            return

        # 3) Block
        self.cp.block_flow(flow.get_id())


    def find_working_path(self, flow, demand):
        src, dst = flow.get_source(), flow.get_destination()
        G = self.pt.get_graph()

        try:
            paths = nx.shortest_simple_paths(G, src, dst)
        except nx.NetworkXNoPath:
            return False, None, None, None

        for path in paths:
            spectrum = [[True]*self.pt.get_num_slots()
                        for _ in range(self.pt.get_cores())]

            for i in range(len(path)-1):
                spectrum = self.image_and(
                    self.pt.get_spectrum(path[i], path[i+1]),
                    spectrum, spectrum
                )

            ok, _, slots = self.calculate_slot_range(spectrum, demand)
            if ok:
                links = [self.pt.get_link_id(path[i], path[i+1])
                        for i in range(len(path)-1)]
                return True, links, slots, []

        return False, None, None, None

    def create_lightpath(self, flow: Flow, links: List[int], slot_list: List[Slot],
                          pcycle: PCycle, backup_paths: List[List[int]], reused: bool):

        lp_id = self.vt.create_light_path(flow, links, slot_list, 0, pcycle)

        if lp_id >= 0:
            lps = self.vt.get_light_path(lp_id)
            flow.set_links(links)
            flow.set_slot_list(slot_list)
            self.cp.accept_flow(flow.get_id(), lps, reused)

            for edge in pcycle.get_cycle_links():
                self.pt.reserve_slots(self.pt.get_src_link(edge),
                                      self.pt.get_dst_link(edge),
                                      pcycle.get_slot_list())

            protected_lp = ProtectingLightPath(lp_id, flow.get_source(),
                                               flow.get_destination(),
                                               links, len(slot_list),
                                               backup_paths)
            pcycle.add_protected_lightpath(protected_lp)

    def initialize_fipp(self, flow: Flow, demand: int):
        path1, path2 = self.get_two_edge_disjoint_paths(flow)
        if not path1 or not path2:
            return False, None, None, None, None, None, None

        spectrum = [[True for _ in range(self.pt.get_num_slots())]
                    for _ in range(self.pt.get_cores())]

        for i in range(len(path1)-1):
            spectrum = self.image_and(self.pt.get_spectrum(path1[i], path1[i+1]),
                                      spectrum, spectrum)

        for i in range(len(path2)-1):
            spectrum = self.image_and(self.pt.get_spectrum(path2[i], path2[i+1]),
                                      spectrum, spectrum)

        ok, _, pcycle_slots = self.calculate_slot_range(spectrum, demand)
        if not ok:
            return False, None, None, None, None, None, None

        spectrum1 = [row.copy() for row in spectrum]
        for s in pcycle_slots:
            spectrum1[s.core][s.slot] = False

        ok1, _, wp_slots = self.calculate_slot_range(spectrum1, demand)
        if ok1:
            links1 = [self.pt.get_link_id(path1[i], path1[i+1])
                      for i in range(len(path1)-1)]
            links2 = [self.pt.get_link_id(path2[i], path2[i+1])
                      for i in range(len(path2)-1)]
            return True, links1, wp_slots, [links2], links1+links2, \
                   list(set(path1)|set(path2)), pcycle_slots

        spectrum2 = [row.copy() for row in spectrum]
        for s in pcycle_slots:
            spectrum2[s.core][s.slot] = False

        ok2, _, wp_slots = self.calculate_slot_range(spectrum2, demand)
        if ok2:
            links2 = [self.pt.get_link_id(path2[i], path2[i+1])
                      for i in range(len(path2)-1)]
            links1 = [self.pt.get_link_id(path1[i], path1[i+1])
                      for i in range(len(path1)-1)]
            return True, links2, wp_slots, [links1], links1+links2, \
                   list(set(path1)|set(path2)), pcycle_slots

        return False, None, None, None, None, None, None
##
    def establish_pcycle(self, p_links, p_nodes, p_slots, demand):

        pcycle = PCycle(p_links, p_nodes, p_slots)


        for edge in p_links:
            src = self.pt.get_src_link(edge)
            dst = self.pt.get_dst_link(edge)
            self.pt.reserve_slots(src, dst, p_slots)

        return pcycle

    def get_two_edge_disjoint_paths(self, flow: Flow):
        src = flow.get_source()
        dst = flow.get_destination()

        G = self.pt.get_graph()

        if src not in G.nodes or dst not in G.nodes:

            return None, None

        try:
            path1 = nx.shortest_path(G, src, dst, weight="weight")
        except nx.NetworkXNoPath:
            return None, None

        G2 = G.copy()
        for i in range(len(path1)-1):
            u = path1[i]
            v = path1[i+1]
            if G2.has_edge(u, v):
                G2.remove_edge(u, v)

        try:
            path2 = nx.shortest_path(G2, src, dst, weight="weight")
        except nx.NetworkXNoPath:
            return path1, None

        return path1, path2


    def image_and(self, img1, img2, res):
        for i in range(len(res)):
            for j in range(len(res[0])):
                res[i][j] = img1[i][j] and img2[i][j]
        return res

    def calculate_slot_range(self, spectrum, demand):
        for c_idx, row in enumerate(spectrum):
            for i in range(len(row)-demand+1):
                if all(row[i:i+demand]):
                    slot_list = [Slot(c_idx, j) for j in range(i, i+demand)]
                    for j in range(i, i+demand):
                        row[j] = False
                    return True, spectrum, slot_list
        return False, spectrum, None

    def extend_or_replace_false(self, lst, core_idx, start, end, demand):
        row = lst[core_idx]
        length = end - start + 1
        need = demand - length
        left, right = start-1, end+1
        indices = list(range(start, end+1))

        while need > 0:
            if left >= 0 and row[left]:
                row[left] = False
                indices.insert(0, left)
                left -= 1
                need -= 1
            elif right < len(row) and row[right]:
                row[right] = False
                indices.append(right)
                right += 1
                need -= 1
            else:
                break

        if need == 0:
            return lst, (core_idx, indices)

        for c_idx, r in enumerate(lst):
            for i in range(len(r)-demand+1):
                if all(r[i:i+demand]):
                    for j in range(i, i+demand):
                        r[j] = False
                    return lst, (c_idx, list(range(i, i+demand)))

        return lst, None

    def flow_departure(self, flow: Flow) -> None:

        return

    def try_extend_pcycle(self, pcycle: PCycle, demand: int):
      
        if pcycle.has_sufficient_slots(demand):
            return True, pcycle.get_slot_list()


        spectrum = [[True for _ in range(self.pt.get_num_slots())]
                    for _ in range(self.pt.get_cores())]

        for edge in pcycle.get_cycle_links():
            spectrum = self.image_and(
                self.pt.get_spectrum(self.pt.get_src_link(edge),
                                     self.pt.get_dst_link(edge)),
                spectrum,
                spectrum
            )


        core, start, end = pcycle.get_core_slot_range()

        spectrum, idx = self.extend_or_replace_false(spectrum, core, start, end, demand)
        if idx is None:
            return False, None

        core_idx, slots = idx
        slot_list = [Slot(core_idx, i) for i in slots]
        return True, slot_list
