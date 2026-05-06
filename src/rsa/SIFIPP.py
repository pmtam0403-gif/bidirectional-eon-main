import math
from collections import deque

from src.rsa.RSA import RSA
from src.Flow import Flow
from src.Slot import Slot
from src.PCycle import PCycle
from src.ProtectingLightPath import ProtectingLightPath


class SIFIPP(RSA):

    def __init__(self):
        self.pt = None
        self.vt = None
        self.cp = None

        self.reused_pcycles = 0
        self.new_pcycles = 0

    def simulation_interface(self, xml, pt, vt, cp, traffic):
        self.pt = pt
        self.vt = vt
        self.cp = cp


    def spectrum_to_bitmap(self, spectrum_2d):
        bitmap = 0
        for i, free in enumerate(spectrum_2d[0]):
            if free:
                bitmap |= (1 << i)
        return bitmap

    def has_contiguous(self, bitmap, demand):
        cnt = 0
        for i in range(self.pt.get_num_slots()):
            if bitmap & (1 << i):
                cnt += 1
                if cnt >= demand:
                    return True
            else:
                cnt = 0
        return False

    def bitmap_to_slots(self, bitmap, demand):
        cnt = 0
        start = 0
        for i in range(self.pt.get_num_slots()):
            if bitmap & (1 << i):
                if cnt == 0:
                    start = i
                cnt += 1
                if cnt >= demand:
                    return [Slot(0, j) for j in range(start, start + demand)]
            else:
                cnt = 0
        return None

    def bfs_incremental(self, src, dst, demand,
                        banned_links=None,
                        forbidden_slots_bitmap=None):

        if banned_links is None:
            banned_links = set()

        full_bitmap = (1 << self.pt.get_num_slots()) - 1
        queue = deque([(src, full_bitmap, [src])])
        visited = set()

        while queue:
            node, bitmap, path = queue.popleft()

            if node == dst:
                candidate = bitmap
                if forbidden_slots_bitmap is not None:
                    candidate &= ~forbidden_slots_bitmap
                if self.has_contiguous(candidate, demand):
                    return path, candidate

            for neigh in self.pt.get_graph().neighbors(node):
                link_id = self.pt.get_link_id(node, neigh)

                if link_id in banned_links:
                    continue

                edge_bitmap = self.spectrum_to_bitmap(
                    self.pt.get_spectrum(node, neigh)
                )

                new_bitmap = bitmap & edge_bitmap

                if new_bitmap == 0:
                    continue

                candidate = new_bitmap
                if forbidden_slots_bitmap is not None:
                    candidate &= ~forbidden_slots_bitmap

                if not self.has_contiguous(candidate, demand):
                    continue

                state_key = (neigh, new_bitmap)
                if state_key in visited:
                    continue

                visited.add(state_key)
                queue.append((neigh, new_bitmap, path + [neigh]))

        return None, None

    # ============================================================
    # WORKING PATH
    # ============================================================
    def find_working_path(self, flow, demand):
        path, bitmap = self.bfs_incremental(
            flow.get_source(),
            flow.get_destination(),
            demand
        )

        if not path:
            return None, None, None

        slots = self.bitmap_to_slots(bitmap, demand)
        if slots is None:
            return None, None, None

        links = [
            self.pt.get_link_id(path[i], path[i + 1])
            for i in range(len(path) - 1)
        ]

        return path, links, slots

    # ============================================================
    # BUILD NEW PCYCLE
    # ============================================================
    def build_pcycle(self, flow, wp_path, wp_links, wp_slots):

        demand = len(wp_slots)

        wp_bitmap = 0
        for s in wp_slots:
            wp_bitmap |= (1 << s.slot)

        path2, bitmap2 = self.bfs_incremental(
            flow.get_source(),
            flow.get_destination(),
            demand,
            banned_links=set(wp_links),
            forbidden_slots_bitmap=wp_bitmap
        )

        if not path2:
            return None, None, None

        slots2 = self.bitmap_to_slots(bitmap2, demand)
        if slots2 is None:
            return None, None, None

        if not set(wp_path[1:-1]).isdisjoint(set(path2[1:-1])):
            return None, None, None

        links2 = [
            self.pt.get_link_id(path2[i], path2[i + 1])
            for i in range(len(path2) - 1)
        ]

        cycle_nodes = wp_path + path2[-2:0:-1]

        cycle_links = []
        for i in range(len(cycle_nodes) - 1):
            cycle_links.append(
                self.pt.get_link_id(cycle_nodes[i], cycle_nodes[i + 1])
            )

        cycle_links.append(
            self.pt.get_link_id(cycle_nodes[-1], cycle_nodes[0])
        )

        slot_ids = sorted(
            set(s.slot for s in wp_slots) |
            set(s.slot for s in slots2)
        )

        pcycle_slots = [Slot(0, i) for i in slot_ids]

        pcycle = PCycle(
            cycle_links,
            cycle_nodes,
            pcycle_slots,
            len(pcycle_slots)
        )

        return pcycle, links2, pcycle_slots


    def try_reuse_pcycle(self, wp_links, wp_slots):

        new_wp_set = set(wp_links)

        for pcycle in self.vt.get_p_cycles():

            conflict = False
            for lp in pcycle.get_protected_lightpaths():
                if new_wp_set & set(lp.get_links()):
                    conflict = True
                    break

            if conflict:
                continue

            cycle_links = set(pcycle.get_cycle_links())
            cycle_nodes = set(pcycle.get_nodes())

            is_on_cycle = new_wp_set.issubset(cycle_links)

            def is_straddling(link_id):
                u = self.pt.get_src_link(link_id)
                v = self.pt.get_dst_link(link_id)

                if u not in cycle_nodes or v not in cycle_nodes:
                    return False

                if link_id in cycle_links:
                    return False

                return True

            is_straddle = all(
                is_straddling(e) for e in wp_links
            )

            if not (is_on_cycle or is_straddle):
                continue

            p_slots = set(s.slot for s in pcycle.get_slot_list())
            wp_set = set(s.slot for s in wp_slots)

            if not wp_set.issubset(p_slots):
                continue

            return pcycle

        return None

    def flow_arrival(self, flow: Flow):

        demand = math.ceil(
            flow.get_rate() /
            self.pt.get_slot_capacity()
        )

        wp_path, wp_links, wp_slots = self.find_working_path(
            flow, demand
        )

        if not wp_links:
            self.cp.block_flow(flow.get_id())
            return

        reused = self.try_reuse_pcycle(wp_links, wp_slots)

        if reused:

            self.reused_pcycles += 1

            lp_id = self.vt.create_light_path(
                flow,
                wp_links,
                wp_slots,
                0,
                reused
            )

            if lp_id < 0:
                self.cp.block_flow(flow.get_id())
                return

            for edge in wp_links:
                self.pt.reserve_slots(
                    self.pt.get_src_link(edge),
                    self.pt.get_dst_link(edge),
                    wp_slots
                )

            reused.add_protected_lightpath(
                ProtectingLightPath(
                    lp_id,
                    flow.get_source(),
                    flow.get_destination(),
                    wp_links,
                    len(wp_slots),
                    []
                )
            )

            self.cp.accept_flow(
                flow.get_id(),
                self.vt.get_light_path(lp_id),
                False
            )
            return

        self.new_pcycles += 1

        pcycle, bp_links, bp_slots = self.build_pcycle(
            flow, wp_path, wp_links, wp_slots
        )

        if not pcycle:
            self.cp.block_flow(flow.get_id())
            return

        self.vt.add_p_cycles(pcycle)

        lp_id = self.vt.create_light_path(
            flow,
            wp_links,
            wp_slots,
            0,
            pcycle
        )

        if lp_id < 0:
            self.cp.block_flow(flow.get_id())
            return

        for edge in wp_links:
            self.pt.reserve_slots(
                self.pt.get_src_link(edge),
                self.pt.get_dst_link(edge),
                wp_slots
            )

        for edge in bp_links:
            self.pt.reserve_slots(
                self.pt.get_src_link(edge),
                self.pt.get_dst_link(edge),
                bp_slots
            )

        pcycle.add_protected_lightpath(
            ProtectingLightPath(
                lp_id,
                flow.get_source(),
                flow.get_destination(),
                wp_links,
                len(wp_slots),
                [(bp_links, bp_slots)]
            )
        )

        self.cp.accept_flow(
            flow.get_id(),
            self.vt.get_light_path(lp_id),
            False
        )
    def flow_departure(self, flow: Flow):
        pass
