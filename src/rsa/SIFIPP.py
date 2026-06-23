import math
from collections import deque

from src.rsa.RSA import RSA
from src.Flow import Flow
from src.Slot import Slot
from src.PCycle import PCycle
from src.ProtectingLightPath import ProtectingLightPath


class SIFIPPBFS(RSA):

    def __init__(self):

        self.pt = None
        self.vt = None
        self.cp = None

        self.graph = None
        self.full_bitmap = None

        self.reused_pcycles = 0
        self.new_pcycles = 0

    def simulation_interface(self, xml, pt, vt, cp, traffic):

        self.pt = pt
        self.vt = vt
        self.cp = cp

        self.graph = self.pt.get_graph()
        self.full_bitmap = (1 << self.pt.get_num_slots()) - 1

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


    def get_path_bitmap(self, path):

        bitmap = self.full_bitmap

        for i in range(len(path) - 1):
            edge_bitmap = self.spectrum_to_bitmap(
                self.pt.get_spectrum(path[i], path[i + 1])
            )
            bitmap &= edge_bitmap

        return bitmap


    def bfs_incremental(
            self,
            src,
            dst,
            demand,
            banned_links=None,
            forbidden_slots_bitmap=None
    ):

        if banned_links is None:
            banned_links = set()

        queue = deque([(src, self.full_bitmap, [src])])
        visited = set()

        edge_bitmap_cache = {}
        link_id_cache = {}

        def get_edge_bitmap(u, v):
            key = (u, v)
            if key not in edge_bitmap_cache:
                edge_bitmap_cache[key] = self.spectrum_to_bitmap(
                    self.pt.get_spectrum(u, v)
                )
            return edge_bitmap_cache[key]

        def get_link_id(u, v):
            key = (u, v)
            if key not in link_id_cache:
                link_id_cache[key] = self.pt.get_link_id(u, v)
            return link_id_cache[key]

        while queue:

            node, bitmap, path = queue.popleft()

            if node == dst:

                candidate = bitmap
                if forbidden_slots_bitmap is not None:
                    candidate &= ~forbidden_slots_bitmap

                if self.has_contiguous(candidate, demand):
                    return path, candidate

            for neigh in self.graph.neighbors(node):
                if neigh in path:
                    continue

                link_id = get_link_id(node, neigh)
                if link_id in banned_links:
                    continue

                edge_bitmap = get_edge_bitmap(node, neigh)
                new_bitmap = bitmap & edge_bitmap

                if new_bitmap == 0:
                    continue

                candidate = new_bitmap

                if forbidden_slots_bitmap is not None:
                    candidate &= ~forbidden_slots_bitmap

                if candidate == 0:
                    continue

                state_key = (neigh, candidate)   # candidate = new_bitmap sau khi áp forbidden
                if state_key in visited:
                    continue

                visited.add(state_key)
                queue.append((neigh, new_bitmap, path + [neigh]))


        return None, None


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

    def get_two_edge_disjoint_paths(self, src, dst):

        G = self.graph

        queue = deque([src])
        parent = {src: None}

        while queue:
            u = queue.popleft()
            if u == dst:
                break
            for v in G.adj[u]:
                if v not in parent:
                    parent[v] = u
                    queue.append(v)

        if dst not in parent:
            return None, None

        path1 = []
        cur = dst
        while cur is not None:
            path1.append(cur)
            cur = parent[cur]
        path1.reverse()

        forbidden_edges = set()
        for i in range(len(path1) - 1):
            forbidden_edges.add((path1[i], path1[i + 1]))
            forbidden_edges.add((path1[i + 1], path1[i]))

        queue = deque([src])
        parent2 = {src: None}

        while queue:
            u = queue.popleft()
            if u == dst:
                break
            for v in G.adj[u]:
                if v in parent2:
                    continue
                if (u, v) in forbidden_edges:
                    continue
                parent2[v] = u
                queue.append(v)

        if dst not in parent2:
            return path1, None

        path2 = []
        cur = dst
        while cur is not None:
            path2.append(cur)
            cur = parent2[cur]
        path2.reverse()

        return path1, path2

    def build_pcycle(self, flow, wp_path, wp_links, wp_slots):

        demand = len(wp_slots)
        src = flow.get_source()
        dst = flow.get_destination()

        path1, path2 = self.get_two_edge_disjoint_paths(src, dst)

        if not path1 or not path2:
            return None, None, None

        bitmap1 = self.get_path_bitmap(path1)
        bitmap2 = self.get_path_bitmap(path2)
        common_bitmap = bitmap1 & bitmap2

        if not self.has_contiguous(common_bitmap, demand):
            return None, None, None

        pcycle_slots = self.bitmap_to_slots(common_bitmap, demand)
        if pcycle_slots is None:
            return None, None, None

        bp_links = [
            self.pt.get_link_id(path2[i], path2[i + 1])
            for i in range(len(path2) - 1)
        ]

        cycle_nodes = path1 + list(reversed(path2[:-1]))

        cycle_links = [
            self.pt.get_link_id(cycle_nodes[i], cycle_nodes[i + 1])
            for i in range(len(cycle_nodes) - 1)
        ]

        last_node = cycle_nodes[-1]
        first_node = cycle_nodes[0]

        if last_node != first_node and self.graph.has_edge(last_node, first_node):
            cycle_links.append(self.pt.get_link_id(last_node, first_node))

        pcycle = PCycle(
            cycle_links,
            cycle_nodes,
            pcycle_slots,
            len(pcycle_slots)
        )

        return pcycle, bp_links, pcycle_slots


    def try_reuse_pcycle(self, wp_links, wp_slots):

        new_wp_set = set(wp_links)
        demand = len(wp_slots)

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

            def classify_link(link_id):
                u = self.pt.get_src_link(link_id)
                v = self.pt.get_dst_link(link_id)

                if link_id in cycle_links:
                    return "ON"
                elif u in cycle_nodes and v in cycle_nodes:
                    return "STRADDLE"
                else:
                    return "INVALID"

            link_types = [classify_link(e) for e in wp_links]

            if any(t == "INVALID" for t in link_types):
                continue

            if pcycle.get_reserved_slots() < demand:
                continue
            return pcycle

        return None

    def flow_arrival(self, flow: Flow):

        demand = math.ceil(flow.get_rate() / self.pt.get_slot_capacity())

        wp_path, wp_links, wp_slots = self.find_working_path(flow, demand)

        if not wp_links:
            self.cp.block_flow(flow.get_id())
            return

        # ===== Try reuse p-cycle =====
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

            self.cp.accept_flow(flow.get_id(), self.vt.get_light_path(lp_id), False)
            return

        self.new_pcycles += 1

        pcycle, bp_links, bp_slots = self.build_pcycle(flow, wp_path, wp_links, wp_slots)

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

        self.cp.accept_flow(flow.get_id(), self.vt.get_light_path(lp_id), False)


    def flow_departure(self, flow: Flow):
        pass
