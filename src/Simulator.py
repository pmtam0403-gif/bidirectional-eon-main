import xml.etree.ElementTree as ET
import time

from src.PhysicalTopology import PhysicalTopology
from src.VirtualTopology import VirtualTopology
from src.TrafficGenerator import TrafficGenerator
from src.EventScheduler import EventScheduler
from src.MyStatistics import MyStatistics
from src.OutputManager import OutputManager
from src.Tracer import Tracer
from src.ControlPlane import ControlPlane
from src.SimulationRunner import SimulationRunner


class Simulator:
    sim_name = "flexgridsim"
    sim_version = "2.0"
    verbose = False
    trace = False

    def __init__(self, sim_config_file: str, trace: bool, verbose: bool, forced_load: float, num_simulations: int):
        Simulator.trace = trace
        Simulator.verbose = verbose

        # === Thêm biến đếm ===
        self.total_requests = 0
        self.accepted_requests = 0
        self.acceptance_rate = 0

        if Simulator.verbose:
            print("#################################")
            print("# Simulator: " + Simulator.sim_name + " version " + Simulator.sim_version + " #")
            print("#################################")
            print("(0) Accessing simulation file " + sim_config_file + "...")

        with open(sim_config_file, 'r') as f:
            root = ET.parse(f).getroot()
            assert root.tag == Simulator.sim_name, "Root element mismatch!"
            assert "version" in root.attrib.keys(), "Missing version attribute!"
            assert root.attrib["version"] <= Simulator.sim_version, "Config file requires newer simulator!"

            for child in root:
                if child.tag == "rsa":
                    self.rsa = child
                elif child.tag == "trace":
                    self.trace = child.attrib
                elif child.tag == "traffic":
                    self.traffic = child
                elif child.tag == "virtual-topology":
                    self.virtual_topology = child
                elif child.tag == "physical-topology":
                    self.physical_topology = child
                elif child.tag == "graphs":
                    self.graphs = child
                else:
                    assert False, "Unknown element " + child.tag

            gp = OutputManager(self.graphs)

            for seed in range(1, num_simulations + 1):
                begin_s = time.time_ns()

                # (1) Load physical topology
                pt = PhysicalTopology(self.physical_topology, verbose)

                # (2) Load virtual topology
                vt = VirtualTopology(self.virtual_topology, pt, verbose)

                # (3) Load traffic
                events = EventScheduler()
                traffic = TrafficGenerator(self.traffic, forced_load, verbose)
                traffic.generate_traffic(pt, events, seed)

                # (4) Setup statistics
                st = MyStatistics.get_my_statistics()
                st.statistics_setup(gp, pt, traffic, pt.get_num_nodes(), 3, 0, forced_load, Simulator.verbose)

                tr = Tracer.get_tracer_object()
                tr.toogle_trace_writing(Simulator.trace)

                rsa_module = self.rsa.attrib["module"]

                # (5) Create ControlPlane
                cp = ControlPlane(self.rsa, events, rsa_module, pt, vt, traffic)

                # === Truyền simulator vào ControlPlane để cập nhật biến đếm ===
                cp.simulator = self

                # (6) Run simulation
                print(f"{sim_config_file} -> Load {forced_load}: Running simulation {seed}")
                SimulationRunner(cp, events)

                # (7) Tính acceptance rate
                if self.total_requests > 0:
                    self.acceptance_rate = self.accepted_requests / self.total_requests
                else:
                    self.acceptance_rate = 0

                print(f"Acceptance rate = {self.acceptance_rate:.4f}")

                # (8) Final statistics
                st.calculate_last_statistics()
                st.finish()

                if Simulator.trace:
                    tr.finish()

            gp.write_all_to_files()