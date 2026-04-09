# Project Matsyota: Autonomous Aquatic Swarm for Fish Feed Dispersion

**Lead Architect:** Uday Gopan, PES University  
**Domain:** Multi-Agent Coverage Path Planning (mCPP), Distributed Systems, Autonomous Surface Vessels (ASVs).

## 1. System Architecture

Project Matsyota utilizes a **Hybrid SCADA (Supervisory Control and Data Acquisition) Architecture** to execute distributed aquatic area coverage. The system is designed for **High Resilience and Low Computational Overhead** by decoupling global strategic planning from local tactical execution.

### Architectural Pillars:
1. **Centralized Strategic Allocation:** A Commander node calculates Voronoi partitions to establish collision-free operational boundaries.
2. **Decentralized Deterministic Execution:** Edge nodes (ASVs) generate localized Boustrophedon (lawnmower) sweeps. Since the algorithm is deterministic, path continuity is guaranteed across agent handovers.
3. **Legacy Task Inheritance:** Instead of fluidly recalculating the entire swarm lattice upon failure, the system uses a **Neighbor-Steal/Debt Registry**. Fallen agents' polygons are queued as high-priority "Legacy Tasks" for the first available healthy agent.
4. **Shapely-Bounded Geometry:** All Voronoi cells are mathematically intersected with the farm perimeter to solve the infinite-bounds problem and ensure strict 100x100m containment.

---

## 2. Workspace Topology

### `matsyota_sim` (C++ / XML)
* `empty_water.world`: Static, zero-collision aquatic environment. Visual buoys define the 100x100m farm.
* `asv.urdf.xacro`: Namespaced ASV chassis with GPS and Planar Move plugins.
* `swarm.launch.py`: Dynamic spawner with anti-collision coordinate generation.

### `matsyota_core` (Python)
* `matsyota_commander.py`: The Fleet Commander. Manages the **Legacy Task Queue**, monitors agent heartbeats (GPS-based), and performs Shapely-based polygon clipping.
* `feed_dispersion_mapper.py`: Visualizes coverage via a 0.25m-resolution `nav_msgs/OccupancyGrid`.

### `matsyota_agent` (Python)
* `asv_driver.py`: The Tactical Edge Node. Performs local Boustrophedon path generation, executes PD control, and performs the **"COMPLETE" Handshake** to trigger legacy task reassignment.

---

## 3. DDS Data Matrix & QoS Profiles

| Topic / Data Stream | Message Type | QoS Profile | Frequency |
| :--- | :--- | :--- | :--- |
| **`/asv_X/gps/fix`** | `NavSatFix` | **BEST_EFFORT** | 10Hz |
| **`/asv_X/assigned_territory`**| `PolygonStamped` | **RELIABLE** | 1Hz |
| **`/asv_X/task_status`** | `String` | **RELIABLE** | Event-based |
| **`/asv_X/odom`** | `Odometry` | **BEST_EFFORT** | 50Hz |
| **`/asv_X/cmd_vel`** | `Twist` | **BEST_EFFORT** | 50Hz |

---

## 4. Fault Tolerance: The Inheritance Model

Matsyota bypasses complex consensus graphs by using a **Centralized Debt Registry**:
* **Detection:** Commander flags a timeout if GPS telemetry ceases for > 2.0s.
* **Archiving:** The failed agent's active polygon and its last known coordinate are pushed to a `legacy_tasks` queue.
* **Inheritance:** When a healthy agent finishes its primary sector, it queries the queue. It "inherits" the dead agent's task, driving to the failure point to resume the deterministic sweep.

---

## 5. Development Roadmap

**Sprint 1: Baseline Architecture (Completed)**
- [x] Workspace generation and Git/Build firewalling.
- [x] Entity definition and dynamic swarm launching.
- [x] Commander Voronoi logic and Watchdog timer.

**Sprint 2: The Geometric & Resilience Payload (Completed)**
- [x] Inject `shapely` for Infinite Bound Clipping.
- [x] Implement Deterministic Boustrophedon sweep logic in `asv_driver.py`.
- [x] Establish the "Task Complete" Handshake and Legacy Task Queue.

**Sprint 3: Orchestration & Validation (Upcoming)**
- [ ] Construct the Master Orchestration Launch File.
- [ ] Perform Stress Tests (simulated agent failure mid-sweep).
- [ ] Tune PD Gains for high-speed aquatic maneuvers.