# Project Matsyota: Autonomous Aquatic Swarm for Fish Feed Dispersion

**Lead Architect:** Uday Gopan, PES University  
**Domain:** Multi-Agent Coverage Path Planning (mCPP), Distributed Systems, Autonomous Surface Vessels (ASVs).

## 1. System Architecture

Project Matsyota utilizes a **Hybrid SCADA (Supervisory Control and Data Acquisition) Architecture** to execute distributed aquatic area coverage while minimizing DDS network bloat and simulation physics friction.

Instead of relying on fragile SLAM algorithms or closed-kinematic collision meshes, this system leverages pure absolute positioning (GPS) and deterministic geometry to guarantee robust performance and a 1.0 Real-Time Factor (RTF) in Gazebo.

### Architectural Pillars:
1. **Centralized Strategic Allocation:** A Commander node calculates dynamic Voronoi partitions to divide the operational area without collision domains.
2. **Decentralized Tactical Execution:** Edge nodes (ASVs) receive polygon boundaries and locally calculate their own Boustrophedon (lawnmower) sweep paths.
3. **Decoupled Physics & Visualization:** Fish feed is not simulated as physical particles to prevent physics engine crashes. Dispersion is mathematically proven via a global Occupancy Grid in RViz2.

---

## 2. Workspace Topology

The codebase is strictly decoupled by function to prevent dependency compilation errors.

### `matsyota_sim` (C++ / XML)
The Physical Layer. Handles rendering, physics solvers, and entity spawning.
* `empty_water.world`: A static, zero-collision water plane. The 100x100m farm boundaries are marked purely visually using red buoys to prevent physics rigid-body failures.
* `asv.urdf.xacro`: The ASV chassis. Dynamically namespaced to isolate TF trees. Equipped with `libgazebo_ros_planar_move.so` and `libgazebo_ros_gps_sensor.so`.
* `swarm.launch.py`: Dynamic spawner utilizing minimum-distance checks to prevent spawn-collisions.

### `matsyota_core` (Python)
The Global Brain and Visualization Layer.
* `matsyota_commander.py`: The Fleet Commander. Ingests all ASV GPS data, monitors fleet health (implicit watchdog), computes `scipy.spatial.Voronoi` lattices, and dispatches `PolygonStamped` territories.
* `feed_dispersion_mapper.py`: The Grading Node. Maps absolute ASV coordinates onto a 0.25m-resolution `nav_msgs/OccupancyGrid` to visually prove 100% surface area coverage.

### `matsyota_agent` (Python)
The Edge Muscle. One instance runs per ASV.
* `asv_driver.py`: Listens for territory assignments. Locally calculates parallel Boustrophedon sweep lines, tracks local `/odom` via a Proportional-Derivative (PD) controller, and publishes its intent to RViz2 via `nav_msgs/Path`.

---

## 3. DDS Data Matrix & QoS Profiles

To protect the ROS 2 DDS middleware from packet saturation, asymmetric Quality of Service (QoS) profiles are enforced.

| Topic / Data Stream | Publisher | Subscriber | QoS Profile | Frequency |
| :--- | :--- | :--- | :--- | :--- |
| **`/asv_X/gps/fix`** | Gazebo | `matsyota_commander` | **BEST_EFFORT** | 10Hz |
| **`/asv_X/assigned_territory`**| `matsyota_commander` | `asv_driver` | **RELIABLE** | 1Hz |
| **`/asv_X/odom`** | Gazebo | `asv_driver` | **BEST_EFFORT** | 50Hz |
| **`/asv_X/local_plan`** | `asv_driver` | RViz2 | **RELIABLE** | On Update |
| **`/asv_X/cmd_vel`** | `asv_driver` | Gazebo | **BEST_EFFORT** | 50Hz |

---

## 4. Fault Tolerance (Implicit Watchdog)

Project Matsyota natively supports fleet resilience without complex Raft consensus graphs. 
* The Commander node monitors the timestamp of incoming GPS packets. 
* If an ASV drops off the network (timeout > 2.0s), the Commander flags it as dead.
* The Voronoi spatial partitioner recalculates the lattice using only the surviving agents. 
* The surviving ASVs are immediately dispatched expanded boundary polygons to absorb the dead agent's territory, ensuring 100% farm coverage.

---

## 5. Development Roadmap

**Sprint 1: Baseline Architecture (Completed)**
- [x] Workspace generation and `.gitignore` firewalling.
- [x] Entity definition (URDF/SDF) and dynamic swarm launching.
- [x] Commander Voronoi logic skeleton and Watchdog timer.
- [x] Agent PD controller and RViz2 pathing protocol.
- [x] OccupancyGrid visualization logic.

**Sprint 2: The Geometric Payload (Current)**
- [ ] Inject `shapely` into `matsyota_commander.py` to clip infinite Voronoi regions to the 100x100m farm boundary.
- [ ] Build the localized `generate_boustrophedon_path()` logic inside `asv_driver.py`.
- [ ] Construct the Master Orchestration Launch File.