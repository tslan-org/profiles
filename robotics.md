# Robotics Profile

A profile for on-body robotics networks built on IEEE 802.1AS, IEEE 1722, IEEE 1722.1, and selected IEEE 802.1 TSN mechanisms from IEC/IEEE 60802-2026 and IEEE 802.1DG-2025: synchronized multi-joint actuation, coherent multimodal perception, bounded AVB media coexistence, and end-to-end black-channel integrity.

Version: 1.0-draft

---

## 1. Scope

On-body robots carry two workloads on the same physical harness: high-rate closed-loop actuation (typically 1 to 4 kHz) with tight round-trip latency bounds, and multimodal perception (cameras, LiDAR, IMUs, force/torque sensors, tactile arrays, microphones, and speakers) that needs one shared hardware timebase. This profile defines how bridges, controllers, and endpoints share one Ethernet fabric while preserving bounded latency and bandwidth for standard AVB media streams on shared trunks.

This profile specifies:

- **Time synchronization:** a dedicated Working Clock domain with fixed port roles, fast-lock requirements, and holdover bounds, separated from Universal Time (§2).
- **Traffic classes, VLANs, and shaping:** an eight-priority mapping with SR Class C and a shaped safety priority above IEEE 802.1BA SR Classes A and B, for ports with eight or four queues, plus worst-case latency budgets for robotics and AVB media (§3).
- **Layer-2 transport framing:** packed cyclic command, joint state, and safety heartbeat payloads carried inside IEEE 1722 TSCF or NTSCF `ACF_GPC` frames (§4).
- **End-to-end black-channel integrity:** Castagnoli CRC-32C protecting `data_id`, `gpc_msg_id`, `avtp_timestamp`, and payload, with sequence/epoch restart rules, deadline verification, and a deterministic actuator safety state machine (§5).
- **Discovery and startup:** static fast-boot bindings for core body streams alongside IEEE 1722.1 ATDECC for enumeration, configuration, and hot-swappable tools (§6).

Control laws, kinematics, perception algorithms, and system-level functional safety certification are out of scope.

### 1.1 Roles

- **Bridge:** an IEEE 802.1Q / TSN bridge (standalone switch, zonal switch, or Linux TSN bridge) that forwards robotics, AVB, and best-effort frames, relays IEEE 802.1AS time, and enforces traffic shaping and ingress policing.
- **Controller:** a compute node that receives cyclic joint state and sensor streams, computes motion targets, and transmits cyclic command streams paced on Working Clock time.
- **Safety supervisor:** a logical role (hosted on a dedicated safety MCU or on the controller) that monitors system health and emits periodic `SafetyHeartbeat` frames authorizing torque enablement per logical safety group.
- **Actuator or zonal endpoint:** a single-joint motor controller, sensor node, or zonal gateway that receives command and heartbeat frames, enforces the safety state machine, latches physical outputs at Working Clock deadlines, and emits telemetry frames.

### 1.2 Conformance Tiers

Implementations declare one of two bridge/endpoint conformance tiers:

| Tier | Typical Hardware | Required Mechanisms | Worst-Case Limb Latency Constraint |
|------|------------------|---------------------|------------------------------------|
| **Tier 1 (Baseline)** | Gigabit trunks, Linux `igc` or embedded prototypes, bounded-MTU 100 Mb/s limb links | IEEE 802.1AS, 802.1Q VLAN/PCP, CBS (802.1Qav), PSFP (802.1Qci) on bridges, §5 E2E integrity | Must satisfy the store-and-forward cycle budget of §3.2 |
| **Tier 2 (Hard-Cyclic)** | Production TSN switches and limb daisy-chains (100BASE-T1, 1000BASE-T1) | All Tier 1 plus TAS (802.1Qbv) and Frame Preemption (802.1Qbu / 802.3br); optional FRER (802.1CB) on torso rings and draft P802.1ASds on 10BASE-T1S hands | Sub-cycle deterministic delivery across multi-hop limb chains |

### 1.3 Physical Zones and Logical Safety Groups

Matching automotive zonal architectures (IEEE 802.1DG-2025), this profile separates **physical hardware zones** (Zonal Control Units or switch boards placed at structural boundaries) from **logical safety and stream groups** (software and protocol isolation domains).

The number and placement of physical zones, as well as the assignment of logical `group_id` values (0 through 15), are left to the implementer so the profile applies equally to humanoids, quadrupeds, mobile manipulators, and single-arm robots.

- **Physical zones** are a hardware packaging and cabling choice; the wire protocol does not mandate a fixed zone count.
- **Logical safety groups (`group_id`, 0 through 15)** define independent stream IDs, implicit E2E `data_id` bindings, PSFP ingress filters, and enable bits in `active_group_mask` at zero hardware cost. Safety coupling policy (which groups stop together on a fault) is configured in the safety supervisor rather than hardwired to physical switch boxes.

**Recommended humanoid layout (informative).** For a bipedal humanoid, three physical zones placed at the neck and waist boundaries minimize trunk cabling while avoiding per-limb switch PCBs: **ZCU-Head** across the neck (one 1000BASE-T1 link plus optional PoDL for cameras, LiDAR, head IMU, AAF microphones/speakers, and neck actuators, staying powered during motor STO), **ZCU-Upper** in the torso (central compute, main TSN switch, Working Clock grandmaster, BMS, and direct shoulder ports for both arms, with hands hanging off the wrists over 10BASE-T1S where needed), and **ZCU-Lower** in the pelvis (pelvis TSN switch, pelvis IMU, locomotion STO hub, and direct hip ports for both legs, fed by one 1000BASE-T1 trunk across the waist). Within those three physical zones, an implementer may assign separate logical groups to torso/waist, head/neck, left arm, right arm, left leg, right leg, left hand, and right hand, coupling torso and legs as one locomotion safety group while letting either arm or hand enter damping and Safe Torque Off independently.

### 1.4 Relation to Other Profiles and Milan

A robot often carries microphones, speakers, and cameras alongside its control loops. This profile allows endpoints conforming to the **Basic Audio Profile**, **Extended Audio Profile**, **Basic Video Profile**, **Extended Video Profile**, **Voice Assistant Profile**, or **Milan** to operate on shared 1 Gb/s robot trunks (typically 1 to 3 hops) without modification, provided the bridge satisfies the coexistence rules of §2.1 and §3:

- **Clock domain:** IEEE 802.1AS Domain 0 remains active with BTCA whenever any AVB media endpoint is present (§2.1).
- **Priorities and VLAN:** Priorities 3 and 2 remain IEEE 802.1BA SR Class A and SR Class B in VLAN 2; robotics sensor streams that do not feed a control loop may use them, with their own reservations (§3.1).
- **Bounded interference:** Priority 7 and SR Class C, the only robotics traffic above SR Classes A and B, are shaped (CBS, or bounded TAS) and ingress-policed, keeping total reserved plus gated bandwidth within 75% of port rate and keeping SR Class A latency within Milan's 2 ms budget across the robot's trunk hop count (§3.2). Priorities 5 and 4 sit below SR Classes A and B. AVB media streams are not expected to traverse narrow 100 Mb/s limb daisy-chains.
- **Robotics TSCF Streams Profile:** this profile's `JointCommandArray` and `JointStateArray` streams are control streams (`ACF_GPC`, `gpc_msg_id = 8C:1F:64:36:C0:08`) of the Robotics TSCF Streams Profile (`robotics_tscf.md`); IMUs, force/torque sensors, and tactile maps may use its `ACF_SENSOR` sensor and event streams and its SR classes (§3.1).
- **Wireless and Lite profiles:** Robotics endpoints bridging over Wi-Fi (such as an untethered operator console or auxiliary wireless sensor) follow the **AVB Wireless Profile** on the wireless hop. Auxiliary AVB media endpoints without a full TSN bridge may use the **AVB Lite Profile**, while closed-loop actuation remains on wired TSN links meeting Tier 1 or Tier 2.
- **DDS-TSN:** Without reservations, DDS traffic (such as ROS 2 over DDS) runs at priority 0 (§3.1). DDS applications that need reserved delivery between compute nodes may describe their streams with OMG DDS Extensions for Time Sensitive Networking (DDS-TSN) 1.0 and reserve them in SR Class A or B, through MSRP (the fully distributed model of IEEE 802.1Q-2022 Clause 46) or static bridge configuration, within the 75% port budget (§3.2). The DDS-TSN traffic specification gives the MSRP TSpec: `max_bytes_per_sample`, plus the IP and UDP headers over UDP/IP, sets `MaxFrameSize`, and `MaxIntervalFrames` counts the most frames, one per fragment, sent in any one class measurement interval. The talker tags each frame with the stream's VLAN and priority and sends it to the stream's group destination MAC address (§6.1), directly over Ethernet (DDS-TSN Annex A) or over UDP/IP with the end station transforming the stream (DDS-TSN §8.3.2), since this profile does not require bridges to identify streams by IP and UDP header fields. Like perception streams, these streams stay off limb ports (§3.3). SR Class C and priority 7 remain for the streams of this profile and the Robotics TSCF Streams Profile, and DDS-TSN defines no end-to-end integrity, so actuator commands and safety heartbeats stay on the frames of §4 and §5 (informative).

### 1.5 Draft Notes (informative)

Points open in this draft:

- Clause citations to IEEE 802.1AS were verified against IEEE 802.1AS-2020; clause numbering should be re-checked against IEEE 802.1AS-2025 once that document is on hand.
- IEEE P802.1ASds (half-duplex / 10BASE-T1S timing) is in Standards Association ballot (draft amendment) and is referenced as optional forward guidance only.
- IEEE 1722.1 AEM vendor control types for robotics-specific parameters (homing, calibration tables, per-joint gain and damping limits) will be allocated from the MA-S control-type pool in a later revision.

---

## 2. Time Synchronization

### 2.1 Working Clock and Universal Time Domains

Following IEC/IEEE 60802-2026, internal motion control is decoupled from external wall-clock time using IEEE 802.1AS multi-domain synchronization:

- **Domain 1 (Working Clock, required for closed-loop actuation):** runs on an arbitrary monotonic timescale (`ARB`) sourced from an internal grandmaster (typically the torso compute or torso TSN bridge). The Working Clock shall never step or slew aggressively while any actuator is armed or active. It drives motion-cycle grids, actuation execution timestamps, observation timestamps, and 802.1Qbv Gate Control Lists.
- **Domain 0 (Universal Time / AVB Media Clock):** runs on the PTP/TAI timescale with BTCA enabled (IEEE 802.1AS-2020 §5.4.1 f) 6)). Per IEEE 802.1AS-2020 §7.2.3, every time-aware system supports Domain 0.
  - Whenever any AVB or Milan media endpoint (Basic Audio, Basic Video, Voice Assistant, or Milan) is present on the robot, **Domain 0 shall be active** with BTCA enabled so those endpoints synchronize normally.
  - External steps, slews, or grandmaster changes on Domain 0 must not perturb Domain 1.
  - Only on a standalone robot with no AVB media endpoints and no external fleet synchronization may Domain 0 be left unused while Domain 1 runs alone.

Endpoints whose PTP hardware clock runs free expose Working Clock time as a linear mapping (local clock to Working Clock nanoseconds, rate ratio, and discontinuity generation), avoiding hardware clock steps or slews (informative).

### 2.2 External Port Configuration and Fast Lock

In IEEE 802.1AS-2020 §10.3.1, `externalPortConfigurationEnabled` is a per-PTP-Instance (per-domain) setting, not a per-link setting.

- Every PTP Instance participating in **Domain 1 (Working Clock)** shall set `externalPortConfigurationEnabled` to `TRUE` and statically configure each internal port's `desiredState` as `TimeTransmitterPort` (facing downstream toward limbs and perception pods) or `TimeReceiverPort` (facing upstream toward the internal grandmaster).
- In accordance with IEEE 802.1AS-2020 §10.3.14 (`PortAnnounceInformationExt`), Announce messages continue to be transmitted on Domain 1 at a slow interval (default `initialLogAnnounceInterval = 0`, 1 s) to convey grandmaster clock properties. Because port roles are fixed, slow Announce intervals do not delay synchronization lock. (An automotive-style variant that suppresses Announce altogether on internal links is a departure from base IEEE 802.1AS and must be configured identically on every node in Domain 1.)
- External service, tether, or docking ports shall not join Domain 1 as `TimeReceiverPort`; dynamic BTCA is permitted only on Domain 0.
- **Fast cold-boot lock (< 50 ms from link-up):**
  - IEEE 802.1AS-2020 defaults `initialLogSyncInterval` to `-3` (125 ms) and `initialLogPdelayReqInterval` to `0` (1 s), and allows ports to ignore unsupported intervals. Therefore, all bridges and endpoints conforming to this profile shall support `initialLogPdelayReqInterval` of **`-5` (31.25 ms)** and `initialLogSyncInterval` values of **`-7` (7.8125 ms)** and **`-6` (15.625 ms)** on internal links.
  - Configuring `initialLogPdelayReqInterval = -5` allows the `MDPdelayReq` state machine (IEEE 802.1AS-2020 §11.2.19) to complete two Pdelay exchanges in about 31 ms, establishing `meanLinkDelay`, `neighborRateRatioValid = TRUE`, and `asCapableAcrossDomains = TRUE` strictly within base IEEE 802.1AS-2020. With Sync arriving every 7.8125 ms (`-7`) immediately afterward, full lock completes within 50 ms of link-up.
  - Once initial Pdelay convergence finishes on a fixed-length harness, a port may slow its steady-state Pdelay interval via signaling or freeze further delay recomputation using `useMgtSettableComputeMeanLinkDelay = TRUE` and `mgtSettableComputeMeanLinkDelay = FALSE` (IEEE 802.1AS-2020 §11.2.21, controlling `computeMeanLinkDelay` in §10.2.5.11). Because `meanLinkDelay` is read-only in IEEE 802.1AS-2020 §14.8.8, pre-loading a calibrated delay without measuring it requires an implementation-specific mechanism outside IEEE 802.1AS (informative).

### 2.3 Clock Integrity, Holdover, and Redundancy

- Every actuator and zonal endpoint shall sanity-check incoming Working Clock Sync updates against its local oscillator before updating its time mapping (defaults: reject single-Sync outliers exceeding 5 µs and rate ratios outside ±200 ppm).
- If valid Working Clock Sync updates cease, an endpoint may continue running in **Clock Holdover** on its local crystal for at most `max_clock_holdover_ns` (default 50,000,000 ns, 50 ms, bounding drift to at most 2.5 µs for a 50 ppm crystal). While in Clock Holdover, the node shall set the `CLOCK_HOLDOVER` fault flag. If sync does not recover before `max_clock_holdover_ns` expires, the node shall set `CLOCK_LOST` and transition out of `Active` or `CommandCoast` into `Damping` (or `SafeTorqueOff` if `Armed`).
- Robots with dual compute modules or a redundant torso ring should run IEEE 802.1AS-2025 Hot Standby (Domain 2 tracking Domain 1 with fixed reverse paths) for zero-step failover, and may apply IEEE 802.1ASed fault-tolerant time selection across redundant domains.
- Segments built on 10BASE-T1S multidrop Ethernet (such as dexterous hands or tactile strips) may synchronize over half-duplex links using draft amendment **IEEE P802.1ASds** once finalized (informative).

---

## 3. Traffic Classes, VLANs, and Shaping

### 3.1 Priority and VLAN Allocation

IEEE 802.1Q-2022 Table 34-1 and Table 9-2 default SR Class A to priority 3, SR Class B to priority 2, and the SR VLAN to VID 2. This profile preserves those assignments so AVB audio and video profiles work without modification, and adds SR Class C of the Robotics TSCF Streams Profile (its §5.1) for robotics streams:

- **SR Class C (priority 6 by default):** control streams (`JointCommandArray`, `JointStateArray`), event streams (contact, slip, impact, and joint fault alerts), and sensor streams that feed a control loop.
- **SR Class B (priority 2), or SR Class A (priority 3) where a stream needs its lower latency:** other sensor streams, alongside AVB media.
- **Priority 7:** `SafetyHeartbeat` and E-Stop / STO frames, shaped and policed but not an SR class.
- **Priorities 5 and 4:** no robotics streams. They stay unshaped, below SR Classes A and B. SR Classes D and E, which would shape event and sensor streams on them, are deferred, as in the Robotics TSCF Streams Profile.

Where bridges run MSRP, robotics streams share the SR VLAN (`SRclassVID`, VID 2 by default). Where core robotics streams are statically configured without MSRP, bridges may isolate robotics control traffic on a separate **Robotics Control VLAN** (VID 3 by default, configurable per platform), on the same priorities.

Per IEEE 802.1Q-2022 §34.5 b) and Annex L.1 NOTE 2, placing an unshaped strict-priority class above a CBS class makes the lower CBS class's `maxInterferenceSize` infinite and breaks its bandwidth guarantee. Therefore, **no unshaped strict-priority queue is permitted above priorities 2 and 3** on any port carrying AVB SR Class A or Class B streams; priority 7 (Safety) is shaped with CBS and ingress-policed via PSFP on every port including the safety supervisor's own port.

Bridges shall configure their Traffic Class Table (IEEE 802.1Q-2022 §12.6.3) and Transmission Selection Algorithm Table (§12.20.2) so priority 7 and SR Class C have shaped queues above those of SR Classes A and B, and priorities 5 and 4 keep the unshaped queues Table 34-1 gives them, below SR Classes A and B:

| Priority | Traffic Class | Queue / Shaper Mechanism | Tier 2 Preemption | Workload |
|----------|---------------|--------------------------|-------------------|----------|
| **7** | Safety | CBS (static small idle slope) + PSFP | Express | `SafetyHeartbeat`, E-Stop / STO control frames (bounded burst) |
| **6** | SR Class C | CBS (802.1Qav) + PSFP, with TAS in Tier 2 | Express | Cyclic actuator command and telemetry streams (1 to 4 kHz), contact, slip, and joint fault events, tactile events, and IMU and force/torque streams that feed a control loop |
| **3** | SR Class A | CBS (802.1Qav) | Preemptable | IEEE 802.1BA / Milan SR Class A media streams, and sensor streams that need Class A's latency |
| **2** | SR Class B | CBS (802.1Qav) | Preemptable | Basic Audio (AAF), Basic Video (CVF/RVF), Voice Assistant streams, and other sensor streams |
| **5** | Unshaped | Strict, below SR Classes A and B | Preemptable | No robotics streams (SR Class D deferred) |
| **4** | Unshaped | Strict, below SR Classes A and B | Preemptable | No robotics streams (SR Class E deferred) |
| **0** | Best effort | Best effort (strict below shaped classes) | Preemptable | IEEE 1722.1 ATDECC (ADP, AECP, ACMP, `MEMORY_OBJECT` firmware), ROS 2 / Zenoh control, diagnostics |
| **1** | Background | Background (strict below shaped classes) | Preemptable | Bulk telemetry logging, non-ATDECC bulk file transfers |

The queues, from the highest down, are those of the Robotics TSCF Streams Profile (its §5.1):

| Queue | Port with eight queues | Port with four queues |
|-------|------------------------|-----------------------|
| Highest | Priority 7 | Priority 7 and SR Class C (priority 6) |
| Second | SR Class C (priority 6) | SR Class A (priority 3) |
| Third | SR Class A (priority 3) | SR Class B (priority 2) |
| Fourth | SR Class B (priority 2) | Priorities 0, 1, 4, and 5 |
| Fifth | Priority 5 | |
| Sixth | Priority 4 | |
| Seventh | Priority 0 | |
| Lowest | Priority 1 | |

**Four queues.** A port with four queues, such as one on a network controller with four transmit queues, puts priority 7 in SR Class C's queue, policed on ingress (§3.3) and counted in Class C's idle slope; a port with more than four queues keeps priorities 7 and 6 apart. On a four-queue port, priority 7 frames wait behind the Class C frames already queued, about 20 µs for each 7-slot limb command at 100 Mb/s (§3.2), and, in Tier 2 where Class C's gate opens only for its window, up to one `T_cycle`. Both stay well within `heartbeat_timeout_ns` (50 ms by default, §5.3) (informative).

**gPTP frames are untagged.** Per IEEE 802.1AS-2020 §11.3.3, frames carrying IEEE 802.1AS messages shall not carry a VLAN tag or a priority tag. They are link-local (destination MAC `01:80:C2:00:00:0E`, EtherType `0x88F7`) and therefore do not appear in the VLAN priority table above; bridges map untagged gPTP frames internally to their highest transmit priority (express queue in Tier 2) and exempt them from the VLAN CBS idle-slope rule (they do not consume priority 7's idle slope). A Sync and Follow_Up pair takes about 198 octets on the wire per Sync interval per domain (~0.20 Mb/s per domain at `-7`), so even with Domain 0, Domain 1, and Domain 2 (Hot Standby) active plus Pdelay exchanges, total gPTP wire load stays under 0.7 Mb/s and its interference into CBS classes is negligible (informative).

### 3.2 Bandwidth Budgets and Latency Bounds

#### Shaper Bandwidth Provisioning

- CBS idle slopes for priority 7 and for the SR Class C reservations of core body streams shall be statically pre-configured on bridges at boot (matching IEEE 802.1DG and IEC/IEEE 60802) so closed-loop control does not wait on dynamic reservations.
- SR Class A and Class B (priorities 3 and 2), including the robotics sensor streams they carry, obtain idle slopes through MSRP reservations per IEEE 802.1BA, or through static bridge configuration where MSRP is disabled.
- On any port carrying both robotics and AVB SR traffic, the sum of the idle slopes of priority 7 and SR Class C, any Tier 2 TAS exclusive-window duty cycle (`(W_ctrl + W_guard) / T_cycle`), and all SR Class A and B reservations shall not exceed **75% of port transmit rate**.

#### AVB SR Class Coexistence Latency (IEEE 802.1Q-2022 Annex L.3)

Because priority 7 and SR Class C (priority 6) sit above SR Class A (priority 3) and SR Class B (priority 2), each must have a bounded maximum burst enforced by CBS and PSFP (or a bounded TAS window `W_ctrl` in Tier 2). Priorities 5 and 4 sit below SR Classes A and B, so like priorities 0 and 1 they add at most one lower-priority frame, counted below:

- **Tier 1 (CBS on priority 7 and SR Class C) on a 1 Gb/s trunk:** Let `S_high` be the sum of the idle slopes of priority 7 and SR Class C (recommended cap `S_high <= 250 Mb/s`, 25% of 1 Gb/s) and `B_high` be the sum of their maximum policed bursts (for a 32-DOF robot: up to 2 priority-7 frames 256 B, plus in Class C 4 limb commands 1,016 B + 4 events 600 B + 4 control-loop sensors 1,120 B = 2,992 octets). Including one max lower-priority frame (`1,542` octets including preamble and IFG), per IEEE 802.1Q-2022 Annex L.3 the worst-case per-hop interference into SR Class A is:
  - `maxInterferenceSize` (lower + higher bursts) = `1,542 + 2,992 = 4,534` octets.
  - At effective remaining rate `1,000 - 250 = 750 Mb/s`, worst-case per-hop queuing delay for SR Class A is `4,534 * 8 / 750,000,000 = 48.4 µs` (plus bridge internal delay ~5 µs). Across a 3-hop robot trunk, SR Class A worst-case latency is **under 0.2 ms**, well within Milan's 2 ms Class A bound and the 50 ms Class B bound.
- **Tier 2 (TAS on SR Class C) on a 1 Gb/s trunk:** A protected Class C TAS window `W_ctrl` plus guard band `W_guard` (20 µs without preemption, about 1 µs with 802.1Qbu preemption) closes lower queues once per cycle `T_cycle`. On a 2 kHz (500 µs) cycle with `W_ctrl = 100 µs` and `W_guard = 20 µs` (24% gate closure), each hop adds at most `120 µs` plus the remaining priority 7 and Class C bursts (about 15 µs) to SR Class A latency, keeping 3-hop trunk latency **under 0.5 ms** (still well within Milan's 2 ms bound) provided `W_ctrl + W_guard` plus other shaped idle slopes stays within the 75% port budget.

#### Tier 1 Robotics Closed-Loop Latency Bound

Without TAS or Frame Preemption, a Tier 1 store-and-forward bridge hop incurs both the control frame's **own transmission time** (`T_own_tx`) and **one maximum lower-priority interfering frame** (`T_blocking_max`), plus internal switch forwarding latency (`T_sw`). Over `H` hops one-way:

```text
T_oneway_max = H * (T_own_tx + T_blocking_max + T_sw)
```

For closed-loop actuation with cycle period `T_cycle`, the round-trip network delay plus controller compute time `T_solve` and actuation safety margin `T_margin` shall satisfy:

```text
T_up_max + T_solve + T_down_max + T_margin <= T_cycle
```

Sizing each command stream to one limb's joints (for example, 5 to 8 slots rather than 60) is strongly recommended for Tier 1 limbs (informative):

- A **60-slot** command frame is 1,506 octets tagged (1,526 octets on the wire with 20-octet preamble and IFG), taking **122.08 µs per 100 Mb/s hop** (488 µs across 4 hops before any blocking), which cannot close a 2 kHz loop at 100 Mb/s.
- A **7-slot** limb command frame (212-octet AVTPDU, 234 octets tagged, 254 octets on the wire) takes **20.32 µs per 100 Mb/s hop** (2.03 µs at 1 Gb/s). A single-joint return state frame (76-octet AVTPDU, 98 octets tagged, 118 octets on the wire) takes **9.44 µs per 100 Mb/s hop** (0.94 µs at 1 Gb/s).
- A full 1,522-octet tagged interfering frame (1,542 octets on the wire) adds **123.36 µs per 100 Mb/s hop** (12.34 µs at 1 Gb/s). To keep full IEEE 1722.1 ATDECC enumeration and `MEMORY_OBJECT` firmware transfers working on 100 Mb/s limb ports (IEEE 1722.1-2021 permits AEM descriptors up to 508 octets and `control_data_length` up to 524 octets, giving a maximum tagged AECPDU of ~558 octets or **580 octets on the wire**), capping non-control frames on 100 Mb/s limb ports to **560 octets tagged (580 octets on the wire)** preserves full ATDECC compatibility while cutting worst-case blocking to **46.40 µs per hop**.

Therefore, Tier 1 networks running 2 kHz (500 µs) loops shall satisfy the round-trip inequality above by doing at least one of:

- operating limb links at **1 Gb/s** (4 hops with 7-slot commands and full 1,522-octet interfering frames: `T_down_max ≈ 4 * (2.03 + 12.34 + 3) = 69.5 µs` one-way, leaving > 300 µs for return telemetry, `T_solve`, and margin), or
- on **100 Mb/s** limb links, sizing streams per limb (<= 8 slots), capping non-control frames on limb ports to 560 octets tagged (580 octets on the wire, `T_blocking_max = 46.40 µs`), and limiting limb depth so `T_up_max + T_solve + T_down_max + T_margin <= T_cycle` (for example, 2 hops: `T_down_max ≈ 2 * (20.32 + 46.40 + 5) = 143.4 µs`, `T_up_max ≈ 2 * (9.44 + 46.40 + 5) + 6 * 9.44 = 178.3 µs`, leaving ~178 µs for `T_solve + T_margin`).

### 3.3 Ingress Policing and Fault Containment (PSFP)

Bridges shall enforce IEEE 802.1Qci Per-Stream Filtering and Policing (PSFP), or equivalent hardware ingress rate, priority, and MTU filters, on every port, including the controller and safety supervisor ports:

- High-bandwidth perception streams (such as head camera and LiDAR streams in SR Class A or B) shall be filtered by VLAN and multicast forwarding rules so they are never forwarded onto limb actuator ports.
- Priority 7 (`SafetyHeartbeat` and immediate E-Stop / STO frames) shall be policed on ingress to its provisioned maximum frame size (<= 128 octets), committed rate matching the configured heartbeat rate (100 Hz up to the control loop rate), and a **burst allowance of at least 2 frames per heartbeat interval** (256 octets) so an immediate E-Stop frame emitted between periodic heartbeats is never dropped at the supervisor's port while still strictly bounding interference into lower CBS classes.
- No limb or peripheral port shall be permitted to inject priority 6 or 7 traffic exceeding its provisioned frame size and rate.

---

## 4. Layer-2 Transport Framing

### 4.1 IEEE 1722 Encapsulation

All robotics control, telemetry, and safety heartbeat frames are transported as IEEE 1722-2016 AVTPDUs (EtherType `0x22F0`) carrying an `ACF_GPC` message (`acf_msg_type = 0x05`, Clause 9.4.7).

- **Cyclic commands (`JointCommandArray`):** shall use **TSCF** (`subtype = 0x05`, 24-octet common stream header) with `sv = 1` and `tv = 1`. The 32-bit `avtp_timestamp` field carries the **Execution Time (`t_exec`)** in Working Clock nanoseconds modulo 2^32, representing the exact instant at which listening actuators latch the command into their local control loops (a true AVTP presentation time per IEEE 1722-2016 §4.3.2).
- **Cyclic telemetry (`JointStateArray`):** shall use **TSCF** (`subtype = 0x05`, `sv = 1`, `tv = 1`) for all streams described and connected through IEEE 1722.1 ATDECC (because IEEE 1722-2016 Annex I defines no stream format for NTSCF). Statically provisioned telemetry links not enumerated via ATDECC may alternatively use **NTSCF** (`subtype = 0x82`, 12-octet alternative header, `sv = 1`).
  - To respect IEEE 1722-2016 §4.4.4 and §9.3.1.1 (where `avtp_timestamp` is a future presentation time, not a past sample time), a Talker sending telemetry in **TSCF** shall set `avtp_timestamp` to `(reference_obs_ns + max_transit_time_ns) mod 2^32` (with `tv = 1`), indicating when the sample is guaranteed available at the controller.
  - Both TSCF and NTSCF telemetry payloads carry an explicit 64-bit `reference_obs_ns` field inside the robotics payload header (§4.3.3) so observation timing is unambiguous and identical across both encapsulations.
- **Safety heartbeats (`SafetyHeartbeat`):** shall use **TSCF** (`tv = 1`, `avtp_timestamp = (tx_time_ns + max_transit_time_ns) mod 2^32`) when described via ATDECC, and may use **NTSCF** (`subtype = 0x82`) on statically provisioned links.
- Multiple ACF messages (such as `ACF_CAN`, `ACF_SERIAL`, `ACF_PARALLEL`, `ACF_SENSOR`, and `ACF_GPC`) may be concatenated inside a single Ethernet frame per IEEE 1722-2016 §9.4.1.

### 4.2 Protocol Identifier (`gpc_msg_id`)

Every robotics `ACF_GPC` message uses a single 48-bit `gpc_msg_id` allocated from Scramble Tools' IEEE MA-S registry (`8C:1F:64:36:C0:08`, on the wire `8C 1F 64 36 C0 08`), paired with a 1-octet `message_kind` inside the robotics cycle header.

### 4.3 Payload Layouts

All multi-octet integer fields are **big-endian** (network byte order). All floating-point fields (`float32`) are **IEEE 754 binary32, big-endian**. Actuators shall reject any command slot containing a `NaN` or infinite floating-point value. Every robotics `ACF_GPC` payload is a multiple of 4 octets (quadlet-aligned) and ends with a 4-octet CRC-32C trailer (§5.1).

#### 4.3.1 Common Robotics Cycle Header (8 octets)

| Offset | Octets | Field | Type | Description |
|--------|--------|-------|------|-------------|
| 0 | 4 | `cycle_counter` | uint32 | Monotonic cycle counter, incremented by 1 each cycle (wraps after 24.8 days at 2 kHz) |
| 4 | 1 | `message_kind` | uint8 | `0x01` JointCommandArray, `0x02` JointStateArray, `0x03` SafetyHeartbeat |
| 5 | 1 | `slot_count` | uint8 | Number of 24-octet slots following the header (0 to 60 for commands and states; 0 for heartbeats) |
| 6 | 2 | `system_flags` | uint16 | Bit 0: `ARMED`, Bit 1: `ACTIVE`, Bit 2: `SOFT_ESTOP`, Bit 3: `HARD_STO_REQUEST`, Bit 4: `CLOCK_HOLDOVER`, Bits 5 to 15: reserved (0) |

#### 4.3.2 `JointCommandArray` (`message_kind = 0x01`)

Consists of the 8-octet Cycle Header, followed by `slot_count` consecutive 24-octet `JointCommand` slots (indexed `slot_index` 0 through `slot_count - 1`), followed by the 4-octet CRC-32C trailer. Maximum `slot_count` in one 1,500-octet Ethernet payload is 60 (24 TSCF + 8 ACF_GPC + 8 Cycle Header + 1,440 slots + 4 CRC = 1,484 octets).

Each 24-octet `JointCommand` slot has the following layout:

| Offset | Octets | Field | Type | Units / Encoding |
|--------|--------|-------|------|------------------|
| 0 | 4 | `position_rad` | float32 | Target joint angle, radians |
| 4 | 4 | `velocity_rad_s` | float32 | Target angular velocity, rad/s |
| 8 | 4 | `torque_ff_nm` | float32 | Feedforward torque, N*m |
| 12 | 4 | `kp` | float32 | Position stiffness gain, N*m/rad |
| 16 | 4 | `kd` | float32 | Velocity damping gain, N*m*s/rad |
| 20 | 1 | `command_mode` | uint8 | `0` Disabled, `1` Damping, `2` Impedance (PD+FF), `3` Torque, `4` Position, `5` Velocity, `6` to `255` reserved |
| 21 | 1 | `command_flags` | uint8 | Bit 0: `BRAKE_RELEASE_REQUEST`, Bit 1: `CLEAR_FAULT_REQUEST`, Bits 2 to 7: reserved (0) |
| 22 | 2 | `max_torque_q12_4` | uint16 | Torque clamp in unsigned Q12.4 N*m (1/16 N*m per LSB, 0.0000 to 4,095.9375 N*m) |

#### 4.3.3 `JointStateArray` (`message_kind = 0x02`)

Consists of the 8-octet Cycle Header, an 8-octet `reference_obs_ns` field, `slot_count` consecutive 24-octet `JointState` slots (indexed `slot_index` 0 through `slot_count - 1`), and the 4-octet CRC-32C trailer. Maximum `slot_count` in one 1,500-octet Ethernet payload is 60 (24 TSCF + 8 ACF_GPC + 8 Cycle Header + 8 `reference_obs_ns` + 1,440 slots + 4 CRC = 1,492 octets).

| Offset | Octets | Field | Type | Description |
|--------|--------|-------|------|-------------|
| 8 | 8 | `reference_obs_ns` | uint64 | Working Clock observation timestamp in nanoseconds (exact latch instant for single-joint frames, or nominal cycle latch instant for aggregated frames) |

Each 24-octet `JointState` slot begins at offset `16 + slot_index * 24`:

| Offset | Octets | Field | Type | Units / Encoding |
|--------|--------|-------|------|------------------|
| 0 | 4 | `position_rad` | float32 | Measured joint angle, radians |
| 4 | 4 | `velocity_rad_s` | float32 | Measured angular velocity, rad/s |
| 8 | 4 | `torque_meas_nm` | float32 | Measured or estimated torque, N*m |
| 12 | 2 | `temp_c_ddeg` | int16 | Winding or driver temperature in tenths of a degree Celsius (0.1 deg C/LSB, -3,276.8 to +3,276.7 deg C, covering Class F 155 deg C and Class H 180 deg C windings) |
| 14 | 2 | `bus_voltage_cv` | uint16 | Local DC bus voltage in centivolts (10 mV/LSB, 0.00 to 655.35 V) |
| 16 | 2 | `fault_flags` | uint16 | Fault bitmask (Table below) |
| 18 | 2 | `status_flags` | uint16 | Actuator FSM state and status bits (Table below) |
| 20 | 4 | `obs_offset_ns` | int32 | Signed offset in nanoseconds from `reference_obs_ns` to this joint's actual encoder latch instant |

**`fault_flags` bit assignments (16-bit bitmask):**

| Bit | Mask | Name | Meaning |
|-----|------|------|---------|
| 0 | `0x0001` | `CRC_ERROR` | One or more frames failed E2E CRC-32C verification |
| 1 | `0x0002` | `SEQUENCE_SKIP` | Cycle counter skipped, repeated, or moved backward |
| 2 | `0x0004` | `DEADLINE_MISS` | Command `t_exec` fell outside the allowed deadline window |
| 3 | `0x0008` | `WATCHDOG_TIMEOUT` | Command or heartbeat silence exceeded allowed limit |
| 4 | `0x0010` | `CLOCK_HOLDOVER` | Working Clock running on local oscillator holdover |
| 5 | `0x0020` | `CLOCK_LOST` | Working Clock holdover expired |
| 6 | `0x0040` | `SOFT_ESTOP` | Soft e-stop active |
| 7 | `0x0080` | `HARD_STO` | Hardware STO or `HARD_STO_REQUEST` active |
| 8 | `0x0100` | `OVERTEMP` | Thermal limit exceeded |
| 9 | `0x0200` | `OVERCURRENT` | Phase or bus overcurrent tripped |
| 10 | `0x0400` | `BUS_VOLTAGE` | DC bus undervoltage or regenerative overvoltage trip |
| 11 to 15 | `0xF800` | Reserved | Set to 0 on transmit, ignored on receive |

**`status_flags` bit assignments (16-bit field):**

| Bits | Mask | Name | Values / Meaning |
|------|------|------|------------------|
| 0 to 3 | `0x000F` | `safety_state` | `0` Init, `1` Armed, `2` Active, `3` CommandCoast, `4` Damping, `5` SafeTorqueOff, `6` to `15` reserved |
| 4 | `0x0010` | `HOMED` | 1 when encoder index or homing reference is established |
| 5 | `0x0020` | `BRAKE_ENGAGED` | 1 when mechanical holding brake is engaged |
| 6 | `0x0040` | `CALIBRATION_VALID` | 1 when motor and sensor calibration data are valid |
| 7 to 15 | `0xFF80` | Reserved | Set to 0 on transmit, ignored on receive |

#### 4.3.4 `SafetyHeartbeat` (`message_kind = 0x03`)

Consists of the 8-octet Cycle Header (`slot_count = 0`), an 8-octet heartbeat body, and the 4-octet CRC-32C trailer (20 octets total inside `ACF_GPC`):

| Offset | Octets | Field | Type | Description |
|--------|--------|-------|------|-------------|
| 8 | 4 | `supervisor_epoch` | uint32 | Monotonic boot or session identifier generated by the safety supervisor |
| 12 | 2 | `active_group_mask` | uint16 | Bit `N` (mask `1 << N`, `N` from 0 to 15) is 1 when logical safety group `N` is permitted to energize torque |
| 14 | 2 | `estop_reason` | uint16 | Fault or e-stop reason code (Table below) |

**`estop_reason` codes:**

| Code | Name | Meaning |
|------|------|---------|
| `0x0000` | `NOMINAL` | No e-stop active |
| `0x0001` | `OPERATOR_SOFT_ESTOP` | Operator or host requested soft stop |
| `0x0002` | `HARDWARE_STO_OPEN` | Physical e-stop button or hardware STO loop opened |
| `0x0003` | `CONTROLLER_WATCHDOG` | Controller detected missing joint telemetry or compute overrun |
| `0x0004` | `JOINT_HARDWARE_FAULT` | Joint reported critical thermal, current, or voltage fault |
| `0x0005` | `CLOCK_INTEGRITY_FAULT` | Working Clock loss or outlier integrity failure |
| `0x0006` | `POWER_BMS_FAULT` | Battery or power rail fault |
| `0x0007` | `SAFETY_BOUNDARY_FAULT` | Kinematic, torque, or workspace safety envelope exceeded |
| `0x0008` to `0x7FFF` | Reserved | Reserved for future profile revisions |
| `0x8000` to `0xFFFF` | Vendor-specific | Platform-specific cause codes |

**`SafetyHeartbeat` operation:**

- The safety supervisor shall transmit `SafetyHeartbeat` frames periodically on priority 7 at **100 Hz** (10 ms period) or faster (up to the control loop rate), and may transmit an immediate asynchronous `SafetyHeartbeat` with non-zero `estop_reason` or cleared group bit upon an emergency stop (within the 2-frame burst budget of §3.3).
- The safety supervisor shall persist `supervisor_epoch` in non-volatile storage (or derive it from a strictly monotonic boot counter) and increment it on every boot or restart, so a restarted supervisor always presents a strictly newer epoch than any previously accepted value.
- A robot-wide `SafetyHeartbeat` stream uses the broadcast sentinel **`group_id = 0xFFFF`** when deriving its E2E `data_id` (§5.1), so every endpoint computes the same `data_id` while inspecting its own bit (`1 << group_id`) in `active_group_mask`. (A platform choosing separate per-group heartbeat streams derives each stream's `data_id` with that group's `group_id`.)
- If no valid E2E heartbeat arrives within `heartbeat_timeout_ns` (default 50,000,000 ns, 50 ms), or if the node's `group_id` bit is cleared, or if `estop_reason` is non-zero, an `Active` or `CommandCoast` node shall transition immediately to `Damping` (or immediately to `SafeTorqueOff` if `HARD_STO_REQUEST` is set or `estop_reason` is `0x0002`).

---

## 5. End-to-End Black-Channel Integrity and Safety State Machine

### 5.1 Castagnoli CRC-32C, `data_id`, and Protected Header Fields

Following the black-channel principle (IEC 61784-3, informative), the Ethernet physical layer, bridges, and host stacks are treated as untrusted transport. Every robotics `ACF_GPC` payload ends with a 4-octet **CRC-32C (Castagnoli)** trailer:

- **CRC parameters:** generator polynomial `0x1EDC6F41` (Koopman notation `0x8F6E37A0`, reflected `0x82F63B78`), initial value `0xFFFFFFFF`, reflected input and output, final XOR `0xFFFFFFFF`. Check value for ASCII `"123456789"` is `0xE3069283`.
- **Exact `data_id` derivation:** Every stream binds a 32-bit `data_id` derived by computing standard CRC-32C over an 11-octet big-endian buffer:
  - Octets 0 to 7: `stream_id` (64-bit unsigned integer, big-endian)
  - Octet 8: `direction` (`0x01` Command, `0x02` State, `0x03` Heartbeat)
  - Octets 9 to 10: `group_id` (16-bit unsigned integer, big-endian; `0xFFFF` for a robot-wide `SafetyHeartbeat` stream)
- **Protected bytes and frame CRC calculation:** To protect `t_exec` (`avtp_timestamp`) and `gpc_msg_id` under the black-channel assumption without adding wire octets, the CRC-32C is computed over the concatenation of:
  1. `data_id` (4 octets, big-endian; implicit, not transmitted on the wire),
  2. `gpc_msg_id` (6 octets, as carried in the `ACF_GPC` header),
  3. `avtp_timestamp` (4 octets, big-endian, as carried in the TSCF header; `0x00000000` when encapsulated in NTSCF),
  4. the robotics payload bytes from offset 0 (the Cycle Header) through the last octet before the 4-octet CRC trailer.

  The resulting 32-bit CRC is appended in big-endian order as the final 4 octets of the `ACF_GPC` payload. Any corruption of `t_exec`, `gpc_msg_id`, payload data, or misrouting to the wrong stream or `group_id` fails CRC verification immediately.

- **Hamming distance and slot count:** CRC-32C is independent of Ethernet's IEEE CRC-32 (`0x04C11DB7`) and has hardware acceleration on x86, ARMv8, and RISC-V. Per Koopman's 32-bit CRC tables (measured in **data bits**, excluding the 32-bit CRC), CRC-32C guarantees:
  - **Hamming Distance 6** (detecting all 1-to-5-bit errors) up to **5,243 data bits** (655 octets), which covers up to **26 joint slots** per frame (`14` header/implicit octets + `8` Cycle Header + `8` `reference_obs_ns` + `26 * 24` slots = `654` octets = `5,232` data bits).
  - **Hamming Distance 4** (detecting all 1-to-3-bit errors and all odd-weight errors) up to **2,147,483,615 data bits**, covering **27 to 60 slots** (up to `1,470` octets = `11,760` data bits for a 60-slot `JointStateArray`).
  - Frames carrying 27 to 60 slots are permitted by this profile with HD = 4; limb-sized frames of 26 or fewer slots are recommended both to retain HD = 6 and to minimize store-and-forward latency (§3.2).

### 5.2 Verification and Sequence / Epoch Restart Rules

#### Command Acceptance (`JointCommandArray`)

Before accepting a command frame, an actuator shall verify:

1. E2E CRC-32C matches over `data_id || gpc_msg_id || avtp_timestamp || payload` (§5.1).
2. `stream_id` matches the node's configured command stream.
3. `avtp_timestamp` (`t_exec`) is valid (`tv = 1`) and lies within `[-max_lateness_ns, +max_exec_horizon_ns]` relative to local Working Clock time modulo 2^32 (signed 32-bit difference). Frames outside this window are rejected with `DEADLINE_MISS`.
4. **Sequence check and restart rule:**
   - While in **`Init`**, the first valid command frame establishes the baseline `last_accepted_counter` (any starting value is accepted).
   - While in **`Armed`**, **`Active`**, or **`CommandCoast`**, `cycle_counter` shall advance monotonically (`(received - last_accepted) mod 2^32` between `1` and `2^31 - 1`). Duplicates and backward values are rejected with `SEQUENCE_SKIP`; gaps greater than 1 set `SEQUENCE_SKIP`.
   - **Armed timeout:** While in **`Armed`** (torque disabled), if no accepted command frame arrives for `max_missed_cycles + 1` consecutive cycle periods, the actuator shall return automatically to **`Init`** and clear `last_accepted_counter`. Thus, if a controller restarts while actuators are `Armed`, rejected backward frames do not trap the actuators; after the timeout expires, they return to `Init` and accept the restarted controller's fresh sequence.
   - **Damping and SafeTorqueOff:** While in **`Damping`**, commands cannot re-energize torque and the actuator completes its damping interval into `SafeTorqueOff`. While in **`SafeTorqueOff`**, if a command frame passes E2E CRC-32C and the `t_exec` Working Clock deadline window (which proves temporal freshness), carries `CLEAR_FAULT_REQUEST` in `command_flags` with `ACTIVE = 0`, `SOFT_ESTOP = 0`, and `HARD_STO_REQUEST = 0`, and no local hardware fault remains latched, the actuator shall accept that frame regardless of its `cycle_counter`, clear latched faults, reset `last_accepted_counter` to the frame's `cycle_counter`, and transition to **`Init`** (or directly to **`Armed`** if `ARMED = 1` and the Working Clock is locked).
5. All `float32` fields in the node's `slot_index` are finite (not `NaN` or infinite).

#### Heartbeat Acceptance (`SafetyHeartbeat`)

Before accepting a heartbeat frame, an actuator or zonal endpoint shall verify:

1. E2E CRC-32C matches using the heartbeat stream's `data_id` (`group_id = 0xFFFF` for robot-wide heartbeats), `gpc_msg_id`, and `avtp_timestamp` (or `0x00000000` for NTSCF).
2. `stream_id` matches the node's configured heartbeat stream.
3. **Epoch and counter check:**
   - If no prior heartbeat has been accepted since `Init`, accept any `supervisor_epoch` and `cycle_counter` as baseline.
   - If `supervisor_epoch` equals the last accepted epoch, `cycle_counter` shall advance monotonically (`1` to `2^31 - 1` mod 2^32); duplicates or backward counters are rejected with `SEQUENCE_SKIP`.
   - If `supervisor_epoch` advances (`(received - last) mod 2^32` between `1` and `2^31 - 1`), indicating a supervisor restart, reset the heartbeat `cycle_counter` baseline to the received counter; if the actuator is currently `Active` or `CommandCoast`, treat the mid-operation supervisor restart as a fault and transition immediately to `Damping`. Older epochs are rejected.

### 5.3 Default Safety Parameters

Implementations shall support configuring these parameters per joint or group, with the following defaults when unspecified:

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `max_missed_cycles` | 3 cycles | Consecutive missed command cycles allowed in `CommandCoast` before entering `Damping` |
| `damping_duration_ns` | 50,000,000 ns (50 ms) | Time spent in passive viscous `Damping` before entering `SafeTorqueOff` |
| `damping_kd` | Platform-configured (`0.0` N*m*s/rad unconfigured) | Local viscous damping gain in `Damping`, persisted per joint inertia/gearing and readable via ATDECC |
| `damping_max_torque` | Platform-configured (`0.0` N*m unconfigured) | Torque clamp during `Damping`, persisted per joint; unconfigured zero falls back safely to zero-torque coast into `SafeTorqueOff` |
| `max_lateness_ns` | 0 ns (strict); 50,000 ns (relaxed) | Maximum past skew allowed for `t_exec`; 0 ns rejects any late frame |
| `max_exec_horizon_ns` | `2 * T_cycle` | Maximum future horizon allowed for `t_exec` |
| `max_clock_holdover_ns` | 50,000,000 ns (50 ms) | Maximum Working Clock holdover before `CLOCK_LOST` fault |
| `heartbeat_timeout_ns` | 50,000,000 ns (50 ms) | Maximum silence from safety supervisor before entering `Damping` |

### 5.4 Deterministic Actuator Safety State Machine

```text
Init -> Armed -> Active -> CommandCoast (<= max_missed_cycles) -> Damping (damping_duration_ns) -> SafeTorqueOff
```

- **Init (0):** power-on state before Working Clock lock and first valid E2E command; torque output disabled; accepts any starting `cycle_counter`.
- **Armed (1):** Working Clock locked and valid E2E commands arriving with `ARMED` set (`ACTIVE` clear); torque disabled. Dropping `ARMED` or exceeding `max_missed_cycles + 1` periods without an accepted command returns to `Init` and resets `last_accepted_counter`.
- **Active (2):** `ARMED` and `ACTIVE` set and safety heartbeat valid with node's `group_id` enabled; each command latches into the local control loop at `t_exec`.
- **CommandCoast (3):** one or more command cycles missed (up to `max_missed_cycles`); actuator holds its last valid safe setpoint. Receiving a valid command returns directly to `Active`.
- **Damping (4):** entered immediately upon exceeding `max_missed_cycles`, `SOFT_ESTOP`, heartbeat loss, group disable, mid-operation `supervisor_epoch` change, or `CLOCK_LOST` from an energized state. Actuator applies local passive viscous damping (`torque = -damping_kd * velocity`, clamped to `damping_max_torque`) for `damping_duration_ns`, then transitions to `SafeTorqueOff`.
- **SafeTorqueOff (5):** torque outputs disabled and mechanical brake engaged if fitted. Entered immediately on `HARD_STO_REQUEST`, `estop_reason = 0x0002`, hardware STO line break, or critical local hardware fault. Leaving `SafeTorqueOff` requires either a local/ATDECC fault clear or a fresh (`t_exec`-validated) E2E command carrying `CLEAR_FAULT_REQUEST` (with `ACTIVE = 0`, `SOFT_ESTOP = 0`, `HARD_STO_REQUEST = 0`, and local hardware faults cleared) per §5.2.

**Functional safety and residual error rate note (informative).** Conforming to this profile's wire format, CRC-32C checks, and actuator state machine does not by itself make a robot functionally safe. Any Safety Integrity Level (IEC 61508) or Performance Level (ISO 13849) claim requires a complete system safety case, certified safety logic, and an independent hardware Safe Torque Off path. Under IEC 61784-3, the residual undetected error rate scales with the message rate; at 1 to 4 kHz cyclic update rates, a safety case targeting SIL 3 should evaluate whether a single 32-bit CRC meets its probability-of-dangerous-failure-per-hour (`PFH`) budget or whether additional mitigation (such as dual-channel comparison or wider authentication) is required.

---

## 6. Discovery, Configuration, and Fast Boot

### 6.1 Stream Addressing, Slot Binding, and Fast Boot

- **`slot_index`:** the zero-based position (`0` to `slot_count - 1`) of an actuator's 24-octet slot inside a packed `JointCommandArray` or `JointStateArray` payload. Each actuator reads only its assigned `slot_index` from its limb's multicast command stream.
- **Multicast MAC addresses:**
  - Core pre-bound on-body streams shall use **statically provisioned locally administered multicast MAC addresses** (configured alongside `stream_id`, `group_id`, `slot_index`, and bridge static FDB / PSFP entries) so core locomotion and actuation lock and begin running within 200 ms of power-on without waiting for MAAP acquisition or MSRP convergence.
  - Dynamically attached or hot-swappable peripherals (such as interchangeable grippers or tool heads) shall acquire stream multicast addresses using **MAAP** (IEEE 1722-2016 Annex B). Because MAAP addresses must be defended after every restart, endpoints shall not blindly reuse persisted MAAP addresses without re-acquiring or defending them on link-up.

### 6.2 IEEE 1722.1 ATDECC Role

Every zonal controller, smart actuator, and modular end-effector runs an IEEE 1722.1-2021 Entity on priority 0 (best effort):

- **ADP:** advertises `entity_id` and `entity_model_id` for topology verification and detection of hot-swapped tools.
- **AEM descriptors:** `STREAM_INPUT` and `STREAM_OUTPUT` declare cyclic robotics streams using the TSCF stream format of IEEE 1722-2016 Annex I.2.7 (`t0v = 1`, `type_0 = ACF_GPC`), `SENSOR_CLUSTER` with vendor format `format_eui48 = 8C:1F:64:36:C0:08`, and `SENSOR_MAP` with `mapping_stream_signal = 0` (matching the Robotics TSCF Streams Profile §7); `CONTROL` descriptors expose non-real-time parameters (homing, calibration offsets, gain limits, `damping_kd`, `damping_max_torque`, fault reset, brake control); `MEMORY_OBJECT` descriptors store read-only kinematic/URDF metadata, calibration tables, and OTA firmware update images.
- **AECP unsolicited notifications:** endpoints report thermal warnings, fault flag transitions, and safety state changes asynchronously to registered controllers.

---

## 7. Conformance

| Requirement | Reference |
|-------------|-----------|
| **Bridge** | |
| Relays IEEE 802.1AS Domain 1 (Working Clock) with `externalPortConfigurationEnabled = TRUE`, fixed port roles, and support for `initialLogPdelayReqInterval = -5` and `initialLogSyncInterval` of `-7` or `-6` | §2.1, §2.2 |
| Runs IEEE 802.1AS Domain 0 with BTCA active whenever AVB or Milan media endpoints are present | §2.1 |
| Transmits and receives gPTP untagged at highest internal priority (express in Tier 2) | §3.1 |
| Supports 8 priorities, preserves priorities 3 and 2 for SR Classes A and B, supports SR Class C at priority 6, places no unshaped strict-priority queue above priorities 2 and 3, maps priorities to queues as §3.1 sets for eight and four queues, and carries robotics control in SR Class C, on VID 2 or the Robotics Control VLAN (VID 3) | §3.1 |
| Configures static CBS idle slopes for priority 7 and the SR Class C reservations of core body streams, keeps total shaped plus gated bandwidth within 75% of port rate, and enforces Tier 1 store-and-forward cycle bounds or Tier 2 TAS plus Frame Preemption | §1.2, §3.2 |
| Enforces PSFP (802.1Qci) or equivalent ingress policing on every port, including controller and safety supervisor ports, isolating perception streams from limb ports | §3.3 |
| **Controller and Safety Supervisor** | |
| Paces `JointCommandArray` TSCF frames on Domain 1 Working Clock with `t_exec` in `avtp_timestamp` and `gpc_msg_id = 8C:1F:64:36:C0:08` | §2.1, §4.1, §4.2 |
| Computes and verifies CRC-32C over `data_id \|\| gpc_msg_id \|\| avtp_timestamp \|\| payload` on all robotics frames | §5.1 |
| Emits `SafetyHeartbeat` at 100 Hz or faster on priority 7 with `group_id = 0xFFFF` (for robot-wide streams), persisted monotonic `supervisor_epoch`, `active_group_mask`, and `estop_reason` codes | §4.3.4, §5.1 |
| Uses statically provisioned multicast MACs for fast-boot core streams and MAAP for dynamically bound tools | §6.1 |
| **Actuator and Zonal Endpoint** | |
| Synchronizes to Domain 1 Working Clock supporting `initialLogPdelayReqInterval = -5`, `initialLogSyncInterval` of `-7` or `-6`, outlier filtering, and `max_clock_holdover_ns` enforcement | §2.1 to §2.3 |
| Decodes `JointCommandArray` at its configured `slot_index`, checking CRC-32C (covering `data_id`, `gpc_msg_id`, and `t_exec`), `t_exec` window, monotonic `cycle_counter` with §5.2 restart rules, and finite floats | §4.3.2, §5.1, §5.2 |
| Validates `SafetyHeartbeat` frames (CRC-32C, `supervisor_epoch`, counter, group bit, and `heartbeat_timeout_ns`) | §4.3.4, §5.2 |
| Implements the 6-state safety FSM (`Init`, `Armed`, `Active`, `CommandCoast`, `Damping`, `SafeTorqueOff`) with persisted/safe `damping_kd` and `damping_max_torque` | §5.3, §5.4 |
| Emits `JointStateArray` with 64-bit `reference_obs_ns`, `obs_offset_ns`, `temp_c_ddeg`, `bus_voltage_cv`, `fault_flags`, `status_flags`, and valid presentation `avtp_timestamp` when using TSCF | §4.1, §4.3.3 |
| Exposes an IEEE 1722.1 Entity on priority 0 for ADP discovery, AEM controls, `MEMORY_OBJECT` metadata/firmware, and AECP notifications | §6.2 |

---

## Annex A. Reference Control and Sensor Sampling Targets (informative)

This annex provides non-normative target sampling rates, control loop frequencies, latency budgets, and §3.1 priority mappings for two representative platforms: high-agility humanoid, bipedal, and quadrupedal robots, and autonomous mobile robots / automotive ADAS drive-by-wire systems.

### A.1 Humanoid, Bipedal, and Quadrupedal Robotics (Agile Locomotion and Dexterous Reflexes)

Achieving fast reflexes and stable stiff-contact interaction across running, push recovery, dexterous manipulation, and physical human-robot interaction relies on separating fast local reflexes from networked whole-body coordination, rather than streaming raw 20 kHz motor or taxel data over Ethernet:

| Subsystem | Local MCU / Sensor Rate | Networked Stream Rate | End-to-End Reflex Latency Target | Priority (§3.1) | Rationale |
|-----------|-------------------------|-----------------------|----------------------------------|-----------------|-----------|
| Motor current / torque (FOC) | 20 to 50 kHz (20 to 50 µs) | Local only | < 100 µs | Local | Matches PWM carrier; makes each joint behave as an ideal torque source on sudden impact |
| Local joint impedance (`kp`, `kd`) | 4 to 20 kHz | Setpoints at 2 kHz (4 kHz stretch) | < 250 µs | 6 (setpoints) | Evaluates `torque_ff + kp*e + kd*de` locally off the joint encoder with near-zero phase lag |
| Whole-body cyclic actuation | Local FOC | 2 kHz (500 µs) target; 4 kHz (250 µs) stretch | < 1 ms sense-to-actuate | 6 | 10 to 20x above limb/drivetrain mechanical resonance (20 to 150 Hz); preserves 150 to 250 µs compute budget |
| Foot and wrist 6-axis F/T | 2 to 4 kHz (anti-aliased) | 2 kHz (locked 1:1 to control cycle) | < 500 µs | 6 (SR C) | Captures steep 2 to 5 ms rigid heel-strike and insertion contact transients cleanly |
| Fingertip slip / micro-vibration | 2 to 4 kHz (piezo, barometric, shear) | 1 kHz continuous plus slip event stream | < 1 to 2 ms (local hand reflex < 500 µs) | 6 (SR C) | Incipient stick-slip and hard-contact vibrations lie in 50 to 800 Hz (human Pacinian band) |
| Dense whole-body tactile skin | 200 to 500 Hz taxel scan | 100 to 250 Hz spatial map plus impact event stream | < 5 to 10 ms | 6 (SR C, impact), 3 (SR A, map) | Static pressure, object weight, and body collision localization sit below ~100 Hz |
| Optical tactile fingertips | 60 to 120 Hz (global shutter) | 60 to 120 Hz | < 15 ms | 3 or 2 (SR A/B) | High spatial geometry/shear reconstruction; pairs with 2 kHz piezo/barometer for fast slip |
| Chassis / head IMU | 1 to 4 kHz | 1 kHz | < 1 ms | 6 (SR C) | Synchronized on Working Clock for state estimation and visual-inertial odometry |
| Whole-body MPC / RL policy | Internal to compute | 50 to 200 Hz | 5 to 20 ms | Internal | Feeds smooth trajectory references and impedance gains into the 2 kHz whole-body controller |

- **Why 2 kHz networked plus local impedance is the sweet spot:** 1 kHz (1 ms) already exceeds human monosynaptic spinal reflex speed (~25 to 40 ms), but rigid contact and stiff force control benefit substantially from 2 kHz (500 µs). Because `JointCommand` (§4.3.2) carries `kp` and `kd` directly to each joint MCU, high-rate PD damping runs locally at 4 to 20 kHz with zero wire delay. Pushing the networked loop to 4 kHz (250 µs) is practical in Tier 2 (or on 1 Gb/s limb links with limb-sized frames) for ultra-stiff assembly or haptic teleoperation, whereas higher network rates show diminishing returns compared to faster local reflexes.
- **Why tactile sensing splits by frequency:** Separating 2 to 4 kHz local slip/impact detection (emitting SR Class C event streams on threshold crossings) from 100 to 250 Hz whole-body spatial maps prevents thousands of idle taxels from saturating limb links while keeping grasp-slip and collision reaction times under 1 to 2 ms.

### A.2 Autonomous Mobile Robots and Automotive ADAS / Drive-by-Wire

Wheeled autonomous vehicles and heavy mobile robots exhibit lower mechanical resonance frequencies than bipedal limbs (chassis, suspension, steering rack, and braking hydraulics typically sit at 2 to 30 Hz), while carrying higher perception bandwidth (multi-camera surround vision, 4D imaging radar, and multi-beam LiDAR) and strict safety-integrity requirements on steer-by-wire and brake-by-wire paths:

| Subsystem | Local ECU / Sensor Rate | Networked Stream Rate | End-to-End Reflex Latency Target | Priority (§3.1) | Rationale |
|-----------|-------------------------|-----------------------|----------------------------------|-----------------|-----------|
| Traction / EPS / EMB motor FOC | 10 to 20 kHz (50 to 100 µs) | Local only | < 200 µs | Local | Drives steering assist, electro-mechanical brakes (EMB), and traction inverters smoothly |
| Steer-by-wire / brake-by-wire / torque vectoring | Local 1 to 2 kHz inner servo | 500 Hz to 1 kHz (1 to 2 ms) target; 200 Hz minimum | < 2 to 5 ms | 6 | 10 to 30x above vehicle yaw/steering/brake mechanical bandwidth (5 to 30 Hz); supports crisp ABS/ESP/stability reflexes |
| Emergency brake / AEB / STO trigger | Local safety monitor | Event-driven plus 100 Hz `SafetyHeartbeat` | < 1 to 2 ms wire delivery | 7 (STO/HB), 6 (SR C, AEB event) | Immediate collision-avoidance actuation and fail-operational fallback across redundant paths (802.1CB FRER optional) |
| Wheel-speed, steering-angle, brake-pressure sensors | 1 to 2 kHz | 500 Hz to 1 kHz (locked to control cycle) | < 1 to 2 ms | 6 (SR C) | Feeds ABS/ESC slip estimation, odometry, and dead reckoning on one shared Working Clock |
| 6-axis chassis IMU / GNSS-INS | 1 kHz IMU; 10 to 50 Hz GNSS | 200 Hz to 1 kHz IMU; 10 to 50 Hz GNSS | < 2 ms (IMU) | 6 (SR C) or 3 (SR A) | High-rate attitude/acceleration for ego-motion compensation of LiDAR and radar sweeps |
| 4D imaging radar | Internal chirp processing | 20 to 50 Hz (20 to 50 ms) | < 20 ms | 3 or 2 (SR A/B) | Point-cloud / range-Doppler frames aligned to Working Clock for all-weather obstacle tracking |
| 3D LiDAR | 10 to 20 Hz spin / solid-state | 10 to 20 Hz (packetized continuously) | < 20 ms | 3 or 2 (SR A/B) | High-throughput point streams confined to 1 Gb/s+ perception trunks by VLAN/PSFP |
| Surround and front ADAS cameras | 30 to 60 Hz (global shutter) | 30 to 60 Hz (CVF, RVF, or 1722-2025 GISF) | < 15 to 33 ms | 3 or 2 (SR A/B) | Coherent multi-camera shutter sync via Working Clock; CBS shaping prevents bursts from delaying control |
| Ultrasonic / short-range proximity | 20 to 50 Hz | 20 to 50 Hz plus close-contact alert event stream | < 5 to 10 ms (alert) | 6 (SR C, alert), 2 (SR B, stream) | Low-speed parking, docking, and bumper-proximity reflex |
| ADAS sensor fusion and path planning | Internal to compute | 20 to 50 Hz (20 to 50 ms) | 20 to 50 ms | Internal | Computes drivable trajectories and longitudinal/lateral targets feeding the 500 Hz to 1 kHz drive-by-wire controller |

- **Coexistence and redundancy:** Because ADAS drive-by-wire loops run at 500 Hz to 1 kHz (1 to 2 ms cycles) while multi-camera and LiDAR perception consume hundreds of megabits per second on gigabit trunks, Tier 1 (1 Gb/s trunks with CBS + PSFP) easily meets automotive latency bounds, while Tier 2 adds TAS (802.1Qbv), Frame Preemption (802.1Qbu / 802.3br), and optional FRER (802.1CB) plus 802.1AS-2025 Hot Standby on safety-critical steer-by-wire and brake-by-wire paths.
