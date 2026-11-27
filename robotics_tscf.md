# Robotics TSCF Streams Profile

This tsLAN profile defines AVB stream types for sensor data and actuator control, carried as IEEE 1722 TSCF streams, reserved like audio and video, and described and connected through IEEE 1722.1.

Version: 1.0-draft

---

## 1. Scope

Robots, machines and vehicles stream measurements to controllers and setpoints to actuators at rates from a few hertz to a few kilohertz. IEEE 1722 already carries such data in its control formats, and IEEE 1722.1 can describe and connect the streams, but neither says which messages to use, what the values mean, or how to reserve for them. This profile does, so that a sensor and a controller from different makers can be connected as readily as a microphone and an amplifier.

This profile specifies:

- **Stream types:** sensor, setpoint and control streams, all TSCF (§2).
- **Value formats:** standard layouts for common measurements, in SI units (§3).
- **Timing:** what the stream and message timestamps mean (§4).
- **SR classes:** one additional SR class, Class C, for control loops, with Classes A and B for other streams and as a fallback (§5).
- **Packing:** how many values and messages go in one frame (§6).
- **Discovery and connection:** how IEEE 1722.1 describes each stream's contents (§7).

Control laws, functional safety, and media-like sensors such as cameras and microphones are out of scope.

### 1.1 Roles

- **Sensor talker:** an ATDECC entity that samples one or more sensors and sends sensor streams.
- **Actuator listener:** an ATDECC entity that receives setpoint or control streams and applies them.
- **Motion controller:** an ATDECC entity that receives sensor streams and sends setpoint or control streams. It is named so here to tell it apart from an ATDECC controller.
- **ATDECC controller:** the ATDECC role that enumerates entities and connects their streams.
- **Bridge:** an AV bridge that forwards and shapes the streams, and supports Class C (§5) where it can.

### 1.2 Relation to Other Profiles and Milan

- **Basic Audio Profile:** a sensor channel sampled faster than 4 kHz, such as a contact microphone or a tactile vibration pickup, is audio for transport purposes and is carried as an AAF stream under the Basic Audio Profile (informative).
- **Basic Video Profile:** sensors that produce images, such as cameras and optical tactile sensors, use the Basic Video Profile.
- **Robotics Profile:** its joint command and joint state arrays are control streams of this profile (§2.3), with `gpc_msg_id` `8C:1F:64:36:C0:08` as their cluster format; its IMUs, force/torque sensors and tactile maps can be sensor streams of this profile. Its Working Clock, safety heartbeat and safety state machine stay in that profile.
- **Milan:** Milan entities need no change: they use only Classes A and B and ignore the streams and SR class of this profile. Class C sits above A and B, though, so on a port carrying both its traffic adds to Class A and B latency; §5.1 caps it and has the network check Class A's bound.

### 1.3 Draft Notes (informative)

Points open in this draft:

- Compact integer value formats, for low-bandwidth links, may follow in a later revision.
- Setpoint semantics for drives (modes, enabling, faults) are left to profiles such as the Robotics Profile; alignment with the IEC 61800-7 drive profiles is a candidate for a later revision.
- SR classes beyond C, such as Classes D and E at priorities 5 and 4 for event and sensor streams, are deferred, and may follow if bench measurements show Classes A, B and C are not enough (§5.1).

---

## 2. Stream Types

### 2.1 Common Rules

Every stream of this profile:

- is an IEEE 1722-2016 TSCF stream (§9.3, `subtype` `0x05`) with `sv = 1`;
- carries one or more ACF messages per AVTPDU (§9.4.1), all of one message type;
- is reserved in an SR class (§5) and connected with ACMP like any other stream.

NTSCF is not used: it has no presentation time, and IEEE 1722-2016 Annex I defines no stream format for it, so IEEE 1722.1 cannot describe an NTSCF stream (informative).

A TSCF stream may list up to four message types (Annex I.2.7), but this profile's clusters and mapping (§7) describe streams of one type only. A stream that mixes message types in one AVTPDU is defined by the profile that uses it, not by this one (informative).

| Stream type | Direction | ACF message | SR class (§5) |
|-------------|-----------|-------------|---------------|
| Sensor | Sensor talker to motion controller | `ACF_SENSOR` (`0x08`) | C, or A, where it feeds a control loop; otherwise B, or A |
| Event | Sensor talker to motion controller | `ACF_SENSOR` (`0x08`) | C, or A |
| Setpoint | Motion controller to actuator listener | `ACF_SENSOR` (`0x08`) | C, or A |
| Control | Motion controller to actuator listener, or the reverse | `ACF_GPC` (`0x05`) | C, or A |

### 2.2 Sensor, Event and Setpoint Streams

Sensor, event and setpoint streams carry values in `ACF_SENSOR` messages (IEEE 1722-2016 §9.4.10). Each message holds the values of one SENSOR_CLUSTER (§7.2):

| Field | Value |
|-------|-------|
| `mtv` | 1 in sensor and event streams; 0 in setpoint streams unless their value format says otherwise |
| `num_sensors` | The number of values, as the value format (§3) gives it; 0 means 128 |
| `sz` | 0: 4-octet values |
| `sensor_group` | The cluster's signal number within the stream, 0 to 63 (§7.3) |
| `message_timestamp` | The acquisition time (§4.2) |
| `sensor_msg_payload` | The values, in the order of the value format |

- **Sensor stream:** sent every sample period, whether or not the values have changed.
- **Event stream:** sent only when the sensor talker detects an event, such as a contact, slip or threshold crossing; each message carries the values that triggered it, and `message_timestamp` gives when the event happened. What counts as an event is set by the sensor talker's maker or its CONTROL descriptors.
- **Setpoint stream:** sent every control cycle; an actuator listener applies the values at the stream's presentation time (§4.1). Setpoint streams suit actuators driven by plain values, such as a joint position or a valve opening (informative).

Setpoint streams use `ACF_SENSOR` rather than `ACF_SENSOR_BRIEF`, and so carry 8 unused timestamp octets per message, so that sensor, event and setpoint streams share one message type and one stream format (§7.1) (informative).

### 2.3 Control Streams

Control streams carry `ACF_GPC` messages (IEEE 1722-2016 §9.4.7), for setpoints and replies that need more than plain values, such as gains, modes, flags and end-to-end protection.

- The `gpc_msg_id` identifies the payload layout, and is the EUI-48 of the profile or maker that defines it, from its own MA-L, MA-M or MA-S (§9.4.7.2).
- The defining profile or maker gives the layout, its timing, and any integrity checks. The Robotics Profile's joint arrays are one such layout.
- The cluster format of a control stream (§7.2) is its `gpc_msg_id`, so an ATDECC controller can tell control streams apart.

---

## 3. Value Formats

### 3.1 Encoding

The value formats of this profile carry each value as an IEEE 754 binary32, big-endian, in the SI unit the format gives.

- **Sensor and event streams:** a value that a sensor cannot provide for a sample, such as an axis out of range, is sent as a quiet NaN, and a listener treats it as missing.
- **Setpoint streams:** a motion controller does not send NaN. An actuator listener treats a NaN setpoint as invalid, and handles it as it handles a setpoint lost with its frame.
- **Control streams:** NaN handling is the defining profile's; the Robotics Profile rejects a joint command containing one.
- Vectors are in the sensor's own right-handed frame, as its maker documents it (informative).

### 3.2 Identification

A value format is named by an EUI-48 carried as the cluster format (§7.2). A format is either one of this profile's (§3.3) or a maker's own, named by an EUI-48 from the maker's MA-L, MA-M or MA-S, with its layout documented by the maker. A listener accepts a cluster only if it knows the cluster's format.

### 3.3 tsLAN Value Formats

| Format ID | Name | Values (`num_sensors`) | Order and unit |
|-----------|------|------------------------|----------------|
| `8C:1F:64:36:C0:10` | Linear acceleration | 3 | x, y, z, in m/s² |
| `8C:1F:64:36:C0:11` | Angular velocity | 3 | x, y, z, in rad/s |
| `8C:1F:64:36:C0:12` | Orientation | 4 | Unit quaternion w, x, y, z |
| `8C:1F:64:36:C0:13` | Magnetic field | 3 | x, y, z, in T |
| `8C:1F:64:36:C0:14` | Force and torque | 6 | Fx, Fy, Fz in N, then Tx, Ty, Tz in N·m |
| `8C:1F:64:36:C0:15` | Joint position | 1 to 128 | One per joint, in rad (revolute) or m (prismatic) |
| `8C:1F:64:36:C0:16` | Joint velocity | 1 to 128 | One per joint, in rad/s or m/s |
| `8C:1F:64:36:C0:17` | Joint torque or force | 1 to 128 | One per joint, in N·m or N |
| `8C:1F:64:36:C0:18` | Pressure | 1 to 128 | One per taxel or port, in Pa |
| `8C:1F:64:36:C0:19` | Temperature | 1 to 128 | One per point, in °C |
| `8C:1F:64:36:C0:1A` to `8C:1F:64:36:C0:1F` | Reserved | | For later formats |

The same formats serve sensor, event and setpoint streams: a joint position cluster in a setpoint stream carries target positions (informative).

**Value counts.** For a format of 1 to 128 values, a talker keeps a cluster's `num_sensors` fixed while the stream runs. Value *i* is the cluster's *i*th joint, taxel or point, in the order the talker's maker documents. A listener uses the values at the indices it is configured for, counts an index at or beyond `num_sensors` as missing, and ignores values beyond those it uses. IEEE 1722.1-2021 SENSOR_CLUSTER has no field for a value count, so motion controllers and actuator listeners learn it from configuration or from the first message received (informative).

---

## 4. Timing

### 4.1 Presentation Time

Each AVTPDU's `avtp_timestamp` is its presentation time (IEEE 1722-2016 §4.3.2), with `tv = 1`:

- **Sensor and event streams:** the acquisition time of the AVTPDU's values plus the stream's maximum transit time.
- **Setpoint and control streams:** the time at which the actuator listener applies the values, which the motion controller chooses no earlier than its transmit time plus the maximum transit time.

### 4.2 Acquisition Time

In sensor and event streams, each `ACF_SENSOR` message's `message_timestamp` is the time the values were sampled, or the event happened, in nanoseconds of gPTP time, as IEEE 1722-2016 Equation (10) gives it. All the messages in one AVTPDU of a sensor stream have the same acquisition time (§6).

### 4.3 Time Base

Timestamps are in the time of the gPTP domain that the entity's clock domain follows: domain 0, unless a profile says otherwise. The Robotics Profile uses its Working Clock domain (informative).

### 4.4 Rates

Sensor streams run at 1 Hz to 4 kHz, setpoint and control streams at 10 Hz to 4 kHz, and event streams at up to 4 kHz at their highest event rate. A cluster's rate is its SENSOR_CLUSTER's `current_sampling_rate` (§7.2).

---

## 5. SR Classes and Reservation

### 5.1 Class C

IEEE 802.1Q-2022 defines SR classes C to G beyond A and B, enabled through management (§35.2.2.9.2, Table 35-6). This profile uses one of them:

| SR class | SRclassID | Priority (default) | classMeasurementInterval (default) | Streams |
|----------|-----------|--------------------|------------------------------------|---------|
| C | 4 | 6 | 500 µs | Setpoint, control and event streams, and sensor streams that feed a control loop |

- **Priority.** IEEE 802.1Q-2022 gives default priorities only to Classes A and B, and leaves the others to management (§35.2.2.9.2, §12.20.4); 6 is this profile's default for Class C. A network may map Class C to another priority, provided every station in the SRP domain uses the same one (§35.1.4), and adjusts the queue mapping below to match. A talker takes Class C's priority from the Domain declaration it receives (§35.2.2.9.3), and uses 6 until it receives one.
- **VLAN.** Class C's SRclassVID is the SR VLAN, VID 2 by default, which a talker likewise takes from the Domain declaration (§35.2.2.9.4).
- **Interval.** The 500 µs classMeasurementInterval is this profile's default, configured through IEEE 802.1Q-2022 §12.20.1, which defines defaults only for Classes A (125 µs) and B (250 µs). The Domain declaration does not carry the interval, so a network that changes it, for example to its control cycle so that a stream sending one frame per cycle reserves exactly that, sets the same value on every bridge and talker in the SRP domain. A talker counts its `MaxIntervalFrames` (§5.3) against that interval.
- **Boundary ports.** A bridge port that is an SRP domain boundary port for Class C regenerates received frames of Class C's priority to priority 0, through its Priority Regeneration Override Table (§6.9.4, §12.20.3), as Table 6-5 does by default for Classes A and B. Frames from outside the domain then cannot take Class C's reservation.
- **PICS.** A bridge or end station that supports Class C states so in its PICS, as IEEE 802.1Q-2022 requires for SR classes beyond B.

A Class C interval that differs between devices harms only Class C streams, as Class C's shaper still bounds its traffic: a bridge interval longer than a talker's reserves too little and delays the stream, and a shorter one reserves more than it uses (informative).

Class C keeps control deadlines of 500 µs and less apart from the 2 ms and 50 ms bounds of Classes A and B. Further classes, such as D and E at priorities 5 and 4, are deferred until bench measurements show a need for them; until then, streams that do not need Class C use Class B or A (informative).

**Queues.** IEEE 802.1Q-2022's default mapping for Classes A and B (Table 34-1) puts priorities 3 and 2 in the two highest traffic classes and priorities 6 and 7 below them, in strict-priority queues. A bridge supporting Class C therefore also configures:

- its Traffic Class Table (§12.6.3), so priorities 7 and 6 take traffic classes above those of priorities 3 and 2, distinct ones where the port has more than four; and
- its Transmission Selection Algorithm Table (§12.20.2), so priority 6's traffic class uses the credit-based shaper, as every SR class does (§34.5).

For ports with eight and with four traffic classes, the mapping is:

| Priority | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 |
|----------|---|---|---|---|---|---|---|---|
| Traffic class, of eight | 1 | 0 | 4 | 5 | 2 | 3 | 6 | 7 |
| Traffic class, of four | 0 | 0 | 1 | 2 | 0 | 0 | 3 | 3 |

With eight traffic classes, priorities 0, 1, 4 and 5 keep their Table 34-1 traffic classes, below Classes A and B. A profile that shapes or polices priorities 4 and 5, as the Robotics Profile does, may place them above Classes A and B instead (informative).

**Four traffic classes.** A port with four traffic classes, such as one on a network controller with four transmit queues, has room for Classes C, A and B and one more. There, priority 7 shares Class C's traffic class, much as Table 34-1 groups priorities where traffic classes are few, and priorities 0, 1, 4 and 5 share the lowest:

- the bridge polices priority 7 on ingress to the rate and burst its profile sets, and adds that rate to the traffic class's idle slope, so Class C keeps its reservation and Classes A and B their bounds; and
- a port with more traffic classes keeps priorities 7 and 6 apart.

The cost falls on priority 7: its frames wait behind the Class C frames already queued, and for the shaper's credit, instead of going ahead of them. Each 676-octet AVTPDU (§6) ahead adds about 57 µs at 100 Mb/s and 6 µs at 1 Gb/s, and a full-size frame about 123 µs at 100 Mb/s. A safety profile that needs a tighter bound for priority 7 uses ports with more than four traffic classes (informative).

This profile leaves priority 7 to safety profiles, such as the Robotics Profile's safety heartbeat. Traffic at priority 7 is shaped or policed, so the shaped classes below keep their reservations, as IEEE 802.1Q-2022 §34.5 b) and Annex L.1 require.

**Sharing a port with Classes A and B.** Class C sits above A and B, so its traffic adds to the interference Class A and B frames see at each hop (IEEE 802.1Q-2022 Annex L.3). On a port carrying Class A or B streams:

- the bridge keeps the bandwidth reserved for Classes A, B and C together within 75% of the port's transmit rate, through each class's deltaBandwidth (§34.3); and
- a network carrying Milan streams checks that Class A stays within its 2 ms bound over its hop count, counting the largest bursts Class C and priority 7 can send. The Robotics Profile's §3.2 works one such case: about 48 µs per hop on a 1 Gb/s trunk (informative).

### 5.2 Choosing a Class

A talker learns the SR classes of its port from the MSRP Domain declarations it receives (IEEE 802.1Q-2022 §35.2.2.9).

- **Setpoint, control and event streams, and sensor streams that feed a control loop:** Class C where the talker's port declares it, and Class A where it does not.
- **Other sensor streams:** Class B, or Class A where they need its lower latency.

A talker sets the CLASS_B flag of its STREAM_INFO for Class B as usual.

**Failure along the path.** A Class C reservation needs every bridge between talker and listener to support the class. Where one does not, as at an SRP domain boundary for the class, MSRP turns the Talker Advertise into a Talker Failed (IEEE 802.1Q-2022 §35.2.2.8.7, failure codes in Table 46-15), listeners declare Listener Asking Failed (§35.1.2.2), and the stream is not forwarded. A talker whose Class C stream draws only Listener Asking Failed declarations withdraws it and declares it again in Class A.

IEEE 1722.1-2021 STREAM_INFO can only signal Classes A and B, and a CLASS_B flag of 0 means Class A, so an ATDECC controller shows a Class C stream as Class A. The stream's priority on the wire is still Class C's, from the Domain declaration (informative).

### 5.3 Reservation

A talker declares each stream's TSpec as IEEE 802.1Q-2022 §35.2.2.8.4 defines it: `MaxFrameSize` is its largest AVTPDU, and `MaxIntervalFrames` the most AVTPDUs it sends in one classMeasurementInterval, at least 1. An event stream reserves for its highest event rate.

Because `MaxIntervalFrames` is at least 1, a stream slower than its class's interval reserves more than it uses. The example below is a 1 kHz IMU stream of two clusters, linear acceleration and angular velocity: 72-octet AVTPDUs, 114 octets on the wire (informative):

| SR class | Interval | Reserved | Used |
|----------|----------|----------|------|
| C | 500 µs | 1.82 Mb/s | 0.91 Mb/s |
| B | 250 µs | 3.65 Mb/s | 0.91 Mb/s |
| A (fallback) | 125 µs | 7.30 Mb/s | 0.91 Mb/s |

A network whose control cycle is 1 ms can set Class C's interval to match, and the stream then reserves only the 0.91 Mb/s it uses (informative).

### 5.4 Fast Start

Where streams have to run before MSRP has converged, as at a robot's power-on, bridges may hold static reservations and forwarding entries for them, and talkers send without waiting for listener registrations. The Robotics Profile does so for its core body streams (informative).

---

## 6. Packing

- **One frame per sample instant.** A sensor talker puts every cluster it samples at the same instant, at the same rate, into one AVTPDU, one `ACF_SENSOR` message per cluster. Clusters at different rates go in different streams.
- **Streams per device and rate.** A sensor talker sends one sensor stream for each rate it samples at, not one per sensor.
- **Control per actuator group.** A motion controller sends one setpoint or control stream for each group of actuators that act together, such as a limb, with each actuator taking its clusters or slots, rather than one stream for all actuators.
- **Size.** An AVTPDU should stay within 676 octets, which keeps forwarding time short on 100 Mb/s links, and keeps a CRC-32C end-to-end check at Hamming distance 6 (up to 5,243 data bits) where a profile adds one, as the Robotics Profile does over its payload and 14 octets of header and implicit identifier.
- **Small frames are free.** An AVTPDU shorter than 42 octets, such as one carrying a single one-value cluster, is padded to the minimum tagged Ethernet frame anyway, so packing a second small cluster into it costs little (informative).

Each frame of a sensor stream carries about 66 octets of Ethernet and TSCF overhead, and each `ACF_SENSOR` message 12 more, so packing several clusters into one frame raises the share of the link that carries values: a single 3-value cluster uses about 13% of its frame, four such clusters about 30% (informative).

---

## 7. Discovery and Connection

### 7.1 Stream Formats

A STREAM_OUTPUT or STREAM_INPUT of this profile lists the TSCF stream format of IEEE 1722-2016 Annex I.2.7 with `m = 0`, `t0v = 1`, `type_0` set to `ACF_SENSOR` or `ACF_GPC`, and the other types not valid.

### 7.2 Clusters

Each stream's contents are SENSOR_CLUSTERs (IEEE 1722.1-2021 §7.2.18) of the stream's STREAM_PORT_OUTPUT or STREAM_PORT_INPUT:

- `current_format` is a vendor-defined sensor cluster format (§7.3.12.2, Figure 7-26): `v = 1`, `type = 0`, the reserved bits 0, and `format_eui48` the cluster's value format (§3.2), or for a control stream its `gpc_msg_id` (§2.3).
- `current_sampling_rate` is the cluster's rate (§7.3.1); for an event stream, its highest event rate.
- The `object_name` or `localized_description` says what the cluster measures or drives, such as "left wrist force/torque" (informative).

### 7.3 Mapping

A SENSOR_MAP (IEEE 1722.1-2021 §7.2.21), or GET_SENSOR_MAP for dynamic mappings (§7.4.50), maps each cluster to its stream: `mapping_stream_signal` is the `sensor_group` of the cluster's `ACF_SENSOR` messages, and so 0 to 63 although the field is 16 bits, and 0 for a control stream.

### 7.4 Connection

An ATDECC controller connects these streams with ACMP like any other. It should connect a talker's stream to a listener only where every cluster the listener maps has the same format as the talker's cluster for that signal.

---

## 8. Conformance

| Requirement | Reference |
|-------------|-----------|
| **Sensor talker** | |
| Sends sensor and event streams as TSCF with `ACF_SENSOR` messages, `sz = 0`, one message per cluster | §2.1, §2.2 |
| Encodes values in a tsLAN value format or its maker's documented format, binary32 big-endian, NaN for missing values | §3 |
| Sets `avtp_timestamp` to the presentation time and `message_timestamp` to the acquisition time | §4 |
| Uses Class C for event streams and sensor streams that feed a control loop where its port declares it, and A otherwise or when the reservation fails; uses B or A for other sensor streams; with a TSpec within §5.3 | §5 |
| Takes Class C's priority and VID from the Domain declaration, 6 and 2 until it receives one, and counts frames against the domain's Class C interval | §5.1 |
| Packs clusters of one sample instant into one AVTPDU, one stream per rate | §6 |
| Describes each stream with the TSCF stream format, SENSOR_CLUSTERs and a sensor mapping | §7 |
| **Motion controller** | |
| Sends setpoint streams as TSCF with `ACF_SENSOR`, or control streams with `ACF_GPC` and a registered `gpc_msg_id` | §2 |
| Sets each AVTPDU's presentation time to when the values apply | §4.1 |
| Uses SR class C where its port declares it, and A otherwise or when the reservation fails | §5 |
| Takes Class C's priority and VID from the Domain declaration, 6 and 2 until it receives one, and counts frames against the domain's Class C interval | §5.1 |
| Sends one setpoint or control stream per actuator group | §6 |
| **Actuator listener** | |
| Applies setpoint and control values at the presentation time | §4.1 |
| Treats a NaN setpoint as invalid, as for a lost setpoint, and follows the defining profile for control streams | §3.1 |
| Uses the configured indices of a variable-count cluster, counting absent ones as missing | §3.3 |
| Describes its inputs with SENSOR_CLUSTERs whose formats name what it accepts | §7 |
| **Bridge** | |
| Supports SR class C, at priority 6 unless configured otherwise, and states so in its PICS, or acts as an SRP domain boundary for it | §5.1 |
| Uses the same Class C priority and interval as the rest of its SRP domain, 6 and 500 µs by default | §5.1 |
| Regenerates Class C's priority to 0 on ports that are SRP domain boundary ports for Class C | §5.1 |
| Maps priority 6 to a credit-shaped traffic class and priority 7 to a shaped or policed one, both above those of Classes A and B; with four traffic classes, polices priority 7 into Class C's, counting its rate in the idle slope | §5.1 |
| Keeps Classes A, B and C within 75% of the port's transmit rate where Classes A or B share the port | §5.1 |
| **ATDECC controller** | |
| Connects these streams only between clusters of matching format | §7.4 |
