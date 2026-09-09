# Janus MAVLink contract v1

Janus uses standard MAVLink wherever ArduPilot already defines the required
state. The protocol is passive: none of these inputs can arm the vehicle,
operate a release actuator, or create a navigation target.

## Payload mass

The STM32 stores payload mass in the standard persistent ArduPilot parameter
`CIMP_MASS` in kilograms. A companion or ground station writes it with the
normal MAVLink parameter protocol (`PARAM_SET`, or `PARAM_EXT_SET` when that
client uses the extended protocol) and must verify the echoed value before
flight. Valid range is 0.05-50 kg. `CIMP_CDA` stores drag coefficient times
reference area in square metres; mass alone is not sufficient for a realistic
wind prediction.

## Camera calibration

For a fixed lens, Janus first uses the standard ArduPilot parameters
`CAM1_HFOV`/`CAM1_VFOV` or `CAM2_HFOV`/`CAM2_VFOV`. `CIMP_C1_HFOV`,
`CIMP_C1_VFOV`, `CIMP_C2_HFOV`, and `CIMP_C2_VFOV` are fallbacks for analogue
cameras that do not populate the standard camera settings. `CIMP_CAMSEL`
selects the video input whose calibration is projected on OSD2.

Set `CIMP_C1_TYPE` or `CIMP_C2_TYPE` to `1` for a varifocal camera. That camera
then requires fresh FOV packets and hides the marker when they expire; it never
silently reuses a stale zoom value.

## Private TUNNEL transport

Dynamic FOV and optional external predictions use MAVLink `TUNNEL` with
`payload_type=32769`. Values 0-32767 are reserved for registered transports;
32769 is deliberately local to the private Janus firmware and must not be sent
to stock ArduPilot. Target system/component are honoured. MAVLink framing
provides the transport checksum; deployments should also enable MAVLink 2
signing on the carrying telemetry link.

All packet integers are little-endian, floats are IEEE-754 binary32, and every
packet is exactly 36 bytes. Common header: magic `0x49434144`, protocol version
`1`, message type, frame, flags, monotonically increasing `uint32` sequence.
Reserved bytes must be zero.

- Type 1, external impact: local-NED frame `1`; north/east/down metres, time to
  impact seconds, horizontal uncertainty metres, and bounded validity in ms.
- Type 2, camera FOV: frame `0`; zero-based camera index (`0` or `1`), horizontal
  and vertical FOV in degrees, and bounded validity in ms.

Receivers reject wrong lengths, magic/version/frame, unset validity, invalid
ranges, non-zero reserved bytes, duplicate or out-of-order sequences, and stale
updates. The external prediction mode also invalidates its point if the EKF
origin changes.
