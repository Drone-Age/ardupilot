# Janus firmware line

This branch is the Drone Age Janus firmware line based on the exact stable ArduPilot Copter 4.7.1 source. Janus version 0.1.0 is displayed and delivered as `4.7.1-j.0.1.0`.

The `CARGO_IMPACT_ASSISTANT` build option is optional and disabled by default. Enabling it adds the passive cargo-impact prediction and OSD2 marker; it does not select VINS, external navigation, or another EKF source.

Release builds must use `MatekH743` (board ID 1013), add `AP_CUSTOM_FIRMWARE_STRING` with the Janus identity, record the exact effective build options, and preserve the upstream ArduPilot version independently. Build artifacts use `arducopter-MatekH743-4.7.1-j.0.1.0-<config8>.apj`.

AI-assisted implementation: the Janus 0.1.0 cargo-impact changes and delivery integration were prepared with Codex under Drone-Age/iVINS-SERVER issue #52. Hardware acceptance remains mandatory before flight use.

`run_sitl_acceptance.py` performs the repeatable software gate without Gazebo,
RViz, VINS, or external odometry. It selects GPS navigation, verifies the OSD2
marker before arming, climbs at increased speed, releases a rigidly carried 2 kg
payload at 50 m only after horizontal speed falls below 0.25 m/s in constant
10 m/s wind, and compares the logged prediction with the simulated landing
point. The gate fails above 1 m horizontal error.
