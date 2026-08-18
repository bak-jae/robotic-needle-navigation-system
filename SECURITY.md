# Security and safety

This is research software that can command physical motors. It is not a certified medical
device and does not provide a complete functional-safety system.

Do not include security reports containing patient data, credentials, private network details,
or proprietary vendor files in a public GitHub issue. Report sensitive vulnerabilities to the
maintainer address listed in `src/ros2_epos_cmd/package.xml`.

Before operating hardware, validate travel limits, homing, command units, motor direction,
CAN configuration, watchdog behavior, and a physical emergency-stop or power-removal method.
Object detection output must not be connected directly to motor commands without independent
limit checks and a defined fail-safe state.
