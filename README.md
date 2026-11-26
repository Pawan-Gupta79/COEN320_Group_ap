# COEN320 p2.3 Display System
This is the display system for the air traffic control simulation. It receives data from the aircraft and displays it on the screen.
- You need to make the project as a separate process and run it in parallel with the aircraft process.
- You can use the shared memory to read aircraft data and IPC to send notifications to the communications system.
- Feel free to use any display layout to show the aircraft data, but make sure it is clear and easy to read.
- The notifications for collision warnings should be displayed prominently on the console.
- To be able to send commands to the aircraft, you can use IPC to send messages to the communications system, which will then forward the commands to the aircraft. (Check `Aircraft.cpp`: `Aircraft::updatePosition()`)
- **DONT USE RAW PRINTS TO DISPLAY AIRCRAFT DATA**

---

# How to build and run (group_ap)

Three Momentics projects: **Lab4_ATC** (aircraft + radar), **Lab5_Computer** (ATC_Computer: collision detection, operator console, comms) and **Lab5_Display** (Display).
Import each folder with *File → Import → Existing Projects into Workspace*, then build for aarch64le.

Run on the target **in this order**, each in its own terminal / console:

1. `Display` (use `Display --plain` in the Momentics console, which doesn't understand colour codes)
2. `ATC_Computer`
3. `Lab4_ATC_ARCH64`: needs `group_ap_planes.txt` in its working directory

Display and ATC_Computer wait for the Radar, and all three exit by themselves when the last aircraft leaves.

Operator commands (typed into ATC_Computer): `speed <id> <vx> <vy> <vz>`, `alt <id> <z>`, `pos <id> <x> <y> <z>`, `info <id>`, `lookahead <s>`, `help`, `exit`.
Commands are logged to `group_ap_operator_log.txt`.

Shared resource names (all in `Msg_structs.h`, identical in every project):

| Resource | Name |
|---|---|
| Shared memory | `/group_ap_radar_shm` |
| Radar channel | `group_ap` |
| Aircraft channels | `group_ap_plane_<id>` |
| ComputerSystem channel | `group_ap_computer` |
| Comms channel | `group_ap_comms` |
| Display channel | `group_ap_display` |
