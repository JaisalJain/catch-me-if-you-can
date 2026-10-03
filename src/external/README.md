# Pinned robot descriptions

Git submodules at the jazzy release tags that packages.ros.org builds its binaries from. The workspace overlay
takes precedence over any version installed under `/opt/ros` (including NVIDIA Isaac builds), so the project does
not depend on system package versions.

| Package | Release repo | Tag | Commit |
|---|---|---|---|
| `robotiq_description` | https://github.com/ros2-gbp/ros2_robotiq_gripper-release | `release/jazzy/robotiq_description/0.0.1-3` | `4131e61c1f07d1e4deeef87ac2f4da3a8e2e6637` |
| `ur_description` | https://github.com/ros2-gbp/ur_description-release | `release/jazzy/ur_description/3.5.1-1` | `6e45dd1bba8810a1e6250ff35be20cba1e838600` |

Used: `ur_description` (`ur_macro.xacro`, `config/ur5e/*.yaml`, `meshes/ur5e`), `robotiq_description`
(`robotiq_2f_85_macro.urdf.xacro`, read by `tools/gen_gripper_urdf.py`; 9 visual meshes). Licenses: BSD-3-Clause
(`ur_description`) and BSD (`robotiq_description`), see each `package.xml`; in `ur_description` only the UR8 Long /
UR15 / UR18 / UR20 / UR30 meshes are under Universal Robots' graphical-documentation terms (not used here). Overview:
`THIRD_PARTY_NOTICES.md`.

Bump: `git -C src/external/<pkg> fetch --tags && git -C src/external/<pkg> checkout <new tag>`, then rerun
`tools/gen_gripper_urdf.py` (robotiq) and the tests, and update this table.

Build with `colcon build --allow-overriding robotiq_description ur_description` (without the flag colcon prints
an override warning; both packages are description-only, no headers or libraries).
