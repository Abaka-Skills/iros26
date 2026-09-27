# WidowX AI meshes

STL meshes and joint origins in this folder come from
[TrossenRobotics/trossen_arm_description](https://github.com/TrossenRobotics/trossen_arm_description)
(BSD-3-Clause, see `LICENSE`), specifically the **WidowX AI follower** and **leader** arms used in the
Trossen AI Stationary kit.

`wxai_follower.json`, `wxai_leader_left.json` and `wxai_leader_right.json` are derived files: the
link/joint trees extracted from `urdf/generated/wxai/wxai_follower.urdf`, `wxai_leader_left.urdf`
and `wxai_leader_right.urdf` so the browser can assemble the chains without a URDF parser. The
leader handle and finger meshes come from the same repository.
Geometry is unmodified.
