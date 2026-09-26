# WidowX AI meshes

STL meshes and joint origins in this folder come from
[TrossenRobotics/trossen_arm_description](https://github.com/TrossenRobotics/trossen_arm_description)
(BSD-3-Clause, see `LICENSE`), specifically the **WidowX AI follower** arm used in the
Trossen AI Stationary kit.

`wxai_follower.json` is a derived file: the link/joint tree extracted from
`urdf/generated/wxai/wxai_follower.urdf` so the browser can assemble the chain without a URDF parser.
Geometry is unmodified.
