# ee_description source snapshot

The `*.source` files are copied verbatim from `~/Downloads/ee_description.zip`
(2026-08-26). The suffix prevents ROS linters from treating the non-UTF-8 ROS 1
export as a buildable package.
The authoritative URDF/Xacro defines only `base_link`; it defines no joints or TCP.
The four STL files are retained under `meshes/ee_description`, but only `base_link.stl`
is referenced because the other meshes have no link/joint placement in the source.

Integration renames `base_link` to `ee_base_link` to avoid the arm's existing
`base_link`. `wrist_to_ee` is an integration-only fixed attachment whose zero origin
is an explicit placeholder until a mating transform is supplied.
