# crater_field (lunar, default environment)

Default lunar environment, moved from `worlds/lunar_surface.sdf`
(WP-05 Step 2, no behaviour change).

- World file: `crater_field.sdf` (`<world name="crater_field">`; file
  and world name stay in sync so the launcher can derive the
  `/world/<name>` services from the SDF). Bare `crater_field` resolves
  here via the variant directory.
- Meshes: `meshes/lunar_crater_environment.glb`, referenced by the SDF
  via the relative URI `meshes/lunar_crater_environment.glb` (unchanged).
  Keep the SDF next to its `meshes/` subdir; add the variant dir to
  `GZ_SIM_RESOURCE_PATH`.
- Blender sources: `lunar_crater_environment.blend` / `.blend1` are the
  terrain authoring sources. They are git-ignored (`*.blend`, `*.blend1`)
  and stay local only; they are not needed at runtime.
