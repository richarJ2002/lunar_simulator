# Contributing docs

Tracked guidance for collaborators: keep the site matching the code in
the same change.

If you change `src/**`, `parameters/**`, `config/*bridge*`, `launch/**`,
or `post_processing/**`, update the matching `docs/**/*.md` too:
purpose, topics, key files, params, diagram, plus GitHub links to the
YAML/source on `main`.

- Never paste tuning numbers as authority; link to the file instead.
- Verify: `mkdocs build --strict` (no ROS/Gazebo needed).
- Keep Mermaid text-in-markdown; no checked-in diagrams.
