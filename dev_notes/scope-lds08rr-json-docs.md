Original Outcome (verbatim): Füge die JSON-Datei, die du oben erstellt hast, in einem passenden Ordner zur Dokumentation ein.
Source: user message in this chat, 2026-10-13. Referenced artifact /tmp/lds08rr-zeichnung/lds08rr-lochbild.json is the previously created LDS08RR mounting-hole and side-view dimension record.
Acceptance Criteria: The existing JSON is stored durably within docs/ros2_pc/hardware, discoverable from the LDS08RR page, with unchanged numeric data, coordinates, interpretations and unknowns. Its source images and drawing references resolve inside the repository without /tmp paths. Record provenance in existing agent observations as project policy requires.
Approved Approach: Copy JSON and its three referenced assets; adjust only paths, add one short mechanical documentation section, and update only directly contradicted mechanical-status statements.
Explicit Non-Goals: CAD or firmware changes, new measurements, resolving ambiguous dimensions, changing electrical notes, other sensors or touchscreen documentation, runtime operation.
Compatibility / Migration Requirements: NONE
Baseline: 97536f695f044cc3d219405404964a7a4097d501, no task commits.
Pre-existing changes: all git status entries at entry are excluded, including earlier LDS08RR docs and notes. Exact pre-task copies of the two files this task will edit are /tmp/navbot-lds08rr-json-baseline/lds08rr-lidar.md and observations.md. Existing source JSON /tmp/lds08rr-zeichnung/lds08rr-lochbild.json is the authority for this copy.
PStack entry: poteto-mode and technical-writing, small contained documentation task.
Frozen plan:
1. Copy JSON to docs/ros2_pc/hardware/mechanics/lds08rr-dimensions.json. Preserve all data except references: source.image -> ../images/lds08rr-mounting-sketch.jpg; side_view.source.image -> ../images/lds08rr-side-sketch.jpg; drawing -> lds08rr-lochbild.svg.
2. Copy /tmp/lds08rr-zeichnung/lds08rr-lochbild.svg to docs/ros2_pc/hardware/mechanics/lds08rr-lochbild.svg. Preserve two original user sketches as docs/ros2_pc/hardware/images/lds08rr-mounting-sketch.jpg and lds08rr-side-sketch.jpg. No generated PNG needed.
3. Add a brief mechanical dimensions section linking JSON and SVG to docs/ros2_pc/hardware/lds08rr-lidar.md; distinguish user-sketch data from independent measurement. Update existing claims that mounting spacing/dimensions are absent to point to this record, preserving remaining unknowns.
4. Append a concise observation/evidence/consequence entry linking the JSON to agent_notes/ros2_pc/sensors/lds08rr/observations.md.
5. Verify JSON parses, all asset references resolve, numbers/data remain identical except paths, and originals are copied byte for byte. Inspect actual task diff and git diff --check; obtain fresh independent Luna final scope check.
Expected extent: one copied JSON (~134 lines), one copied SVG, two original JPEG assets, at most ~20 changed/added prose lines across two existing files, plus this local contract. No other files.
User amendment, 2026-10-13: "kein review". Review portion of step 5 skipped at explicit user direction. JSON data equality, referenced-file existence, byte-identical asset copies, and diff whitespace checks passed. No review agent dispatched.
