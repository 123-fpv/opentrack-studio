# Quick start

OpenTrack Studio is a track editor for **VelociDrone**, with English and
Simplified Chinese UI. Use the language selector, or start with `--lang en` /
`--lang zh`. Language selection does not translate or replace your project data.

1. Drag one clear track diagram into the image box, overhead view or 3D view.
   Local recognition generates an editable draft. One Undo restores the state
   before the import. Dropping several photos opens the optional AI workflow;
   it does not send them automatically.
2. Check the physical gate structures and the separate numbered pass sequence.
   Select a pass to inspect its entry/exit direction. A progressive upright wrap
   changes height while circling the upright and reaches another aperture.
   Inspect the actual current route before replacing it with a manual template.
3. Correct missing structures, order, faces, dimensions, heights and wrap sides.
   Click a component once to select its halo, then drag it on a second press.
   Right-drag to orbit, middle-drag to pan, wheel to zoom.
4. Set the independent takeoff grid, base height and heading. It is separate
   from the first checkpoint. No landing point is required by the editor.
5. Save `.otproj` to keep the complete editable path, image and associations.
   Export `.trk` to create native simulator objects and ordered checkpoints.
6. Put the `.trk` file in Windows Documents and import it in VelociDrone's
   **Track Editor → Import Track**. Check the scene and perform a flight test.

T1, T2, etc. are essential winding controls, not additional physical gates.
Ordinary fitted samples are not exported as checkpoints. Do not remove an
essential T control merely to bypass its required wrapping side.

Source images do not cover the 3D ground. Use the optional overhead reference
when comparing the imported image. Frame cells default to 3 m centreline
spacing, 3.2 m beams and a 0.1 m square section; verify real measurements.

Recognition supports specific clean diagram styles and may still misinterpret
crossings or hidden depth. A model-clearance check is not a guarantee of
physical flight or race-rule validity. `.trk` cannot recover every preview
guide and aperture offset; keep the `.otproj` source project.

Try the public synthetic [threading example](../examples/threading-demo.otproj).
It and the screenshots do not contain private user diagrams.

The current source is [GPL-3.0-or-later](../LICENSE). Contributions are welcome
through [forks and pull requests](../CONTRIBUTING.md); user-created track data
is not automatically licensed under GPL.
