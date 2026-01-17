# How to continue README/docs edits with the AI assistant

Purpose
- Keep a short, checked-in note that documents where we left off and how to continue the work from another computer.

What this file contains
- Short summary of recent edits (files changed).
- Commands to regenerate annotated images (Inkscape CLI) used in this repo.
- A small checklist of follow-up tasks you or the AI can pick up later.

Useful commands (Windows PowerShell)
- Re-generate the annotated PNG from the overlay SVG (export width 747):
  "\"/c/Program Files/Inkscape/bin/inkscape.com\" docs/xbox-controller-overlay.svg --export-type=png --export-filename=docs/xbox-controller.png --export-width=747"
- Export a higher-resolution PNG (2x):
  "\"/c/Program Files/Inkscape/bin/inkscape.com\" docs/xbox-controller-overlay.svg --export-type=png --export-filename=docs/xbox-controller@2x.png --export-width=1494"

Files of interest
- `docs/controller_template.png` — base bitmap artwork (editable in Paint, Photoshop)
- `docs/xbox-controller-overlay.svg` — vector overlay that places text boxes and markers
- `docs/xbox-controller.png` — exported annotated PNG (what README displays)

How to resume the AI conversation (best practices)
1. Push any local changes to the repository and open an issue or create a branch describing what you want next.
2. Mention the file `docs/AI_CONTINUE.md` and the branch name in your chat with the AI so it can read the repo state.
3. If you want the AI to continue automatically, include: the exact file path(s) to edit, the text to add/remove, and any pixel/position tweaks (or place a colored pixel marker in the template image to indicate exact locations).

Quick checklist (current state)
- [x] Generated `docs/xbox-controller.png` (annotated image)
- [x] Edited `README.md` to embed the annotated image
- [x] Created/kept `docs/controller_template.png` (editable base)
- [x] Created `docs/xbox-controller-overlay.svg` (overlay source)
- [ ] Optional: export high-res annotated PNG for smoother scaling in Markdown preview
- [ ] Optional: add a CONTRIBUTING.md or short doc that explains how to add new labels + export flow

If you want me to continue now, tell me which checklist item to work on and I will do it and commit the changes.

— End of AI_CONTINUE.md
