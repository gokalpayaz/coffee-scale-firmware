# Optional and legacy files

Nothing in this folder is required by the normal ESP32 boot path. Do not copy
this folder when uploading `firmware` to the scale.

- `calibration` contains the optional serial calibration utility and its
  debounce helper. The normal firmware uses the on-device calibration wizard.
- `host` contains PC-only development and flashing dependencies.
- `legacy` contains display drivers and helpers retained for reference but no
  longer imported by `boot.py`, `main.py`, or their dependencies.
