# Third-party notices

The repository's original Python code, configuration, workflows, and
documentation are licensed under Apache License 2.0 unless a file or directory
states otherwise. The following bundled assets and vendored code retain their
upstream licenses.

## Zerith H1 SDK (vendored)

- Paths: `third_party/H1_SDK_1.3.7/`
- Source: Zerith H1 SDK v1.3.7 Python 3.12 bindings (robot SDK and camera
  SDK), vendored from the vendor SDK distribution.
- License: the vendor SDK distribution does not include a license text file.
  Redistribution is subject to the vendor's SDK terms; contact Zerith for
  the applicable terms.

Only the Python runtime packages (`h1_sdk_v1.3.7_py3.12/`,
`camera_sdk_python/`) with prebuilt cpython-312 extensions are vendored; the
full C++ SDK trees are not included.

## H1 PRO model assets

- Paths: `assets/mjcf/`, `assets/glb/`, `assets/urdf/`
- The MJCF, URDF, and GLB model assets were imported from the vendor-provided
  H1 PRO model package and are maintained as delivery assets in this
  repository. See the README files in each asset directory for details.

## Runtime dependencies

Python dependencies are not vendored into this repository. Their licenses are
provided by their respective distributions, including:

- `av`: BSD-3-Clause (PyAV / FFmpeg bindings)
- `dora-rs`: MIT (according to the installed PyPI distribution metadata)
- `forge-common`, `forge-msgs`, `forge-robot`: see their PyPI distributions
- `numpy`: BSD-3-Clause
- `opencv-python`: Apache-2.0
- `protobuf`: BSD-3-Clause
- `PyYAML`: MIT
- `Typer`: MIT
