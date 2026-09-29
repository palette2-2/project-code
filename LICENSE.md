# Licensing status

This repository contains material from several projects. **No single new license
is granted for the entire repository by this notice.** Existing copyright and
license notices are retained; this file does not override them.

| Material | Retained notice |
| --- | --- |
| Inherited HOMIE deployment code | [Original license](licenses/HOMIE-LICENSE.txt), [original README](docs/upstream/HOMIE-README.md) |
| Unitree SDK2 | [BSD-3-Clause](third_party/unitree_sdk2/LICENSE) |
| SDK bundled dependencies | [Vendor license directory](third_party/unitree_sdk2/licenses/) |
| XRoboToolkit Python callback patch | [MIT notice](licenses/XRoboToolkit-Pybind-LICENSE.txt) |

The inherited HOMIE license text says **CC BY-NC 4.0**, while its README says
**CC BY-NC-SA 4.0**. These are different statements. Both are preserved for
traceability; the discrepancy has not been resolved or replaced with a more
permissive license. The inherited `setup.py` previously contained a BSD label
that did not describe the repository as a whole; that misleading package
metadata has been removed.

The release license for the project's own additions, teleoperation integration
and ONNX weights has not yet been specified by the maintainer. Inclusion in this
local repository does not establish a new license for those materials.

Before public distribution, the maintainer needs to settle the inherited license
discrepancy, state the license and provenance of their own contributions, and
confirm the intended distribution terms for the checkpoint. See
[third-party notices](THIRD_PARTY_NOTICES.md) for the recorded sources and changes.
