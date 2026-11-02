# Source SBOM

Source repository inventory only; completeness is UNKNOWN. Syft inventories tracked source/package manifests. Gitlinks and declared vendored/model evidence are inspected at immutable pins. CMake declarations are conditional requirements, not resolved installations. This is NOT proof of operating-system packages, static-link closure, build-time resolution, or the contents of release binaries.

Generation: **success**

## Source identity

wave-owl/sstv-core `6e3cc06a266f0bd0c19ee2a0f9df975ea27a6669`

- sbom:role: source-root
- sbom:git-commit: 6e3cc06a266f0bd0c19ee2a0f9df975ea27a6669
- sbom:git-tree: 87bf06f10803485b17ee3e4cb3cfb3647e4aa9d5
- sbom:source-repository: https://github.com/wave-owl/sstv-core
- sbom:release-tags: ["v0.1.0"]
- sbom:license-evidence: [{"path": "LICENSE", "sha256": "c5ac5d99cc60c0fb26d7b381037982f0ea928c02ef4a40f4de1f44aef8f102fe"}]
- sbom:license-resolution: unresolved; manual review required

## Generation errors

None.

## Components and unresolved licenses

| Component | Version | Role | License |
| --- | --- | --- | --- |
| wave-owl/sstv-core | 6e3cc06a266f0bd0c19ee2a0f9df975ea27a6669 | source-root | unresolved; manual review required |
| wave-owl/fldigi-dsp-core | f6f9af1af811243b89abc53132de27d5e2620850 | gitlink | unresolved; manual review required |
| wave-owl/pySSTV | d998fad154d3e6ad2d73af5add49beec0d2ab59f | reference-only | unresolved; manual review required |
| wave-owl/robot36 | 2af2390954aa248725042ba6b1332bbcca68c357 | reference-only | unresolved; manual review required |
| wave-owl/slowrx | ca6d7012ae788b5057646170bd86590a7f68bd69 | reference-only | unresolved; manual review required |
| src/sstv_rx.c | ca6d7012ae788b5057646170bd86590a7f68bd69 | declared-vendored-source | unresolved; manual review required |
| src/sstv_modes.c | ca6d7012ae788b5057646170bd86590a7f68bd69 | declared-vendored-source | unresolved; manual review required |
| src/sstv_tx.c | d998fad154d3e6ad2d73af5add49beec0d2ab59f | declared-vendored-source | unresolved; manual review required |
| src/sstv_pil.c | unresolved | declared-vendored-source | unresolved; manual review required |
| actions/checkout | v4 | Syft source/package discovery | unresolved |
| actions/setup-java | v4 | Syft source/package discovery | unresolved |
| actions/setup-python | v5 | Syft source/package discovery | unresolved |
| actions/upload-artifact | v4 | Syft source/package discovery | unresolved |
| /Users/gerolfziegenhain/src/wave-owl/local/sbom-core-preflight-v3/sstv-core/.source-input/.github/workflows/ci.yml | unresolved | Syft source/package discovery | unresolved |

License evidence paths/hashes, source manifests, model hashes and conditional
CMake declarations are recorded in SBOM.cdx.json. Declared upstream pins do not
prove vendored equivalence. No dependency installation or build was performed.
