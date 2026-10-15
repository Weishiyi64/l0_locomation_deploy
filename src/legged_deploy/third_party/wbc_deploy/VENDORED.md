# Vendored WBC packaging subset

This directory contains only the runtime files used by
`text_motion_wbc_packager.py`, copied from the local `wbc_deploy` project:

- `scripts/deploy/prepare_monolithic_deploy.sh`
- `scripts/deploy/prepare_robot_deploy_onnx.py`
- `scripts/deploy/build_monolithic_robot_policy.py`

Training assets, MuJoCo assets, cached bundles, recordings and the upstream Git
history are intentionally excluded. `README.upstream.md` and `requirements.txt`
are retained for provenance and dependency information.
