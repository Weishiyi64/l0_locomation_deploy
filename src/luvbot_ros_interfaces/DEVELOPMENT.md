# Development and release workflow

## Node repository setup

Copy `luvbot_interfaces.repos.example` into the consuming repository, replace
`INTERFACE_REPOSITORY_URL`, and rename it to `luvbot_interfaces.repos`.  Keep
the release tag pinned.

```bash
mkdir -p interface_ws/src node_ws/src
vcs import interface_ws/src < luvbot_interfaces.repos
source /opt/ros/humble/setup.bash
colcon build --base-paths interface_ws/src --install-base /opt/luvbot/interfaces/1.0.0
source /opt/luvbot/interfaces/1.0.0/setup.bash
colcon build --base-paths node_ws/src --install-base node_ws/install
```

A node repository must build in a clean workspace with only its declared ROS
dependencies and the pinned interface release.  It must not use paths into
another team's source tree.

## Interface change review

1. Update the `.msg`/`.srv`, `CONTRACT.md`, and affected mock contract tests in
   one PR.
2. Record producers, consumers, migration order, rollback, and wire
   compatibility in the PR template.
3. Obtain approval from the system interface owner and at least one affected
   producer or consumer owner.
4. Build and test on Ubuntu 22.04 amd64 and arm64.
5. Update both package versions using semantic versioning, merge, then create
   an annotated repository tag such as `v1.1.0`.
6. Consumer repositories update their `.repos` tag in separate reviewed PRs.

Do not add node code, launch files, robot mappings, CAN/HPM protocol details,
policy assets, or reusable runtime utilities to this repository.

## Breaking changes

Published types are immutable.  Introduce an explicit V2 type and a new topic,
run old and new endpoints during migration, and remove V1 only in the next
major release after every producer and consumer has moved.
