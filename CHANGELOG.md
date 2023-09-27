# CHANGELOG

Release Versions:

- [3.0.0](#300)
- [2.3.0](#230)
- [2.2.0](#220)
- [2.1.1](#211)
- [2.1.0](#210)

## Upcoming changes (in development)

- Revise test stage to fail on test errors (#48)
- build: unify dockerfiles and add vs code configuration (#50)
- feat: add on_step_callback for component (#51)
- build: use iron as ros2 distro (#53)

## 3.0.0

### July 26, 2023

Version 3.0.0 is a major update to modulo with a breaking change in the way predicates are handled. Instead of
individual topics per predicate, predicates are now published to a global `/predicates` topic.

### Breaking changes

- Refactor component predicates with a single Predicate publisher (#26)

## 2.3.0

### July 25, 2023

Version 2.3.0 is a minor update to modulo that improves test coverage and adds a feature to publish outputs manually.
It also contains behind-the-scenes structural improvements to the build system and the CI.

### Changes

- feat(ci): add prebuilt modulo image akin to other internal ones (#36)
- Add and install component descriptions (#31)
- Apply AICA C++ style guide (#30)
- Add option to publish outputs manually instead of periodically (#23)
- Add unittests for modulo component service calls and triggers (#20)
- Extend testutils with C++ classes (#19)

## 2.2.0

### March 21, 2023

Version 2.2.0 is an update to support the latest release of
[aica-technology/control-libraries version 7.0](https://github.com/aica-technology/control-libraries/releases/tag/v7.0.0).
This release contains no breaking changes to the modulo API and includes a few additional features and improvements.

### Features and improvements

Since `rclpy.lifecycle` was introduced in ROS2 humble, the `LifecycleNode` is now used as base class for the
`modulo_components.LifecycleComponent`.

The 'lookup_transform' method has been overloaded with another signature that allows to specify the validity period for
the requested transform.

There is a new package `modulo_utils` that contains fixtures for creating tests for modulo components. 

### Behind the scenes

`state_representation` states will not be published unless they are not empty, i.e. contain data and information that
are worth transmitting.

A contributor license agreement and signature workflow have been added to protect and encourage open source development.

The majority of changes were required to fix `modulo_core` after breaking changes on the develop branch of
control libraries. In addition, there were some improvements in the CI actions and workflows.

### Changes

- Do not publish empty states (epfl-lasa#129)
- Fix test actions and workflows (epfl-lasa#147, epfl-lasa#148)
- Use rclpy lifecycle node as base class for lifecycle components (epfl-lasa#134, epfl-lasa#140, epfl-lasa#146)
- Fix creation of triggers and improve tests (epfl-lasa#149)
- Change order of execution in step callback (epfl-lasa#150)
- Transfer repository ownership to AICA (#1)
- Add lookup transform variant with validity period (#2)
- Update tests after removal of SpatialState constructor (#3)
- Remove build-push workflow (#4)
- Fixes to account for breaking changes in control libraries (#6)
- Fix parameter translator after Parameter class changes (#8)
- Fix include of clproto header (#9)
- Add modulo_utils package for shared test fixtures (#11)
- Add more test fixtures to modulo utils (#12)
- Require CLA signatures with PR workflow (#14)
- Update control libraries version to 7.0.0 (#15)

## 2.1.1

### October 28, 2022

Version 2.1.1 is a patch release to update the license of the Modulo project from MIT to GPLv3. A core dependency
of modulo is [control libraries](https://github.com/aica-technology/control-libraries), making it a combined work.
To be compliant with the GLPv3 license of control libraries, modulo must also be licensed under GPLv3.

### Changes

- Change license to GPLv3 (epfl-lasa#143)

## 2.1.0

### October 21, 2022

Version 2.1.0 is the most recent release of modulo authored by AICA employees under EPFL employment and granted to AICA
SA under a non-exclusive license agreement. All subsequent contributions have been made solely by AICA employees and
are the property of AICA SA. For a full history of changes prior to version 2.1, refer to the original project here:

- https://github.com/epfl-lasa/modulo/tree/v2.1.0