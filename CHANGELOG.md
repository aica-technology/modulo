# CHANGELOG

Release Versions:

- [4.2.1](#421)
- [4.2.0](#420)
- [4.1.0](#410)
- [4.0.0](#400)
- [3.2.0](#320)
- [3.1.0](#310)
- [3.0.0](#300)
- [2.3.0](#230)
- [2.2.0](#220)
- [2.1.1](#211)
- [2.1.0](#210)

## Upcoming changes

- refactor: rename function in modulo translators (#77)
- feat: add mutex around step callback (#67)
- refactor: move utilities to modulo_utils package (#89)
- refactor: move exceptions to modulo_utils (#90)
- refactor: remove modulo_component_interfaces (#88)

## 4.2.2

### June 14, 2024

Version 4.2.2 is a patch that brings a few fixes and minor improvements to the controller interface. Mainly, it improves
the controller stability by safely handling a potentially failing dynamic pointer cast.

- fix(modulo-controllers): improve log message (#110)
- fix(modulo-components): add missing space in log message (#113)
- feat(modulo-controllers): add missing docs (#112)
- fix(modulo-controllers): correctly handle nullptr in read_input (#118)

## 4.2.1

### May 29, 2024

Version 4.2.1 is a patch that fixes a missing version update in the aica-package.toml and a missing action in the
build-release workflow.

- ci: create git tag on release (#109)

## 4.2.0

### May 28, 2024

Version 4.2.0 features a new package `modulo_controllers` that extends the controller interface from ROS 2 control with
concepts and capabilities known from `modulo_components`. These classes can and should be used to write derived
ROS 2 controller plugins within the AICA robotics framework. Additionally, this version introduces the package builder
that replaces the Dockerfile to improve clarity, maintenance and standardization of the build system.

- feat: improve logging in parameter translators (#65)
- fix(component-interface): make python subscriptions type safe (#71)
- build: change base workspace image version (#83)
- feat: add modulo controllers (#84, #93, #94, #95, #96, #97, #98, #99)
- build: aica-package.toml for package-builder:v1 (#91)

### Dependencies

- package-builder: v1 (#91)
- ros2-ws: v1.0.1-iron (#83, #91)
- control-libraries: 7.4.0 -> 7.4.1 (#91)

## 4.1.0

### April 19, 2024

Version 4.1.0 is a minor update which adds support for Analog and Digital IO States from control librareis and contains
a few quality of life improvements. Additionally, it comes with a new package called `modulo_interfaces` which is a more
general version of `modulo_component_interfaces` and will replace the latter upon the next major release.

- chore: fix links in readme (#68)
- feat: add warn log when topics are sanitized (#66)
- feat: add modulo interface package (#73)
- feat: support io states in modulo core (#70)

### Dependencies

- control-libraries: 7.1.0 -> 7.4.0 (#76)

## 4.0.0

### November 7, 2023

Version 4.0.0 is a major update to modulo with a new non-templated component interface.

### Breaking changes

The component interface now uses node interfaces - a new feature in ROS2 Iron - instead of templated inheritance from
nodes to interface with ROS. Due to that change in the components inheritance, C++ components built with modulo 3.x.x
and below will not be compatible with modulo 4.0.0 and will need to be recompiled. Note that this only concerns the
`modulo_components` package and that there is no need to change anything in the code, recompiling derived components
will be sufficient.

- chore: remove unnecessary tests (#63)
- Remove callback group for topics and services (#61)
- Avoid conflict with get_parameter (#42)
- Revise ComponentInterface by moving implementations to source file (#39)
- Update component classes to inherit from new ComponentInterface (#38)
- Refactor ComponentInterface to use node interfaces (#33)

## 3.2.0

### October 23, 2023

Version 3.2.0 is a minor update to modulo with a new feature that allows the component period parameter to be set
through a new parameter called 'rate'. In the future, the period parameter will be deprecated.

Additionally, this is the last release with the current release strategy of creating release branches from develop and
merging them to main. From now on, branches will be directly created from main and merged back to main. If there is a
version update in such a PR, the CI will automatically create a new tag and push a tagged image.

### Features

- feat: add rate parameter to component interface (#57)

### Behind the scenes

- build: prepare workflows for deletion of develop branch (#58)

## 3.1.0

### September 27, 2023

Version 3.1.0 is a minor update to modulo with a new feature in the components and it marks the first official version
that uses `iron` as its ROS2 distribution.

### Features

- Add the on_step_callback for components (#51)

### Behind the scenes

- Revise test stage to fail on test errors (#48)
- Unify dockerfiles and add vs code configuration (#50)
- Use iron as ros2 distro (#53)
- Change test workflow to docker build (#55)

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