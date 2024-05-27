# Modulo

Modulo is part of the AICA robotics framework.

It is an extension layer to ROS 2 that adds interoperability support for
[aica-technology/control-libraries](https://github.com/aica-technology/control-libraries) and provides a modular
framework for [application composition](https://docs.ros.org/en/iron/Concepts/About-Composition.html) through custom
component classes in both C++ and Python.

Modulo API documentation is available at [aica-technology.github.io/modulo](https://aica-technology.github.io/modulo/).

To start developing custom components and controllers with modulo, refer to the
[aica-technology/component-template](https://github.com/aica-technology/component-template) repository and corresponding
documentation at https://docs.aica.tech/docs/category/custom-components.

## Modulo Core

The core package implements interoperability between ROS 2 and state_representation data types,
allowing state data and parameters to be directly translated and handled as messages on the ROS 2 interface layer.

## Modulo Components

Modulo components are wrappers for ROS 2 Nodes which abstract low-level ROS methods for greater user convenience and
consistency between components.

While ROS 2 Nodes provide a highly flexible and customizable interface, there is often a significant amount of 
boilerplate code that is duplicated with each new Node. In addition, the interoperability of multiple nodes in an
application depends on them using compatible message types and behaving in a generally consistent manner.

The component classes are intended to simplify the development of compatible application modules.
They provide more concise methods for adding parameters, services, transforms, topic subscriptions and publications.
They also introduce new concepts such as predicate broadcasting, error handling and the direct binding of
data attributes to their corresponding interface.

The package provides two variant classes: `modulo_components::Component` and `modulo_components::LifecycleComponent`.
See the package documentation for more information.

## Modulo Component Interfaces

This package defines custom standard interfaces for modulo components.

## Modulo Utils

This package contains shared test fixtures.

---

## Additional resources

### Interfacing with ROS 1

It is possible to use a bridge service to interface ROS 2 with ROS 1 in order to publish/subscribe to topics from ROS 1.

ROS provides a docker image with this service. Run the following lines to pull the bridge image, launch an interactive
shell, and subsequently start the bridge topic.

```bash
# run the bridge image interactively
docker run --rm -it --net=host ros:galactic-ros1-bridge
# start the bridge service in the container
root@docker-desktop:/# ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
```

### Further reading

- [aica-technology/component-template](https://github.com/aica-technology/component-template)
- [aica-technology/control-libraries](https://github.com/aica-technology/control-libraries)
- [aica-technology/docker-images](https://github.com/aica-technology/docker-images)
- [ROS 2 Managed Nodes](https://design.ros2.org/articles/node_lifecycle.html)

### Authors and maintainers

- [Baptiste Busch](https://github.com/buschbapti)
- [Enrico Eberhard](https://github.com/eeberhard)
- [Dominic Reber](https://github.com/domire8)