# Modulo Components

This package provides base component classes as wrappers for ROS2 Nodes for convenience and consistency when
developing modules for a dynamically composed application.

## Component

The `modulo_components::Component` (C++) and `modulo_components.Component` (Python) classes are wrappers for
the standard ROS Node that simplify application composition through unified component interfaces.

This class is intended for direct inheritance to implement custom components that perform one-shot or
externally triggered operations. Examples of triggered behavior include providing a service, processing signals
or publishing outputs on a periodic timer. One-shot behaviors may include interacting with the filesystem or
publishing a predefined sequence of outputs.

Developers should override `on_validate_parameter_callback()` if any parameters are added and `on_execute_callback()`
to implement any one-shot behavior. In the latter case, `execute()` should be invoked at the end of the derived
constructor.

## LifecycleComponent

The `modulo_components::Component` (C++) and `modulo_components.Component` (Python) classes are state-based alternatives
that follow the [lifecycle pattern for managed nodes](https://design.ros2.org/articles/node_lifecycle.html).

This class is intended for direct inheritance to implement custom state-based components that perform 
different behaviors based on their state and on state transitions. An example of state-based behaviour is a 
signal component that requires configuration steps to determine which inputs to register and subsequently should 
publish outputs only when the component is activated. 

Developers should override `on_validate_parameter_callback()` if any parameters are added. In addition, the following
state transition callbacks should be overridden whenever custom transition behavior is needed:
- `on_configure_callback()`
- `on_activate_callback()`
- `on_deactivate_callback()`
- `on_cleanup_callback()`
- `on_shutdown_callback()`
- `on_error_callback()`
- `on_step_callback()`

## Further reading

See the [generated C++ documentation](https://aica-technology.github.io/modulo) for more details.
Refer to the [AICA Component SDK](https://github.com/aica-technology/component-sdk) for usage examples and templates
for creating custom components.