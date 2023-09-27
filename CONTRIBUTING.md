# Contributing

This document is still work in progress.

### Exceptions

This section describes how a component should handle exceptions and errors.

As a general rule, a component interface methods do not throw an exception or raise an error unless there is no other
meaningful option. More precisely, only non-void public/protected methods throw, i.e. all setters and `add_xxx` methods 
do not throw but catch all exceptions and log an error.

If an exception is thrown, it is either a `ComponentException` (in C++) or a `ComponentError` (in Python) or any
derived exception, such that all exceptions thrown by a component can be caught with those base exceptions (for example
in the periodic `step` function).

Currently, the following methods throw:
- `get_parameter`
- `get_parameter_value`
- `lookup_transform`

### Logging

Similar to the exceptions, the logging of debug, info, and error messages should follow some principles:

- Methods that catch an exception and are not allow to rethrow, log an error with the exception message
- `add_xxx` methods use non-throttled logging
- Setters and getters as well as all other methods that are expected to be called at a high frequency use throttled
  logging


## Development Environment

Our development and testing workflow uses a Docker container to host the project build dependencies.

The following section describes the configuration steps to use this workflow. Of course, contributors may use whatever
development environment they prefer, as long as they adhere to the overall contribution guidelines.

### Configuring the development environment

Prerequisites: Install Docker and Visual Studio Code.

Step 1: Open VS Code and install the [Dev Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) extension.

Step 2: Use **Dev Containers: Open Folder in Container...** from the command palette and select this directory.

Step 3: The CMake profile should be automatically selected, now you can run and debug library and test targets entirely
with the devcontainer toolchain.