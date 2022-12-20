# CHANGELOG

Release Versions:
- [2.1.1](#211)
- [2.1.0](#210)
- [2.0.0](#200)
- [1.1.0](#110)
- [1.0.0](#100)
- [Pre-release versions](#pre-release-versions)

## Upcoming changes (in development)

- Fix test actions and workflows (#147, #148)
- Use rclpy lifecycle node as base class for lifecycle components (#134, #140, #146)
- Fix creation of triggers and improve tests (#149)
- Do not publish empty states (#129)
- Change order of execution in step callback (#150)

## 2.1.1
### October 28, 2022

Version 2.1.1 is a patch release to update the license of the Modulo project from MIT to GPLv3. A core dependency
of modulo is [control libraries](https://github.com/aica-technology/control-libraries), making it a combined work.
To be compliant with the GLPv3 license of control libraries, modulo must also be licensed under GPLv3.

### Changes

- Change license to GPLv3 (#143)

## 2.1.0
### October 21, 2022

Version 2.1.0 contains several smaller improvements to the Modulo project, such as more consistency with parameter
naming, declaring and removing of signals, and cleaner exception handling and logging. Another new feature is the
ability to provide a user-defined callback for input signals.

There have also been some changes to support both galactic and humble releases of ROS.

### Changes

- Support multiple build test actions (#117)
- Manually export modulo_core dependencies (#118)
- Ensure compatibility with humble (#119)
- Add static tf broadcaster (#120)
- Use prefix for all exceptions (#121)
- Accept list of transforms in send_transform (#122)
- Rename all frame_name parameters to frame (#123)
- Check that the data pointer is not null when adding a signal (#128)
- Rename validate parameter callback and validate period value (#126)
- Add double underscore to private function (#127)
- Improve error messages and logs further (#131)
- User-defined callback for inputs (#124, #132, #133)
- Add humble-devel build and test workflow (#135)
- Correctly list rclcpp_components as build dependency (#137)
- Add option to declare signals (#136)
- Correct default value for user callback on add_input (#138)
- Add option to remove signals (#139)

## 2.0.0
### August 05, 2022

Version 2.0.0 constitutes a major refactor of the Modulo project. It now provides stand-alone classes and helpers
for handling and translating messages and parameters in modulo_core, with new and improved component classes
for dynamic application composition in modulo_components.

The old concepts of Cell and Component have been fully redeveloped into modulo_components::Component and
modulo_components::LifecycleComponent, which provide a simple framework for creating stateless (unmanaged) or
state-based (managed) custom components.

For more information, see the new description of each package in the respective README.md files and view
the full generated documentation on [aica-technology.github.io/modulo](aica-technology.github.io/modulo).

### Changes

- Fix develop after control libraries StateType refactor (#55)
- Fix examples after DS factory refactor (#56)
- Create modulo_components package (#42)
- Modulo translators (#41)
- Add MessagePairInterface and MessagePair including tests (#43)
- Use translators in MessagePair (#44)
- Add EncodedState support (#45)
- Add basic methods and attributes to component interface (#46)
- Add tests for the predicate interface (#48)
- Add publisher interface and handler (#47)
- Fix modulo_core after StateType refactor (#49)
- Add the base ComponentInterface class in python (#51)
- Specialize all available MessagePairs and add read capability (#52)
- Implementation of publisher interface (#53)
- Add parameter translators (#54)
- Use galactic-devel base tag (#59)
- Parameter translation improvements (#57)
- Support write/read for MessagePair<EncodedState, State> (#60)
- Component Parameters (#58)
- Component parameter description and improvements (#61)
- Update predicate logic in ComponentInterface (#50)
- Add methods to add output/publishers (#62)
- Add docstrings and QoS getter/setter (#63)
- Parameter translators in python (#65)
- Subscription handler/interface and add_input (#64)
- Implement execution thread for component (#68)
- Improve test coverage in modulo new core (#66)
- Add option to add deamon and update step function (#67)
- Clean up python message translators (#69)
- Add python component with execution thread (#70)
- Minor improvements (#72)
- Improve components tests (#71)
- Parameters in python ComponentInterface (#73)
- Handle logging and exceptions consistently (#74)
- Build and test packages in CI (#76)
- TF improvements (#77)
- Lifecycle transitions for LifecycleComponent (#75)
- Refactor 'msg' to 'message' for readability (#79)
- Extend documentation and consistent formatting (#80)
- Fix MessagePair to enable creating from any SR type (#82)
- Add QoS and periodic callbacks to Python ComponentInterface (#81)
- Make sure predicate is a bool (#85)
- Inherit from NodeT with public keyword (#84)
- Add inputs and outputs to python component interface (#83)
- Translation of empty parameters (#86)
- Add fallback name to constructors (#87)
- Add lifecycle component in Python (#89)
- Rename transition handlers and callbacks (#90)
- Remove old modulo_core and rename modulo_new_core (#91)
- Improve exception handling in modulo_core (#94)
- Don't run image build on push on develop (#95)
- Improve parameter translators for NOT_SET cases (#96)
- Don't start execute thread on construction (#98)
- Execute user transition callback first (#100)
- Handle empty parameters in ComponentInterface (#97)
- Minor improvements (#101)
- Improved exception handling (#99)
- Add trigger and send predicate on set (#102)
- Component services (#103)
- Create service under local namespace (#104)
- Add service improvements (#105)
- Generate documentation (#106)
- Rename test directories (#107)
- Modulo Core documentation (#109)
- Improve State message reading (#108)
- Catch core and component exceptions (#111)
- Modulo components documentation (#112)
- Better versioning and changelog (#113)
- Fix const reading of State parameters (#114)

## 1.1.0
### February 18, 2022

This version contains some general improvements and fixes, including support
for the breaking changes introduced by control libraries 5.0

- Update docker paradigm (#30)
- Refactor the shutdown logic (#31)
- Modify build script for production by default (#32)
- Change the underlying message type to UInt8MultiArray (#33)
- Support int and int array parameters (#34)
- Remove the pure virtual step function (#35)
- Fix the examples due to breaking changes in DS (#36)
- Correct the include due to changes in control-libraries (#38)

## 1.0.0
### September 22, 2021

Modulo core library to provide ROS2 Cell and Component classes built
around state_representation and clproto libraries.

---

# Pre-release versions

## 0.3.0 (November 18, 2020)

Latest version that fixes different bugs and add the possibility to isolate the containers to run them on dedicated network. Complete list of commits is:

  - Add network isolation at runtime
  - Merge branch 'develop' of github.com:epfl-lasa/modulo into develop
  - visual indentation
  - Merge branch 'develop' of github.com:epfl-lasa/modulo into develop
  - bug in DynamicalSystem.cpp
  - Correct the error of logic
  - More efficient computation
  - Changes the order of the attractor
  - Ignore build files
  - Add missing googletest dependency
  - Add funtions to clamp the joint velocities
  - Correct the test that was failing due to an exception
  - Correct the problem of the reference frame if the state is already given in the correct frame
  - Merge branch 'develop' of github.com:epfl-lasa/modulo into develop
  - Add a security checking in case one of the value is not provided
  - Changes buils script to adapt to base version
  - Base the image on ros2_ws image
  - Fixes bug in circular.hpp
  - Remove non developed library
  - Build with the correct path and remove the building of the python version
  - Move everything back to the root directory
  - Remove old dependency to zmq
  - Put an argument to the distance function to specify the dimensions needed (default all)
  - Correct the formula regarding angular velocity and acceleration
  - Add the reference frame from the attractor
  - Test the moving of reference frame of dynamical systems
  - Move the operator* to the state base class
  - Remove old parameter
  - Remove the virtual pure in the step function
  - Add the possibility to braodcast a transform from a Parameter<CartesianPose> pointer
  - Correct the changing of reference for the full state
  - Add a get_reference_frame function
  - Remove the reference frame checking at message reading
  - Add multiple definitions of add_transform_broadcaster and possibility to always publish
  - Add Identity cartesian pose
  - Move instantiation to cpp file
  - Correct an error that was creating a result with wrong joint names
  - Remove parameter from the doc
  - Rename JacobianMatrix to just Jacobian
  - Changes the way of handling the quaternion hemishpere problem
  - Ensure that all quaternions are on the same hemisphere
  - Add a random changing attractor when previous one is achieved
  - Add quaternion hemishpere operation to all operations
  - Remove all graphics options to leave it to specific images
  - Correct the template deduction error by correctly removing DurationT
  - Add missing tag
  - CHange the base image to foxy LTS
  - Correct errors and change name
  - Separate the rotation from the orientation
  - Merge branch 'develop' of github.com:epfl-lasa/modulo into develop
  - Changes the way the ds is calculated
  - Merge branch 'develop' of github.com:epfl-lasa/modulo into develop
  - Added function for setting and getting rotation angle for circular DS
  - Remove non ros libraries
  - change to c++17
  - CHange to ros_distro
  - Add missing assignment operators that were generating warnings
  - Create contrsuctor with not timeout and/or no recipients
  - Remove the timeout from publihser and transform broadcaster
  - Divide by the radius to get a constant velocity
  - Correct problems with the circular dynamical system
  - Correct error with the velocity not expressed in the correct frame
  - changes limit_circle to limit_cycle
  - Remove the testing on the center name
  - Add setter and getters for the limit circle
  - Add cpp source for the circular ds that is not template anymore
  - Handle ellipsoid parameters
  - Add possibility to declare an ellipsoid parameter
  - Add missing include
  - add from and to std vector converision
  - Uses the ellipsoid as limit circle of the dynamical system
  - set filled for publication
  - Add missing inline keyword
  - add missing include
  - send the ellipsoid as float64 array
  - Store the rotation angle and calculate orientation accordingly
  - Add Geometrical states with Ellipses first
  - Replace the scalar gain by a matrix gain
  - Add the possibility to set and get matrix based parameters
  - Add the operator for multiplication with an Eigen matrix
  - Add the possibility to define parameters of eigen matrix
  - Clean trajectory
  - Add set_zero functions
  - Add an Event class to represent predicate wiht memory
  - Add the possibility to set the parameter from command with string and value
  - Add an exception when looking for a unexistent parameter
  - Clean the headers to have pragma once
  - Add functions to set the parameter on the parameter server
  - add gdb
  - Add functions to set the parameter value from the node
  - Correct the error in the set_center function
  - Correct the documentation
  - Add Predicate as a Parameter<bool>
  - Change to const functions using at for the map and change static_cast to static_pointer_cast
  - Change to const functions
  - Update with the new syntax using nvidia-container-toolkit
  - Add the possibility to rename the transform before publishing it

## 0.2.2 (mai 08, 2020)
  - Merge branch 'develop'
  - Correct all the problems of Circular and some linking errors that could arise
  - Add -j argument to speed up building

## 0.2.1 (mai 07, 2020)
  - Correct synchronization with mutex

## 0.2.0 (mai 07, 2020)
  - Remove unnecessary break
  - Remove scale function
  - Merge branch 'develop'
  - Add parameter test
  - Correct name for coherence with Cartesian example
  - Add JointPositions parameter get and remove CartesianState and JointState parameter handlers
  - Add from_std_vector function for JointPositions
  - Correct parameter initilaization in JointSpace
  - Catch exceptions that arise during parameter set
  - Clean dynamical system library to handle properly parameters
  - Clean Cell package to remove unnecessary files
  - Add conversion from std vector to set position and orientation
  - Clean header files
  - Propagate changes made to the parameter conversion
  - Add parameters to the dynamical systems
  - Handle the parameters and conversions between them
  - Test the setting of a pose parameter
  - Add from_std_vector function
  - Remove trailing space
  - Use a proxy parameter interface to be able to add a list of parameters
  - Add the possibility to prefix the parameter name
  - Simplify the transform braodcasting by having a single transform braodcaster and improve readability
  - Properly handle the template specialization of the add_parameter function
  - Add the to_std_vector() function
  - Move the parameter handling to the constructor instead of the configure
  - Handle parameters and communication with the parameter server in background
  - Parameter update add all parameter types
  - Parameters update: Add different types and derive constructors
  - Clean unnecessary files
  - Correct example due to changes
  - Tentative to add parameter handlers
  - Remove the shared_ptr calls and correct the float issue
  - Clean the library and add a Blending dynamical system
  - Split StateConversion in WriteStateConversion and ReadStateConversion
  - Remove the lock from the on_X functions
  - Send CartesianState instead of Pose in the send transforms
  - Add test std vector
  - Add random constructor
  - Specify that it overides
  - Add to std_vector as a virtual function of the base class
  - Add a not implemented exception
  - Slowly move to #pragma once
  - Remove unused example
  - Clean by removing Actions and SML that are to be included in the state machine
  - Add override keyword
  - Add conversion from and to std vector for the pose
  - Push googletest updates
  - Push googletest updates
  - Remove sml from the base library
  - Merge branch 'develop' of github.com:epfl-lasa/modulo into develop
  - Corrected time_from_start when adding point in trajectory
  - Passes the active, configured and shutdown boolean as public instead of protected
  - Remove parameters from the Cell
  - Test for state machine with better implementation
  - Test for state machine (Problem of warning with inconsistent states)
  - Changes related to polishing demo
  - Correct buggy tests
  - Include SML in the installation process
  - Add guard functions for SML integration
  - Rearrange the calls to avoid having to reset in case of errors
  - Install rviz2 that is not included (anymore?)
  - Move the dist function to state instead of pose
  - Remove controller and sensor_interface. They are now included in hardware_interface package
  - Major change: Add possibilities to configure type functions to fail
  - implementation and update python library
  - add dynamical system functions
  - Update circular ds
  - Merge branch 'master' of github.com:epfl-lasa/modulo
  - Build on top of eloquent
  - Add software property commons to be able to use ppa
  - Add write_msgs functions with new msg types
  - Merge branch 'master' into develop
  - Minor changes
  - Remove the RobotInterface to build it in a different package
  - Merge branch 'master' of github.com:epfl-lasa/modulo
  - Added functions to Trajectory class
  - Add missing eigen3 dependency that was downloaded by default in ROS2 image
  - Merge branch 'master' of github.com:epfl-lasa/modulo
  - Completed Trajectory class and implemented corresponding unitary tests
  - Merge branch 'master' into develop
  - Remove include
  - Merge branch 'hotfix-0.1.3' into develop
  - Merge branch 'hotfix-0.1.2' into develop
  - Merge branch 'release-0.1.2' into develop
  - Merge branch 'master' of github.com:epfl-lasa/modulo
  - Added scaling

## 0.1.4 (febbraio 26, 2020)
  - Merge branch 'master' of github.com:epfl-lasa/modulo
  - Testing of Trajectory (publish JointTrajectory message)

## 0.1.3 (febbraio 26, 2020)
  - Merge branch 'hotfix-0.1.2'
  - Forgot to remove the file from the CMakeLists.txt

## 0.1.2 (febbraio 26, 2020)


## 0.1.1 (febbraio 26, 2020)


## 0.1.0 (febbraio 26, 2020)


## 0.1.0 (febbraio 26, 2020)
  - Remove the robot interface for testing purpose
  - Remove lifecycle client as nodes already have the possibility to call lifecycle functions internally
  - Simplify Dockerfile
  - Merge branch 'master' of github.com:epfl-lasa/modulo
  - Add publisher for trajectory point
  - Add representation for trajectories
  - Change back to SingleThreadedExecutor
  - Remove all the milliseconds dependencies
  - Improve documentation
  - Declare the timers as template to accept any type std::chrono::duration
  - Put back the wokdir in home directory after build
  - Respect naming convention
  - Simplify the demo
  - \Merge branch 'master' of github.com:epfl-lasa/modulo
  - Propagate a CommunicationTimeoutException and catch it properly in the monitor
  - Add a default client to the lifecycle change state service of the node
  - Increase timeout even further
  - Put client in a ServiceClient subfolder and namespace
  - Increase timeout period
  - Reorganize Communication folder into subcategories
  - Correct the problems with the monitor by waiting that the service is available
  - Add warning message
  - Merge branch 'master' of github.com:epfl-lasa/modulo
  - Initialize with 0 position
  - Output should be a Twist not a Pose
  - Simplify the conversion
  - Merge branch 'master' of github.com:epfl-lasa/modulo
  - Car demo
  - Randomize orientation as well and publish attractor at a different rate than the node period
  - Add possibility to set a pose from position and orientation
  - Changes the way to specify the publishing period in publishers
  - Add missing docstring
  - Merge branch 'master' of github.com:epfl-lasa/modulo
  - Better handle the client side with blocking request
  - Change order of arguments on Monitor
  - First node that uses a client to get the status of the nodes
  - First attempt to define a client communication paradigm
  - Clean the unused variables
  - Add a base class between CommunicationHandler and the other ones to remove the recipient
  - Add missing docstring
  - Improve the action example
  - Add a random target generator
  - Add an action class
  - Add AngularVelocity
  - Add the angle representation
  - Clean and add the negative operator
  - Define Velocity as a template class with its own literals
  - First implementation of a unit class using custom literals
  - Clean the Communication folder to have proper conventions and docstring
  - Change the type string to an enumeration for later switch case usage
  - Properly clean the main thread on cleanup
  - Merge branch 'master' of github.com:epfl-lasa/modulo
  - Add methods to add periodic call of a function
  - Merge branch 'master' of github.com:epfl-lasa/modulo
  - Change docker tag due to errors in the nightly build
  - Merge branch 'master' of github.com:epfl-lasa/modulo
  - Updated circulard DS
  - Add Parameter representation to be able to send and receive standard types
  - Reorganize StateRepresentation to integrate Parameters
  - First attempt for circular DS on IIWA
  - Copy and apply ownership in single line
  - Remove nvidia runtime
  - Correct typos and add missing doc
  - Remove garbage in files
  - resolve typo
  - resolve conflict
  - merge files
  - create files
  - create all files
  - Merge branch 'master' of github.com:epfl-lasa/modulo
  - Scale the quaternion based on the scaled angular velocity from identity
  - Put the quaternion log and ecp into functions in a file named MathTools
  - Add missing memory import
  - Merge branch 'master' of github.com:epfl-lasa/modulo
  - add python files
  - First attempt to use a launchfile but not working correctly
  - Test shared_ptr acceptation and synchronus send_transform
  - Add a way to publish synchronusly. This is used to perform multiple send_transform
  - Accept shared_ptr as input
  - Accept shared_ptr as input
  - Quaternion addition is not commutative
  - Add rm option in container
  - Merge branch 'master' of github.com:epfl-lasa/modulo
  - Check the multiplication sign after the operation
  - Merge branch 'master' of github.com:epfl-lasa/modulo
  - add clamping in JointTorques
  - Add tested joint acclerations and torques + reorganize test folder
  - Add tested joint positions and velocities
  - Change printing function
  - Remove trailing whitespace
  - Add tested joint state
  - Add name in description
  - Remove unused jacobian matrix
  - Test python implementation of State
  - Reorganize to ease the mount of volumes
  - Add python based StateRepresentation of State
  - Reorganize StateConversion for clarity
  - Change writing of State
  - Remove protobuf
  - Remove protocol buffers
  - Put a cpp folder to include a python version of the lib
  - Merge branch 'master' of github.com:epfl-lasa/modulo
  - Remove unnecessary script
  - Update README.md
  - Reorganize folders architecture
  - Correct typos and test filtering
  - Do not check for name equality when performing the operations only for reference frames
  - Add function to record a joint state
  - Proper implementation of the recorder to only record subscriptions
  - Merge branch 'master' of github.com:epfl-lasa/modulo
  - Merge branch 'master' of github.com:epfl-lasa/modulo
  - The step function is not virtual here
  - Add the Recorder subclass
  - Use double quotes
  - Catch by reference
  - Add the badge to the code verification using codacy
  - Improve coding style
  - Add an exception to raise when a controller is not supported
  - Better style formatting
  - add a dockerignore file
  - Simplify the image by using WORKDIR
  - Merge branch 'master' of github.com:epfl-lasa/modulo
  - Add functions for parameters (future) and boolean for activation, configure and shutdown
  - Merge branch 'master' of github.com:epfl-lasa/modulo
  - Add noise ratio for both angular and linear velocities
  - remove the stop container at exit
  - Modify the way to calculate the displacement
  - Very important here to multiply by the gain after calculating a velocity
  - Correctly calculate the angular velocity (tested on the franka robot)
  - Create the base  functions to handle parameters using ROS2 paramter system
  - Clean the Cell header
  - Put the subscribers/publishers in on_configure functions and validate the lifecycle process
  - Add CartesianWrench and JointTorques representations
  - Add operations + and - with Eigen Vectors
  - Refactor CartesianVelocity to CartesianTwist
  - Also pass the joint names in the JacobianMatrix message
  - Add placeholders for velocity clamping in joint space
  - Build missing modulator
  - Add a function to access the mutex
  - Add the jacobian and change topic names
  - Remove unnecessary mutex lock and pass its shared_ptr by const reference
  - Add possibility to initialize a jacobian without knowing the number of joints
  - Add a copy function based on the copy constructor
  - Put the default timeout ot 10*period and set it from an attribute to the function
  - Correct the mutex problem that was causing the publisher to randomly crash
  - Set dt as the time period of the node
  - Correct a missing factor argument. Now position and orientation converges at the same rate
  - Add exception to be thrown in not yet implemented functions
  - Test the jacobian publication and correct errors in StateConversion
  - Simplify the name of tests
  - JacobianMatrix implementation tested
  - Major restructuration
  - Add a class to represent the jacbobian matrix of a robot
  - Put the initialization of the base class that set empty to false
  - Add twist setter and a constructor from a twist
  - Add JointPositions and JointVelocities (not tested)
  - Add distance calculations
  - Add a noise ration to apply a deadzone under which the velocity is set to 0
  - Add clamping functions for velocity both in place and with return value
  - The quaternion error was incorrect resulting in an angular velocity of inversed sign
  - Coorect conditions in orientation for the tests
  - Correct the problem of non convergence of orientation that was due to incorrect ordering
  - Solve the constant inversion problem but make the DS test fail
  - Not sending the robot transform was causing problem for the test
  - Simplify calculation of the log of the quaternion error
  - always normalize and change the way to check if quaternions are on the same hemisphere
  - Adapt to changes made in state_representation and simplify
  - Add implicit conversion between pose and velocities by using operator over period of 1 second
  - Correct mounting volume
  - Changes the operators to have operations that have physical meaning
  - Always normalize
  - Add googletests dependencies
  - Add JointState example
  - Simplify by only considering a singe float gain for the moment
  - Add NotImplementedException
  - Add missing return statement
  - Add missing empty constructor
  - Change h header
  - Add operator to be able to do commutative operations with a scalar
  - Merge branch 'master' of github.com:epfl-lasa/modulo
  - Add a publisher for Quaternion messages
  - Update README.md
  - Correct missing path in cmake
  - Add the possibility to build modulo on top of specific version of ROS2
  - Add c++ highlightning
  - Update README.md
  - Update README.md
  - Update README.md
  - Update README.md
  - Update README.md
  - Add README
  - Modify the README
  - Add the software architechture at its current stage of development
  - Initial commit
