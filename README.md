# pose_kit

Collection of C++ libraries based on [`Eigen`](http://eigen.tuxfamily.org/index.php?title=Main_Page) for storing and operating on the state of a rigid body in 3D space, for fast and efficient computations.

## Contents

Three classes:

- `Pose`: position and orientation.
- `KinematicPose`: position, orientation and linear/angular velocity.
- `DynamicPose`: position, orientation, linear/angular velocity and linear/angular acceleration.

### Features

All classes are based on the `double` numeric type, and offer:

- Various constructors for different use cases and data availability.
- Representation of attitude as both Euler angles and quaternions.
- Full compatibility with fixed-size Eigen geometric types.
- Getters and setters.
- Assigment operators, copy and move constructors.
- Compatibility with ROS 2 messages from standard packages and the `tf2` library.
- Integration with the `ament` build system.

### Geometric operations

Currently implemented and planned:

- [x] Composition with a ROS `tf` transform to express a pose in a *parent frame* with data coming from a pose expressed in a *child frame*.

## Requirements

Builds on ROS 2 Humble Hawksbill with Eigen 3.4.0 or higher.

Requires `tf2` libraries.

See [`package.xml`](package.xml) for more information.

---

## Copyright and License

Copyright 2024 dotX Automation s.r.l.

Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with the License.

You may obtain a copy of the License at <http://www.apache.org/licenses/LICENSE-2.0>.

Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.

See the License for the specific language governing permissions and limitations under the License.
