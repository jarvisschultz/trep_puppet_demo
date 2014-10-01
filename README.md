trep\_puppet\_demo
====================

This [ROS] package is a demo uses [trep] to integrate the dynamics of a humanoid
puppet while a user is directly controlling string endpoints through various
interfaces. A [URDF] of the puppet as well as stl files for each of the joints
is used in conjunction with the [robot state publisher] and [rviz] to animate
the simulation.

#### Simulation breakage

In the trep simulation of the puppet's dynamics, there are several important
things to note. The strings are implemented as constraints between the endpoints
and the attachment points on the model. As such, it is quite easy for the user
to move the endpoints to a geometrically infeasible configuration e.g. there is
no set of puppet joint configurations that allow the constraints to be
satisfied. In this case warnings saying *No solution to DEL equations!* are
printed, and the simulation is automatically reset. As the number of constraints
goes up, for example, through the addition of leg strings, these infeasible
configurations become easier and easier to find.

Another note is that the strings are currently more accurately thought of as
rods. They are fixed length, and actually able to support compressive
forces. This may be changed in the future.

## Interfaces

A description of the interfaces implemented thus far is presented below.

#### Direct joint control

In this mode, there are no dynamics, and the user has the ability to directly
control each of the puppet's joints via the `joint_state_publisher`
GUI. Access with `puppet_vis.launch` launch file.

#### Interactive markers

Use interactive markers to control the string endpoints in [rviz] via the
[interactive\_markers] ROS package. Access with the `puppet_sim.launch` launch
file.

#### Skeleton tracking

This mode uses an OpenNI compatible device along with the NITE skeleton tracking
libraries to allow a user to control the string endpoints via their own hands,
shoulders, knees, etc. This mode requires the [openni\_launch] ROS package, as
well as the [skeletontracker\_nu] and [skeltonmsgs\_nu] packages. These packages
are very similar to the [openni\_tracker] package, but instead of just sending
`/tf` information, they also publish topics with custom messages containing all
of the transforms for each user being tracked. Access this mode via the
`puppet_skeleton_control.launch` launch file.


## Launch file options

`gui` (bool, default:false)
* specify whether or not to start up the `joint_state_publisher` GUI. This is
really only useful in the `puppet_vis.launch` launch file.

`vis` (bool, default:false)
* if true, start `rviz` with a nice configuration for the current mode

`legs` (bool, default:false)
* if true, connect strings to the legs

`shoulders` (bool, default:false)
* if true, connect two independent strings to the shoulders rather than two
strings going to a single control point

`path_len` (int, default:60)
* in the skeleton tracking interface paths are rendered to show the path that
the control has followed, this integer controls how many points should be in
that path

`kinect` (bool, default:true)
* if true, try and run `openni.launch` when running the skeleton interface mode;
may be useful for debugging



License
-------

Copyright (C) 2013 Jarvis Schultz, schultzjarvis@gmail.com

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.


[trep]: http://murpheylab.github.io/trep/
[ROS]: http://www.ros.org/
[URDF]: http://wiki.ros.org/urdf
[robot state publisher]: http://wiki.ros.org/robot_state_publisher
[rviz]: http://wiki.ros.org/rviz
[interactive_markers]: http://wiki.ros.org/interactive_markers
[openni\_launch]: http://www.ros.org/wiki/openni_launch
[openni\_tracker]: http://www.ros.org/wiki/openni_tracker
[skeletontracker\_nu]: https://github.com/jakeware/skeletontracker_nu
[skeltonmsgs\_nu]: https://github.com/jakeware/skeletonmsgs_nu
