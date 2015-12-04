# rosworkon
Workspace tool for ros

To use, set environment variable ROS_WORKON_ROOT to directory containing workspaces and source rosworkon_init, for example in .bashrc:
export ROS_WORKON_ROOT="~/dev"
source rosworkon_init
 Then use rosworkon command.

rosworkon : lists available workspaces
rosworkon wsname : activates workspace wsname
rosworkon --deactivate : deactivates current workspace
rosworkon --install-deps : runs rosdep install with some switches for the active workspace
(tab completion also works)
