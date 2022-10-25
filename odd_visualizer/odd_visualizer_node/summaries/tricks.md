# Bugs occured in developing
#### 1. The modification of ros params declared through ros launch and .yaml file is only valid after rebuild the package
> **Reason and Solution** ([source](https://answers.ros.org/question/365695/colcon-build-from-any-directory-and-need-to-re-build-for-changes-in-launch-file/))
> When only using `colcon build`, there will be a copy of the package (including the lacunch and yaml file) in the install folder and the launch file can only recognize the path in the install folder. In other word, the node will only invoke the .yaml file in the install folder, which is the original copy without any manual modification in the workspace.
> To let the launch file launch the modyfied .yaml file, use `--symlink-install` after `colcon build`