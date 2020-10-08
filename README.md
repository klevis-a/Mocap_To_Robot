This (mainly) Matlab based software package provides an algorithmic pipeline for recreating motion capture kinematics on an industrial robot. The supporting library for this package can be found at [Motion Capture to Robot Library](https://github.com/klevis-a/Mocap_To_Robot_Library). Two additional C++ based algorithms ([Gradient Based Trajectory Optimization](https://github.com/klevis-a/Trajectory_Optimization_Gradient_Based) and [Derivative Free Trajectory Optimization](https://github.com/klevis-a/Trajectory_Optimization_Derivative_Free)) support this package by providing the ability to optimally map a motion capture trajectory to the robot's joint space. All algorithms included in this and supporting repositories are described in a manuscript which is currently being peer-reviewed. The manuscript provides details on the appropriate use of each algorithm and a link to it will be added here when published. The user manual for this package is attached to the release (and could be different with each release). Example configuration files for the package are also attached to the release.

The choice to segment code between four repositories was based upon predicted use case scenarios, and the reasons are outlined in the manuscript and user manual. Briefly, the optimization algorithms can be used as standalone applications to optimally map a motion capture trajectory to the robot joint space without additional pre/post processing of the underlying data. The [Gradient Based Trajectory Optimization](https://github.com/klevis-a/Trajectory_Optimization_Gradient_Based) relies on the proprietary package [SNOPT](https://web.stanford.edu/group/SOL/snopt.htm) and is utilized in relatively few scenarios. To simplify the installation process, the [Gradient Based Trajectory Optimization](https://github.com/klevis-a/Trajectory_Optimization_Gradient_Based) and the [Derivative Free Trajectory Optimization](https://github.com/klevis-a/Trajectory_Optimization_Derivative_Free) (which does not rely on proprietary packages) were separated. A concerted effort was made to maintain all algorithmic logic in the [Motion Capture to Robot Library](https://github.com/klevis-a/Mocap_To_Robot_Library). The code in this repository acts a thin wrapper over the [Motion Capture to Robot Library](https://github.com/klevis-a/Mocap_To_Robot_Library) to perform input/output operations while relagating logic to the underlying library. Since other users will very likely have different input/output formats and pre/post processing scenarios, this separation affords them the ability to easily reuse code from the [Motion Capture to Robot Library](https://github.com/klevis-a/Mocap_To_Robot_Library).

The corresponding Zenodo archive (see below) provides a link to the dataset that was processed via this package. The dataset includes motion capture trajectories that were replicated on a M20-iA industrial robot, and the optical motion capture data that verify proper replication.

[![DOI](https://zenodo.org/badge/239798210.svg)](https://zenodo.org/badge/latestdoi/239798210)