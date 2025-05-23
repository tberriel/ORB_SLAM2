Updated from ORB-SLAM2 custom g2o to  g2o - 20170730 to fix Eigen3.3 compatibility. 
All files included in this g2o version are BSD, see doc/license-bsd.txt

See the oficial g2o - 20170730 repository here : https://github.com/RainerKuemmerle/g2o/tree/20170730_git 
The main changes to the official repo are:

Implemented rmur changes from ORB-SLAM2 to:
* g2o/types/sba/types_six_dof_expmap.h
* g2o/types/sba/types_six_dof_expmap.cpp
* g2o/types/sim3/types_seven_dof_expmap.h
* g2o/types/sim3/types_seven_dof_expmap.cpp

Removed unused code:
* g2o/apps
* g2o/examples
* g2o/stuff/opengl_*
* g2o/solvers/
	- cholmod
	- csparse
	- pcg
	- slam2d_linear
	- structure_only
* g2o/types/
	- data
	- deprecated
	- icp
	- slam2d
	- slam*_addons

Removed unrequired dependencies:
* CSparse
* Cholmod
* QGLViewer
* OpenGL

Update CMakeLists.