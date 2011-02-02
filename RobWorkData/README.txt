
There are several file formats in RobWork and many of these
are based on xml. To indicate the file type using the file name 
we recommend the following filename conventions:

<file>.wc.xml	: Workcell definition file (this can also be device definitions)
<file>.dwc.xml	: Dynamic workcell definition file
<file>.prox.xml : Proximity setup file
<file>.prop.xml : Property definition file
<file>.traj.xml : Trajectory definition file
<file>.task.xml : Task definition file

If both fine grained and coarse grained files of the same device or workcell is added
then the filename should also include _c (coarse), _m (medium), _f (fine)



All files are ordered in the following directories

scenes 		: Complete scene definitions
devices 	: Specific device definitions
objects		: various objects and their CAD models
tasks		: diffrent task definitions
scripts		: example scripts
properties	: database of different object properties (material, friction)