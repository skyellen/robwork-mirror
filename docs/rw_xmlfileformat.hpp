// -*- latex -*-

/**

\page page_xml_workcell_format RW XML File Format

- \ref sec_rwxml_intro
- \ref sec_rwxml_format
- \ref sec_rwxml_elements
	- \ref sec_rwxml_workcell
	- \ref sec_rwxml_device
		- \ref sec_rwxml_serialdevice
		- \ref sec_rwxml_treedevice
		- \ref sec_rwxml_paralleldevice
		- \ref sec_rwxml_mobiledevice
	- \ref sec_rwxml_frame
		- \ref sec_rwxml_joint
		- \ref sec_rwxml_dhjoint
	- \ref sec_rwxml_drawable
	- \ref sec_rwxml_collisionmodel
	- \ref sec_rwxml_property
	- \ref sec_rwxml_polytope	
	- \ref sec_rwxml_transform


\section sec_rwxml_intro Introduction
 The workcell XML file format have suffix \c .xml and follow the rules of standard XML.
\ref sec_rw_manual_load_workcell "Tag workcell files are loaded"
with rw::loaders::WorkCellLoader::load().

\section sec_rwxml_format RW XML Format

 Before going deep into the grammar of the rw xml format some common structure need be 
explained. The complete kinematic description is based on a construct named Frame.
 The Frame has a name and a transform relative to its parent frame or refframe. 
This means that a kinematic 
Frame tree can be described with multiple frames. The Frame can be of different 
types the simplest being Fixed, which means that the frame transform is unchangeable. 

 Another common structure is the Property. The property is linket to a frame and has a 
name, description and a string value. Properties are used to link different 
information to frames. F.ex. collision models and drawables are described using properties.
properties are also used to allow users to attach user specific information to frames.

 In the xml file format Frames are grouped logicly in container type elements. These elements 
are WorkCell and device types. To avoid name clashes frames belonging to a container type will 
have the container name prepended. Example: a frame named "base" specified in a device named 
"PA10" in a workcell named "scene" will have the unique name "scene.PA10.base".

 The WorkCell element is the root element in the fileformat. It implicitly defines a Fixed frame 
named World. This world frame is the root frame in the kinematic frame tree.

workcell
  *(frame
   | Drawable
   | SerialDevice 
   | TreeDevice 
   | ParallelDevice
   | MobileDevice
   | CollisionSetup )


\section sec_rwxml_elements XML Elements

\subsection sec_rwxml_workcell WorkCell

- \b WorkCell

- \b Attributes 
	- \b name: a string identifying the workcell.

- \b Child elements: 
	- \b Device: 
	- \b Frame:
	- \b Drawable
	- \b CollisionModel
	- \b CollisionSetup

\subsection sec_rwxml_device Device

\subsubsection sec_rwxml_serialdevice
\subsubsection sec_rwxml_treedevice
\subsubsection sec_rwxml_paralleldevice
\subsubsection sec_rwxml_mobiledevice
\subsection sec_rwxml_frame
\subsubsection sec_rwxml_joint
\subsubsection sec_rwxml_dhjoint
\subsection sec_rwxml_drawable
\subsection sec_rwxml_collisionmodel
\subsection sec_rwxml_property
\subsection sec_rwxml_polytope	
\subsection sec_rwxml_transform


*/