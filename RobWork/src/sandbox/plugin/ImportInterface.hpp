

/**
 * @brief defines an interface for importing user defined functionality
 * into RobWork. Objects that can be imported will can be of arbitrary type.
 *  There is two usage patterns of this interface:
	 1. let your implementation inherit from a RobWork type, e.g. Device, 
		CollisionDetector and so on. This will enable one to use RobWork 
		functionality on these objects without further work.
	 2. implement your own user type. To use this with RobWork functionality
		you need to handle the casting to your own type again.

	An example of usage of case 2 is:
		You would like the WorkCell loader to parse your userdefined object at the
		same time it parses a WorkCell file. This will enable specifying user properties 
		directly in the workcell file and loading them into your own specific object. So
		create a rwplugin inheriting from ImportInterface and make it available to the 
		WorkCellParser.
		In your application you can request for the imported object and cast it too the 
		specific object type.
 */
class ImportInterface {
public:
	typedef enum {WorkCell, Device, CollisionDetector, UserDefined} ImportType;
	
	virtual ImportType getType() = 0;

	void *import( ,PropertyMap );

}