import dk.robwork.*;

public class State_Path_1 {
	public static void main(String[] args) throws InterruptedException{
		// Use this for debugging:
		// System.out.println(System.getProperty("java.library.path"));
		
		// Load
		LoaderRW.load();
		LoaderRWS.load();
		
		// Try to open RobWorkStudio and load the workcell
		RobWorkStudioPtr rwstudio = rws.getRobWorkStudioInstance();
		WorkCellPtr wc = WorkCellFactory.load("/home/thomas/Hentede filer/rob_scenes/PA10WorkCell/Scene.wc.xml");
		rwstudio.postWorkCell(wc);
		Thread.sleep(2000);
		
		// Find the device and relevant frames
		DevicePtr arm = wc.findDevice("PA10");
		Frame gripper = wc.findFrame("Tool");
		Frame object = wc.findFrame("Bottle");
		Frame table = wc.findFrame("Table");
		
		// Get the current state from RobWorkStudio
		State state = rwstudio.getState();
		Q q = arm.getQ(state);
		System.out.print("Example of printing vector Q: ");
		System.out.println(q);
		
		// Create PathTimedState
		PathTimedStatePtr path = new PathTimedStatePtr(new PathTimedState());
		path.clear();
		
		// Add configurations
		addQ(path, 1, toQ(0,0.1,0,0,0,0,0), arm, state);
		addQ(path, 1, toQ(0,0.1,0,0,0,0,0), arm, state);
		addQ(path, 2, toQ(0,0.2,0,0,0,0,0), arm, state);
		addQ(path, 3, toQ(0,0.3,0,0,0,0,0), arm, state);
		addQ(path, 4, toQ(0,0.4,0,0,0,0,0), arm, state);
		addQ(path, 5, toQ(0,0.5,0,0,0,0,0), arm, state);
		addQ(path, 6, toQ(0,0.6,0,0,0,0,0), arm, state);

		// Grasp the object
		graspObject(path, object, gripper, state);

		// Add configurations
		addQ(path, 8, toQ(0,0.7,0,0,0,0,0), arm, state);
		addQ(path, 9, toQ(0,0.6,0,0,0,0,0), arm, state);
		addQ(path, 10, toQ(0,0.5,0,0,0,0,0), arm, state);
		addQ(path, 11, toQ(0,0.4,0,0,0,0,0), arm, state);
		addQ(path, 12, toQ(0,0.3,0,0,0,0,0), arm, state);
		addQ(path, 13, toQ(0,0.2,0,0,0,0,0), arm, state);
		addQ(path, 14, toQ(0,0.1,0,0,0,0,0), arm, state);
		
		// Set object on table
		graspObject( path, object, table, state );

		// Add configurations
		addQ(path, 15, toQ(0,0.1,0,0,0,0,0), arm, state);
		addQ(path, 16, toQ(0,0.1,0,0,0,0,0), arm, state);
		addQ(path, 18, toQ(0,0.2,0,0,0,0,0), arm, state);
		addQ(path, 19, toQ(0,0.3,0,0,0,0,0), arm, state);

		// Set path in RobWorkStudio
		// Use the PlayBack module to execute
		rwstudio.setTimedStatePath(path);
		Thread.sleep(60000);
		
		rwstudio.postExit();
		Thread.sleep(2000);
	}

	private static void addQ(PathTimedStatePtr path, double time, Q q, DevicePtr device, State state) {
		device.setQ(q, state);
		TimedState tstate = new TimedState(time, state);
		path.add(tstate);
	}
	
	private static void graspObject(PathTimedStatePtr path, Frame object, Frame tool, State state) {
	    rw.gripFrame(object, tool, state);
	    double time = path.elem(path.size()-1).getTime();
		TimedState tstate = new TimedState(time, state);
	    path.add(tstate);
	}
	
	private static Q toQ(double ... numbers) {
		return new Q(numbers.length,numbers);
	}
	
	static int sum (int ... numbers)
	{
	   int total = 0;
	   for (int i = 0; i < numbers.length; i++)
	        total += numbers [i];
	   return total;
	}
}