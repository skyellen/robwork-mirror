import dk.robwork.*;
import java.util.Date;

public class Simulation {
	static RobWorkStudioPtr rwstudio;
	
	private static class Callback implements ThreadSimulatorStepCallbackHandler {
		long time;
		
		public Callback() {
			time = 0;
		}
		
		@Override
		public void callback(ThreadSimulator simulator, dk.robwork.State state) {
			Date now = new Date();
			if ((now.getTime()-time) > 1000) {
				time = now.getTime();
				rwstudio.setState(state);
				System.out.print("Time: ");
				System.out.println(simulator.getTime());
			}
		}
	}
	
	public static void main(String[] args) {
		// Use this for debugging:
		// System.out.println(System.getProperty("java.library.path"));
		
		// Load
		LoaderRW.load();
		LoaderRWS.load();
		LoaderRWSim.load();
		
		// Try to open RobWorkStudio and load the workcell
		rwstudio = rws.getRobWorkStudioInstance();
		DynamicWorkCellPtr dwc = DynamicWorkCellLoader.load("/home/thomas/Eclipse/LearnBiP/marvin/MarvinCalibratedScene.dwc.xml");
		WorkCellPtr wc = dwc.getWorkcell();
		dk.robwork.State state = wc.getDefaultState();
		rwstudio.postWorkCell(wc);
		
		// Create Simulator
		ContactDetectorPtr cd = new ContactDetectorPtr(new ContactDetector(wc));
		cd.setDefaultStrategies();
		PhysicsEnginePtr simulator = new PhysicsEnginePtr(new ODESimulator(dwc, cd));
		simulator.initPhysics(state);
		DynamicSimulatorPtr dsim = new DynamicSimulatorPtr(new DynamicSimulator(dwc, simulator));
		ThreadSimulatorPtr tsim = new ThreadSimulatorPtr(new ThreadSimulator(dsim, state));
		tsim.setTimeStep(0.001);
		tsim.setRealTimeScale(1);
		
		// Set target configuration for robot
		PDControllerPtr controller = dwc.findPDController("UR1Controller");
		Q qTarget = new Q(6,-1.666,-2.332,-0.413,-1.283,0.793,-0.219);
		controller.setTargetPos(qTarget);
		
		// Create callback
		ThreadSimulatorStepCallbackHandler cb = new Callback();
		ThreadSimulatorStepCallbackEnv fct = new ThreadSimulatorStepCallbackEnv(cb);
		tsim.setStepCallBack(fct);
		
		// Start simulation
		tsim.start();

		try {
			while (true) {
				Thread.sleep(1000);
				if (tsim.isInError())
					System.out.println("Simulation is in error!");
				if (!tsim.isRunning()) {
					System.out.println("Simulator stopped running!");
				}
			}
		} catch (InterruptedException e) {
			rwstudio.postExit();
		}
	}
}