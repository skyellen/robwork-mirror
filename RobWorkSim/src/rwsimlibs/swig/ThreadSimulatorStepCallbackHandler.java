package dk.robwork;

import dk.robwork.ThreadSimulatorStepCallbackEnv;

/**
 * Interface for a callback from {@link ThreadSimulator}.
 * A class implementing this interface can be wrapped by {@link ThreadSimulatorStepCallbackEnv}
 * which can then be added to a {@link ThreadSimulator}. The {@link ThreadSimulatorStepCallbackEnv}
 * transparently adds Java context information required for native code to call the implemented callback.
 * 
 * Please see the
 * <a href="http://www.robwork.dk/apidoc/nightly/rw/page_rw_scriptinterface.html">RobWork Java script interface documentation</a> for a detailed description.
 * 
 * @since 2013-11-22
 * @see ThreadSimulatorStepCallbackEnv
 * @see ThreadSimulator#setStepCallBack(ThreadSimulatorStepCallbackEnv)
 */
public interface ThreadSimulatorStepCallbackHandler {
	/**
	 * The callback method that is called from the simulator.
	 * 
	 * @param simulator	the {@link ThreadSimulator} that makes the callback.
	 * @param state		the new {@link State} after the simulation cycle.
	 */
	public void callback(ThreadSimulator simulator, State state);
}