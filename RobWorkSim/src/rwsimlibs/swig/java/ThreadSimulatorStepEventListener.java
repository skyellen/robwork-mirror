package dk.robwork;

import java.util.EventListener;

/**
 * Interface for listeners that wants to receive step events from {@link ThreadSimulator}.
 * Implement this interface and register an object of this type on the {@link ThreadSimulatorStepEventDispatcher}
 * to be notified with step events.
 * 
 * Please see the
 * <a href="http://www.robwork.dk/apidoc/nightly/rw/page_rw_scriptinterface.html">RobWork Java script interface documentation</a> for a detailed description.
 * 
 * @since 2013-11-22
 * @see ThreadSimulatorStepEventDispatcher
 */
public interface ThreadSimulatorStepEventListener extends EventListener {
	/**
	 * The method that handles received step events from the simulator.
	 * @param event the received event.
	 */
	void stepEvent(ThreadSimulatorStepEvent event);
}
