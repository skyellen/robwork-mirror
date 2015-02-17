package dk.robwork;

import java.util.EventObject;

/**
 * This class allows an event based callback implementation for {@link ThreadSimulator}.
 * 
 * @since 2013-11-22
 * @see ThreadSimulatorStepEventDispatcher
 * @see ThreadSimulatorStepEventListener
 */
public class ThreadSimulatorStepEvent extends EventObject {
	private static final long serialVersionUID = 1L;

	/**
	 * Create a new event.
	 * @param source			the object that created this event.
	 * @param threadsimulator	the {@link ThreadSimulator} that makes the callbacks.
	 * @param state				the new {@link State} after the simulation step.
	 */
	public ThreadSimulatorStepEvent(java.lang.Object source, ThreadSimulator threadsimulator, State state) {
		super(source);
		this.threadsimulator = threadsimulator;
		this.state = state;
	}
	
	/**
	 * @return the {@link ThreadSimulator} that makes the callbacks.
	 */
	public ThreadSimulator getThreadSimulator() {
		return threadsimulator;
	}
	
	/**
	 * @return the {@link State} after the simulation step. 
	 */
	public State getState() {
		return state;
	}

	private final ThreadSimulator threadsimulator;
	private final State state;
}
