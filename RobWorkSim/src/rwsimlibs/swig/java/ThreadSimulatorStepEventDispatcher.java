package dk.robwork;

import java.util.EventListener;
import java.util.Vector;

/**
 * Register listeners for step events from the {@link ThreadSimulator}.
 * When this object is added to the {@link ThreadSimulator}, it will dispatch a {@link ThreadSimulatorStepEvent}
 * for each callback from the stepper loop of the simulator.
 * All registered {@link ThreadSimulatorStepEventListener}s will be notified.
 * The dispatcher will first dispatch events when it is added to the {@link ThreadSimulator} using the
 * setStepCallBack method.
 * 
 * @since 2013-11-22
 * @see ThreadSimulator#setStepCallBack(ThreadSimulatorStepCallbackEnv)
 */
public class ThreadSimulatorStepEventDispatcher implements ThreadSimulatorStepCallbackHandler {
	
	/**
	 * Construct new dispatcher.
	 */
	public ThreadSimulatorStepEventDispatcher() {
		listeners = new Vector<ThreadSimulatorStepEventListener>();
	}
	
	/**
	 * Add a listener that will be notified of new events.
	 * @param listener the listener to add.
	 */
    public synchronized void addThreadSimulatorStepEventListener(ThreadSimulatorStepEventListener listener) {
    	listeners.addElement(listener);
    }
    
    /**
     * Remove a registered listener that will then receive no more events.
	 * @param listener the listener to remove.
     */
    public synchronized void removeThreadSimulatorStepEventListener(ThreadSimulatorStepEventListener listener) {
    	listeners.removeElement(listener);
    }
    
    /**
     * Get the number of listeners currently registered.
     * @return the number of listeners.
     */
    public synchronized int getNumberOfListeners() {
    	return listeners.size();
    }
    
    /**
     * @see ThreadSimulatorStepCallbackHandler#callback
     */
	@SuppressWarnings("unchecked")
	@Override
	public void callback(ThreadSimulator simulator, State state) {
        Vector<ThreadSimulatorStepEventListener> listenersCopy;
        synchronized(this) {
        	listenersCopy = (Vector<ThreadSimulatorStepEventListener>)listeners.clone();
        }
        for (int i=0; i<listenersCopy .size(); i++) {
        	ThreadSimulatorStepEvent event = new ThreadSimulatorStepEvent(this, simulator, state);
            listenersCopy.elementAt(i).stepEvent(event);
        }
	}
	
	private Vector<ThreadSimulatorStepEventListener> listeners;
}
