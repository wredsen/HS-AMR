package parkingRobot.per_encoder_sample;

import parkingRobot.IPerception;

/**
 *
 *
 * @author PMP
 */
public class PerceptionThread extends Thread {
    
	/**
	 * 
	 */
	IPerception perception;
	
	
	/**
	 * 
	 * 
	 * @param perception perception object
	 */
	public PerceptionThread(IPerception perception){
    	this.perception = perception;
    }
	
	
	/* (non-Javadoc)
	 * @see java.lang.Thread#run()
	 */
	@Override
    public void run() {
        while(true){
        	try{
            	perception.updateSensors();
	            Thread.sleep(10);
        	} catch(InterruptedException ie){	        		
        	}
        }
    }	
}