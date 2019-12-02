package parkingRobot.hsamr1;

import parkingRobot.IPerception;

/**
 *
 *
 * @author PMP
 */
public class PerceptionThread_Kin2 extends Thread {
    
	/**
	 * 
	 */
	IPerception perception;
	
	
	/**
	 * 
	 * 
	 * @param perception perception object
	 */
	public PerceptionThread_Kin2(IPerception perception){
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
	            Thread.sleep(20);
        	} catch(InterruptedException ie){	        		
        	}
        }
    }	
}