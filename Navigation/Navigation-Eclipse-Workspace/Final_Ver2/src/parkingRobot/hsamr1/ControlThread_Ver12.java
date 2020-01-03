package parkingRobot.hsamr1;

import parkingRobot.IControl;

/**
 * thread started by the 'Control' class for background calculating. For simplification each main module class has its own
 * calculation thread calculate all relevant algorithms independent from the other modules. In case of shared data access with
 * other main module classes synchronized access must be guaranteed.
 * 
 */
public class ControlThread_Ver12 extends Thread {
    
	/**
	 * 
	 */
	IControl control;
	
	
	/**
	 * 
	 * 
	 * @param perception
	 */
	ControlThread_Ver12(IControl control){
    	this.control = control;
    }
	
	
	/* (non-Javadoc)
	 * @see java.lang.Thread#run()
	 */
	@Override
    public void run() {
        while(true){
        	try{
            	control.exec_CTRL_ALGO();
	            Thread.sleep(30);
        	} catch(InterruptedException ie){	        		
        	}
        }
    }	

}
