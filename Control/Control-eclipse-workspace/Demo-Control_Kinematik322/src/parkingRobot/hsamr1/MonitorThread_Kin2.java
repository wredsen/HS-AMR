package parkingRobot.hsamr1;

import parkingRobot.IMonitor;

/**
 * thread started by the 'Monitor' class for background calculating. For simplification each main module class has its own
 * calculation thread calculate all relevant algorithms independent from the other modules. In case of shared data access with
 * other main module classes synchronized access must be guaranteed.
 *
 */
public class MonitorThread_Kin2 extends Thread {
	
	IMonitor monitor;
	
	MonitorThread_Kin2(IMonitor monitor) {
		this.monitor = monitor;
	}
	
	/* (non-Javadoc)
	 * @see java.lang.Thread#run()
	 */
	@Override
    public void run() {
        while(true){
        	try{
            	monitor.run();
	            Thread.sleep(1);
        	} catch(InterruptedException ie){	        		
        	}
        }
    }	

}