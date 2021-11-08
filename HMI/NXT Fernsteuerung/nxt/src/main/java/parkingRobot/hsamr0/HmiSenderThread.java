package parkingRobot.hsamr0;

import java.io.IOException;

import parkingRobot.IMonitor;
import parkingRobot.INavigation.ParkingSlot;
import parkingRobot.hsamr0.GuidanceAT;
import parkingRobot.hsamr0.HmiPLT.Command;
import lejos.nxt.comm.RConsole;
import lejos.robotics.navigation.Pose;

/**
 * Thread for remote communication via Bluetooth. Hosts the HMI module of class {@code HmiPlt}, from which it was started. For 
 * simplification each main module class has its own calculation thread calculate all relevant algorithms independent from the 
 * other modules. In case of shared data access with other main module classes synchronized access must be guaranteed.
 * @author PLT
 *
 */
public class HmiSenderThread extends Thread{

	HmiPLT hmi;
	IMonitor monitor;

	/**
	 * The HmiThread constructor gets the hmi object
	 * @param hmi hmi object
	 * @param monitor monitor object
	 */
	public HmiSenderThread(HmiPLT hmi, IMonitor monitor) {
		this.hmi = hmi;
		this.monitor = monitor;
	}

	@Override
	public void run() {

		while(true){			 
			// If we us the HMI at all outputs
			if(hmi.useHMI){
				processOutputs();
				
				// MONITOR (example)
//				monitor.writeHmiComment("Hmi_Sender");
			}

			try{	            	
				Thread.sleep(100);

			} catch(InterruptedException ie){
				System.out.println("Interruption of HmiSenderThread in sleep()");
			}
		}
	}
	
	/**
	 * Writes current robot status. This method is to be called from the {@code HmiThread}. Execution time depends on the number of newly 
	 * discovered parking slots, as these data are enqueued behind position information during one method run.
	 * parking_slot implementation has been rewritten to get updates after one has been added
	 */
	private int timer = 0;
	private int counter = 0;
	private synchronized void processOutputs() {

		try
		{
			// write status - this has the highest transmission priority, thus it is executed first if thread gets interrupted early
			hmi.dataOut.writeInt(Command.OUT_STATUS.ordinal());
			hmi.dataOut.writeInt(GuidanceAT.getCurrentStatus().ordinal());
			hmi.dataOut.flush();
			RConsole.println("Status data geflusht.");

			//rewritten
			// getParkingSlots() returns null until it is implemented
			if (hmi.navigation.getParkingSlots() == null)
			{
				// do nothing
			}
			else
			{
				// send periodically one parking slot of list
				//loops through list
				// must be done this way, because sending data takes a lot of time
				timer ++;
				if (timer > 2){
					timer = 0;
					ParkingSlot newSlot = hmi.navigation.getParkingSlots()[counter];
					hmi.dataOut.writeInt(Command.OUT_PARKSLOT.ordinal());
					hmi.dataOut.writeInt(newSlot.getStatus().ordinal());
					hmi.dataOut.writeInt(newSlot.getID());
					hmi.dataOut.writeFloat(newSlot.getFrontBoundaryPosition().x);
					hmi.dataOut.writeFloat(newSlot.getFrontBoundaryPosition().y);
					hmi.dataOut.writeFloat(newSlot.getBackBoundaryPosition().x);
					hmi.dataOut.writeFloat(newSlot.getBackBoundaryPosition().y);
					hmi.dataOut.flush();
					counter++;
					if (counter >= hmi.noOfParkingSlots){
						counter = 0;
					}
				}
			}

			// write position - this has least priority, no damage is caused when position is not delivered during one cycle.
			Pose pose = hmi.navigation.getPose();

			hmi.dataOut.writeInt(Command.OUT_POSITION.ordinal());
			hmi.dataOut.writeFloat(pose.getX());
			hmi.dataOut.writeFloat(pose.getY());
			hmi.dataOut.writeFloat(pose.getHeading());

			// Distance sensor values in clockwise directions (front, right, back, left) in mm
			hmi.dataOut.writeDouble(hmi.perception.getFrontSensorDistance());
			hmi.dataOut.writeDouble(hmi.perception.getFrontSideSensorDistance());
			hmi.dataOut.writeDouble(hmi.perception.getBackSensorDistance());
			hmi.dataOut.writeDouble(hmi.perception.getBackSideSensorDistance());

			hmi.dataOut.flush();


		} catch (IOException e)
		{
			System.out.println("IO-Exception in processOutputs()");
		}
	}

}
