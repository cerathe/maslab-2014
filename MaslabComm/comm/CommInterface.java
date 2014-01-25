package comm;

import devices.MapleDevice;
import edu.mit.felixsun.maslab.cvData;

public interface CommInterface {
	public void registerDevice(MapleDevice device);
	
	public void initialize();
	
	public void transmit();
	
	public void updateSensorData();
	
	public cvData fakeImageProcessor();
}
