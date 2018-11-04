package objects;

import java.util.ArrayList;

import com.vividsolutions.jts.geom.Coordinate;

import sim.SimpleDrivers;
import swise.agents.SpatialAgent;

public class Depot extends SpatialAgent implements Burdenable {

	SimpleDrivers world;
	
	ArrayList <Parcel> parcels;

	int numBays;
	ArrayList <Driver> inBays;
	ArrayList <Driver> waiting;
	
	public Depot (Coordinate c, SimpleDrivers world){
		super(c);
		parcels = new ArrayList <Parcel> ();
		inBays = new ArrayList <Driver> ();
		waiting = new ArrayList <Driver> ();
		this.world = world;
	}
	
	@Override
	public void addParcel(Parcel p) {
		parcels.add(p);
	}

	@Override
	public boolean removeParcel(Parcel p) {
		return parcels.remove(p);
	}

	@Override
	public Coordinate getLocation() {
		return geometry.getCoordinate();
	}
	
	/**
	 * 
	 * @param d - the driver
	 * @return the amount of time before which to activate again. If <0, the Depot will
	 * activate the Driver when ready. 
	 */
	int enterDepot(Driver d){
		if(inBays.size() >= numBays){
			waiting.add(d);
			return -1;
		}
		else
			inBays.add(d);
		return SimpleDrivers.loadingTime;
	}
	
	/**
	 * 
	 * @param d the Driver to remove from the Depot
	 */
	void leaveDepot(Driver d){
		
		// if the Driver was originally there, remove it
		if(inBays.contains(d)){
			inBays.remove(d);
			
			// if there are Drivers waiting in the queue, let the next one move in
			if(waiting.size() > 0){
				Driver n = waiting.remove(0);
				inBays.add(n);
				world.schedule.scheduleOnce(world.schedule.getTime() + SimpleDrivers.loadingTime, n);
			}
		}
		else
			System.out.println("Error: driver was never in bay");
	}
}