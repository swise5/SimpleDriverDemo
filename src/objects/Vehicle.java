package objects;

import java.util.ArrayList;
import java.util.UUID;

import com.vividsolutions.jts.geom.Coordinate;

import sim.SimpleDrivers;
import sim.engine.SimState;
import sim.engine.Steppable;
import sim.util.geo.MasonGeometry;
import sim.util.gui.WordWrap;
import swise.agents.MobileAgent;

public class Vehicle extends MobileAgent implements Burdenable {
	
	SimpleDrivers world;
	
	Driver owner;
	ArrayList <Parcel> parcels = new ArrayList <Parcel> ();
	
	
	/* 
	 * OUTPUT AND LOGGING VARS
	 * Secondary variables, for capturing and communicating information out of the model
	 * Not to be used for operational processes
	 */
	
	UUID vehicleUID;
	String shortID = "";
	ArrayList <String> waypointsTrace = new ArrayList <String> ();
	int waypointTraceInterval = 10;
	
	/*
	 * END OUTPUT AND LOGGING VARS
	 */
	
	public Vehicle(Coordinate c, Driver d){
		super((Coordinate)c.clone());
		isMovable = true;
		owner = d;
		this.world = owner.world;
		
		vehicleUID = UUID.randomUUID();
		shortID = vehicleUID.toString().substring(0, 8);
		shortID = "V-" + shortID;
		
		//scheduling of path recording, polling rate can be modified
		Steppable steppable = new Steppable(){
			
			public void step(SimState state) {
				LogWaypoint();
			}
		};
		world.schedule.scheduleRepeating(world.schedule.EPOCH, 3, steppable, waypointTraceInterval);
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
	public void addParcels(ArrayList<Parcel> ps) {
		parcels.addAll(ps);
	}

	@Override
	public boolean removeParcels(ArrayList<Parcel> ps) {
		return parcels.removeAll(ps);
	}

	@Override
	public Coordinate getLocation() {
		return geometry.getCoordinate();
	}
	
	public void setLocation(Coordinate newC) {
		updateLoc(newC);
	}

	@Override
	public boolean transferTo(Object o, Burdenable b) {
		try{
			if(o instanceof ArrayList){
				parcels.removeAll((ArrayList <Parcel>) o);
				b.addParcels((ArrayList <Parcel>) o);
			}
			else {
				parcels.remove((Parcel) o);
				b.addParcel((Parcel) o);
			}
			return true;
		} catch (Exception e){
			return false;
		}
	}
	
	void setStationary(){
		if(owner != null){
			Coordinate c = owner.getLocation();
			updateLoc(new Coordinate(c.x, c.y));
			owner = null;
		}
		else
			updateLoc(getLocation());
	}
	
	void setDriver(Driver d){
		this.owner = d;
	}
	
	void LogWaypoint() {
		//row format: ID, TIME, X, Y
		double t = world.schedule.getTime();
		Coordinate c = getLocation();
		String r = shortID + "," + t + "," + c.x + "," + c.y;
//		System.out.println(r);
		waypointsTrace.add(r);
	}
	
	public ArrayList <String> getWaypointsTrace() {return waypointsTrace; }
}