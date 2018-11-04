package objects;

import java.util.ArrayList;

import sim.SimpleDrivers;
import sim.engine.SimState;
import sim.engine.Steppable;
import sim.engine.Stoppable;
import sim.util.geo.MasonGeometry;
import swise.agents.TrafficAgent;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;

public class Driver extends TrafficAgent implements Steppable, Burdenable {

	Coordinate location = null;
	ArrayList <Coordinate> deliveries = new ArrayList <Coordinate> ();
	ArrayList <Parcel> parcels;
	public Stoppable stopper = null;
	double speed = 3.;
	
	public Driver(Coordinate c){
		super(c);
		this.location = (Coordinate) c.clone();
		parcels = new ArrayList <Parcel> ();
	}
	
	public void addDelivery(Coordinate c){
		deliveries.add((Coordinate) c.clone());
	}
	
	@Override
	public void step(SimState arg0) {
		
		SimpleDrivers world = (SimpleDrivers) arg0;
		
		// if you've finished your deliveries, stop it!
		if(deliveries.size() <= 0){
			if(stopper != null) stopper.stop();
			System.out.println(this.toString() + " is done with the round! It took " + world.schedule.getTime());
			return;
		}
		
		// otherwise, go to the location of the next delivery
		
		// find the vector to the next location
		Coordinate next = deliveries.get(0);
		double dx = next.x - location.x;
		double dy = next.y - location.y;

		// if we're within the resolution of the goal, you've done it! "Deliver" 
		// the parcel and move on to the next one
		if(Math.sqrt(Math.pow(dy,2) + Math.pow(dx, 2)) < world.resolution) {
			//System.out.println(this.toString() + " delivered parcel to " + deliveries.get(0).toString());
			deliveries.remove(0);
			world.schedule.scheduleOnce(world.schedule.getTime() + 5, this);
			return;
		}
		
		// otherwise, move onward toward the goal
		
		// normalise it, shittily
		double scale = Math.sqrt(Math.pow(dx, 2) + Math.pow(dy, 2));
		if(scale > speed){
			dx /= scale / speed;
			dy /= scale / speed;
		}
		
/*		// update the new location
		location = new Coordinate(SimpleDrivers.wrapMe(location.x + dx, 0, world.width), SimpleDrivers.wrapMe(location.y + dy, 0, world.height));
		((SimpleDrivers)arg0).citySpace.setObjectLocation(this, (int)location.x, (int)location.y);
		//System.out.println(">>>> now I am at " + location.toString() + " and I am going to " + deliveries.get(0).toString());
		world.schedule.scheduleOnce(this);
		*/
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
}