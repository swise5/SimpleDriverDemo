package objects;

import java.util.ArrayList;

import com.vividsolutions.jts.geom.Coordinate;

import sim.SimpleDrivers;
import sim.engine.SimState;
import sim.engine.Steppable;
import swise.agents.SpatialAgent;
import swise.objects.network.GeoNode;
import utilities.DepotUtilities;
import utilities.DriverUtilities;

public class Depot extends SpatialAgent implements Burdenable {

	SimpleDrivers world;
	
	GeoNode myNode = null;
	
	private ArrayList <Parcel> parcels;
	private ArrayList <ArrayList <Parcel>> rounds;

	int numBays;
	ArrayList <Driver> inBays;
	ArrayList <Driver> waiting;
	
	public Depot (Coordinate c, int numbays, SimpleDrivers world){
		super(c);
		parcels = new ArrayList <Parcel> ();
		inBays = new ArrayList <Driver> ();
		waiting = new ArrayList <Driver> ();
		this.world = world;
		this.numBays = numbays;
		rounds = new ArrayList <ArrayList <Parcel>> ();
	}
	
	public void setNode(GeoNode node){
		myNode = node;
	}
	
	public GeoNode getNode(){ return myNode;}
	
	@Override
	public void addParcel(Parcel p) {
		parcels.add(p);
	}

	@Override
	public boolean removeParcel(Parcel p) {
		return parcels.remove(p);
	}

	public boolean removeParcels(ArrayList <Parcel> ps){
		return parcels.removeAll(ps);
	}
	

	@Override
	public void addParcels(ArrayList<Parcel> ps) {
		parcels.addAll(ps);
	}

	@Override
	public boolean transferTo(Object o, Burdenable b) {
		try{
			if(o instanceof ArrayList){
				ArrayList <Parcel> ps = (ArrayList <Parcel>) o;
				for(Parcel p: ps)
					p.transfer(this, b);
			}
			else {
				((Parcel) o).transfer(this, b);
			}
			return true;
		} catch (Exception e){
			return false;
		}
	}

	
	@Override
	public Coordinate getLocation() {
		return geometry.getCoordinate();
	}
	
	@Override
	public void step(SimState arg0){
		world.schedule.scheduleOnce(this);
	}
	
	/**
	 * 
	 * @param d - the driver
	 * @return the amount of time before which to activate again. If <0, the Depot will
	 * activate the Driver when ready. 
	 */
	public int enterDepot(Driver d){
		
		System.out.println(d.toString() + " has entered the depot!");
		
		if(rounds.size() == 0) {
			d.stopWaypointLogger();
			return -1; // finished with everything
		}
		else if(inBays.size() >= numBays){
			waiting.add(d);
			world.schedule.scheduleOnce(new Steppable(){

				@Override
				public void step(SimState state) {
					if(inBays.size() < numBays){
						waiting.remove(d);
						enterBay(d);
					}
					else
						state.schedule.scheduleOnce(this);
				}
				
			});
		}
		else {
			enterBay(d);
			
		}
		
		return SimpleDrivers.loadingTime;
	}
	
	ArrayList <Parcel> getNextRound(){
		if(rounds.size() > 0)
			return rounds.remove(world.random.nextInt(rounds.size()));
		else
			return null;
	}
	
	void enterBay(Driver d){
		inBays.add(d);
		if(rounds.size() <= 0) {
			return;
		}
		else {
			d.setStatus(DriverUtilities.driverStates.LOADING);
			world.schedule.scheduleOnce(world.schedule.getTime() + world.loadingTime, new Steppable(){

				@Override
				public void step(SimState state) {
					ArrayList <Parcel> newRound = getNextRound();
					
					//no one new to thing
					if(newRound == null || newRound.size() <= 0)
						return;
					
					if(d.myVehicle != null){
						transferTo(newRound, d.myVehicle);	
					}
					else
						transferTo(newRound, d);
					
					System.out.println(d.toString() + " has taken on a new load: " + newRound.size());
					leaveDepot(d);
					d.startRoundClock();
					if(d.myVehicle != null)
						d.updateRoundClustered();
					else
						d.setWalkingRoute();
						
				}
			
			});
		}
	}
	
	/**
	 * 
	 * @param d the Driver to remove from the Depot
	 */
	public void leaveDepot(Driver d){
		
		// if the Driver was originally there, remove it
		if(inBays.contains(d)){
			inBays.remove(d);
			if(d.myVehicle != null)
				d.setStatus(DriverUtilities.driverStates.DRIVING_FROM_DEPOT);
			else
				d.setStatus(DriverUtilities.driverStates.WALKING_FROM_DEPOT);
			world.schedule.scheduleOnce(d);
			
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
	
	public void addRounds(ArrayList <ArrayList <Parcel>> rounds){
		this.rounds = rounds;
	}
	
	public void generateRounds(){
		rounds.addAll(DepotUtilities.gridDistribution(parcels, world.deliveryLocationLayer, world.approxManifestSize));
	}
}
