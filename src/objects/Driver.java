package objects;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Map.Entry;
import java.util.UUID;

import sim.SimpleDrivers;
import sim.engine.SimState;
import sim.engine.Steppable;
import sim.engine.Stoppable;
import sim.field.network.Edge;
import sim.util.Bag;
import sim.util.geo.AttributeValue;
import sim.util.geo.MasonGeometry;
import swise.agents.TrafficAgent;
import swise.objects.network.GeoNode;
import swise.objects.network.ListEdge;
import utilities.DriverUtilities;
import utilities.DriverUtilities.driverStates;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.LineString;
import com.vividsolutions.jts.geom.Point;
import com.vividsolutions.jts.linearref.LengthIndexedLine;

public class Driver extends TrafficAgent implements Steppable, Burdenable {

	SimpleDrivers world;
	Coordinate homeBase = null;
	Coordinate targetDestination = null;
	double roundStartTime = -1;
	double roundDriveDistance = 0, roundWalkDistance = 0;

	ArrayList <Parcel> parcels = new ArrayList <Parcel> ();
	//ArrayList <Parcel> myRound = new ArrayList <Parcel> ();
	ArrayList <MasonGeometry> myRound = null;
	HashMap <MasonGeometry, ArrayList <Parcel>> parkingPerRound = new HashMap <MasonGeometry, ArrayList <Parcel>> (); 
	ArrayList <String> history = new ArrayList <String> ();

	int roundIndex = 0;
	int miniRoundIndex = -1;
	public Stoppable stopper = null;
	double speed = 3.; // m per second
	
	double enteredRoadSegment = -1;

	Parcel currentDelivery = null;
	ArrayList <Parcel> miniRound = null;
	Vehicle myVehicle = null;
	boolean inVehicle = false;
	
	/* 
	 * OUTPUT AND LOGGING VARS
	 * Secondary variables, for capturing and communicating information out of the model
	 * Not to be used for operational processes
	 */
	
	//TODO verify that currentDriverState is being set correctly at all times. ATM it is good enough, but might break at any moment
	driverStates currentDriverState = driverStates.DEFAULT;
	
	ArrayList <String> roundStats = new ArrayList <String> ();
	
	//logging vars
	double timeSpentDriving = 0;
	double timeSpentWalking = 0;
	double timeSpentVehicleParked = 0;
	double timeSpentDrivingStem = 0;
	double distanceDriven = 0;
	double distanceWalked = 0;
	double distanceDrivenStem = 0;
	
	UUID agentUID; //agent unique ID
	String shortID = ""; //shorter id, not guaranteed to be unique
	
	int roundParcelsCount = 0; //number of parcels in the round
	MasonGeometry firstDeliveryInRound; // For logging purposes, to identify stem mileage
	Coordinate coordinateAtPreviousStep;
	
	ArrayList <String> waypointsTrace = new ArrayList <String> ();
	int waypointTraceInterval = 1; //update interval for logging the current waypoint, similar to GPS polling rate
	
	int overallRoundIndex = 0; // which round we are currently in, in case of multiple rounds per driver
	double timeSinceRoundStarted = 0;
	
	Steppable steppableWaypointLogger;
	Steppable steppableRoundStatsUpdate;
	Stoppable stoppableWaypointLogger;
	
	/*
	 * END OUTPUT AND LOGGING VARS
	 */
	
	public String toString() { return "[ " + shortID + " | " + currentDriverState.toString() + " ]"; } // returns driver ID and its currente state
	
	
	public Driver(SimpleDrivers world, Coordinate c){
		super(c);
		homeBase = (Coordinate) c.clone();
		this.world = world;
		parcels = new ArrayList <Parcel> ();
		
		speed = world.speed_pedestrian;
		
		edge = SimpleDrivers.getClosestEdge(c, world.resolution, world.networkEdgeLayer, world.fa);
		
		if(edge == null){
			System.out.println("\tINIT_ERROR: no nearby edge");
			return;
		}
			
		GeoNode n1 = (GeoNode) edge.getFrom();
		GeoNode n2 = (GeoNode) edge.getTo();
		
		if(n1.geometry.getCoordinate().distance(c) <= n2.geometry.getCoordinate().distance(c))
			node = n1;
		else 
			node = n2;

		segment = new LengthIndexedLine((LineString)((MasonGeometry)edge.info).geometry);
		startIndex = segment.getStartIndex();
		endIndex = segment.getEndIndex();
		currentIndex = segment.indexOf(c);
		
		this.isMovable = true;
		
		timeSpentDriving = 0;
		timeSpentWalking = 0;
		coordinateAtPreviousStep = (Coordinate) getLocation().clone();
		
		agentUID = UUID.randomUUID();
		shortID = agentUID.toString().substring(0, 8);
		shortID = "D-" + shortID;
		
		//scheduling of path recording, polling rate can be modified
		steppableWaypointLogger = new Steppable(){
			public void step(SimState state) {
				LogWaypoint();
			}
		};
		stoppableWaypointLogger = world.schedule.scheduleRepeating(world.schedule.EPOCH, 3, steppableWaypointLogger, waypointTraceInterval);
		
		//scheduling of round stats recording, happens EVERY tick
		steppableRoundStatsUpdate = new Steppable(){
			public void step(SimState state) {
				UpdateRoundStats();
				coordinateAtPreviousStep = (Coordinate) getLocation().clone();
			}
		};
		world.schedule.scheduleRepeating(world.schedule.EPOCH, 4, steppableRoundStatsUpdate,1);
	}
	
	public void stopWaypointLogger() {
		stoppableWaypointLogger.stop();
		if(myVehicle != null) {
			myVehicle.stoppableWaypointLogger.stop();
		}
	}
	
	public void startRoundClock(){
		roundStartTime = world.schedule.getTime();
		roundDriveDistance = 0;
		roundWalkDistance = 0;
		roundParcelsCount = parcels.size();
		if(myVehicle != null)
			roundParcelsCount += myVehicle.parcels.size();
		
		overallRoundIndex ++;
	}
	
	
	boolean attemptDelivery(double time){
		
		// failed delivery ):
		double mynextrand = world.random.nextDouble(); 
		if (mynextrand < world.probFailedDelivery) { 
			System.out.println(
					this.toString() + " has NOT been able to deliver parcel " + currentDelivery.toString());
			miniRoundIndex++;
			currentDelivery.status = 1; // failed attempt
		} 
		
		// successful delivery!
		else { 
			currentDelivery.deliver(world.fa.createPoint(currentDelivery.deliveryLocation));
			miniRound.remove(miniRoundIndex);
			//System.out.println(this.toString() + " has delivered parcel " + currentDelivery.toString());
			world.deliveryLocationLayer.addGeometry(currentDelivery);
		}

		// reset things
		world.schedule.scheduleOnce(time + world.deliveryTime, this);
		currentDelivery = null;
		path = null;

		return true;
	}
	
	void exitVehicle(){
		if(!inVehicle || myVehicle == null){
			System.out.println("ERROR: not in vehicle!");
			return;
		}
		
		inVehicle = false;
		myVehicle.setStationary();
		this.speed = SimpleDrivers.speed_pedestrian;
	}
	
	void enterVehicle(){
		if(myVehicle == null){
			System.out.println("ERROR: do not have a vehicle!");
			return;
		}
		
		inVehicle = true;
		myVehicle.setDriver(this);
		this.speed = SimpleDrivers.speed_vehicle;
	}
	
	void scheduleNextGoal(Coordinate c){
		headFor(c);
		world.schedule.scheduleOnce(this);
	}
	
	void setWalkingRoute(){
		myRound = new ArrayList <MasonGeometry> ();
		parkingPerRound = new HashMap <MasonGeometry, ArrayList <Parcel>> ();

		MasonGeometry homeBaseMB = new MasonGeometry(world.fa.createPoint(homeBase));
		myRound.add(homeBaseMB);
		ArrayList <Parcel> parcelsCopy = (ArrayList <Parcel>) parcels.clone(); 
		parkingPerRound.put(homeBaseMB, parcelsCopy);
		
		firstDeliveryInRound = myRound.get(0);
	}
	
	@Override
	public void step(SimState arg0) {
		
		double time = world.schedule.getTime(); // find the current time
		
		if(inVehicle) {
			myVehicle.setLocation(this.getLocation());
		}
		
	     // make sure the round has been defined with parking spaces if needed; update it if not
	     if(myRound == null){
	    	 if(myVehicle != null)
	    		 updateRoundClustered();
	    	 else
	    		 setWalkingRoute();
		}

		// is there more path to travel? If so, take it
		if(path != null){
			navigate(world.resolution);
			world.schedule.scheduleOnce(this);
			return;
		}
		
		// if you're in the process of delivering a parcel, proceed
		if (currentDelivery != null) {
			
			// if not in vehicle, attempt delivery
			if(myVehicle == null || !inVehicle){
				currentDriverState = driverStates.WALKING_TO_DELIVERY;
				if (geometry.getCoordinate().distance(currentDelivery.deliveryLocation) < world.resolution) {
					attemptDelivery(time);
					currentDelivery = null; // it has been attempted! TODO ensure this is sorted
					return;
				}
				else{
					this.walkTo(currentDelivery.deliveryLocation, world.resolution);
					world.schedule.scheduleOnce(this);
					return;
				}
			}
			
			// otherwise get out of vehicle and try again next time
			else if(inVehicle){
				
				// check for others in the parking space - if so, wait around
				Bag b = world.agentLayer.getObjectsWithinDistance(geometry, world.resolution);
				if(b.size() > 1){
					System.out.println("waiting for other parker to move");
					world.schedule.scheduleOnce(this);
					return;
				}
				
				exitVehicle();
				world.schedule.scheduleOnce(this);
				
				currentDriverState = driverStates.WALKING_TO_DELIVERY;
				return;
			}

			else { // a wild problem appears!
				System.out.println("ERROR: problem with driver behaviours for " + this.toString());
				return;
			}			
		}

		// otherwise if we've got to the next miniround starting point, transition to doing a miniround!
		else if(roundIndex < myRound.size() && 
				myRound.get(roundIndex).geometry.getCoordinate().distance(this.geometry.getCoordinate()) <= world.resolution && 
				miniRoundIndex < 0){
			miniRound = parkingPerRound.get(myRound.get(roundIndex));
			//myRound.remove(roundIndex); // clean it up - we'll not come here again
			miniRoundIndex = 0;
			
			if(myVehicle != null && miniRound != null){
				ArrayList <Parcel> dummyParcels = (ArrayList <Parcel>) miniRound.clone();
				dummyParcels.retainAll(myVehicle.parcels);
				myVehicle.transferTo(dummyParcels, this);
			}
			
			world.schedule.scheduleOnce(this);
			return;
		}
		
		// otherwise if you've been trying to get in the vehicle and are close enough, get in
		else if(miniRound == null && myVehicle != null && !inVehicle &&
				myVehicle.getLocation().distance(geometry.getCoordinate()) <= world.resolution){
			
			transferTo(parcels, myVehicle);
			
			enterVehicle();
			world.schedule.scheduleOnce(this);
				
			currentDriverState = driverStates.DRIVING;
			return;
		}


		// HOUSEKEEPING ON MINIROUND
		
		// if anything left to deliver on this miniround, deliver it
		if(miniRound != null && miniRoundIndex < miniRound.size()){
			
			// make sure that you're heading to the next closest delivery point
			int nextClosestDeliveryPoint = nextClosestDelivery(miniRoundIndex);
			if(nextClosestDeliveryPoint > miniRoundIndex){
				Parcel p = miniRound.remove(nextClosestDeliveryPoint);
				miniRound.add(miniRoundIndex, p);
			}
			
			// schedule the next delivery
			currentDelivery = miniRound.get(miniRoundIndex);
			scheduleNextGoal(currentDelivery.deliveryLocation);
			return;
		}

		// if the miniround has been completed, increment the round counter
		else if(miniRound != null && miniRoundIndex >= miniRound.size()){
			miniRound = null;
			miniRoundIndex = -1;
			roundIndex++;
		}
		
		// this miniround has been completed. Head back to the vehicle, if appropriate
		if(myVehicle != null && !inVehicle){
			scheduleNextGoal(myVehicle.getLocation());
			currentDriverState = driverStates.WALKING_TO_VEHICLE;
			return;
		}
		
		// HOUSEKEEPING ON ROUND
		
		// otherwise, is there another organising point to hit up? If so, go to it
		if(roundIndex < myRound.size()){
			
			// make sure you're heading for the next closest spot, rejigging the ordering if necessary
			int nextClosestRallyPoint = nextClosestParkingSpot(roundIndex);
			if(nextClosestRallyPoint > roundIndex){
				MasonGeometry temp = myRound.remove(nextClosestRallyPoint);
				myRound.add(roundIndex, temp);
			}
			
			// schedule to move on to the next closest parking space
			scheduleNextGoal(myRound.get(roundIndex).geometry.getCoordinate());
			return;
		}
		
		// the round is finished! Back to the depot with you!
		else if(homeBase.distance(geometry.getCoordinate()) > world.resolution) { 
			returnToDepot();
			return;
		}
		//  otherwise, you're at the Depot - enter it
		else {
			cleanupAtDepot();
		}

	}

	
	/**
	 * Schedule the Driver to return to the home Depot. Change the status based on whether they have a vehicle.
	 */
	void returnToDepot(){
		scheduleNextGoal(homeBase);
		if(myVehicle != null)
			currentDriverState = driverStates.DRIVING_TO_DEPOT;
		else
			currentDriverState = driverStates.WALKING_TO_DEPOT;
	}
	
	/**
	 * 
	 * @param startIndex - starting from this index, search for the next nearest parking space
	 * @return index of the nearest parking space
	 */
	int nextClosestParkingSpot(int startIndex){
		double distance = Double.MAX_VALUE;
		int bestIndex = -1;
		for(int i = startIndex; i < myRound.size(); i++){
			double distToSpace = myRound.get(i).geometry.distance(this.geometry);
			if(distToSpace < distance){
				distance = distToSpace;
				bestIndex = i;
			}
		}
		return bestIndex;
	}
	
	int nextClosestDelivery(int startIndex){
		double distance = Double.MAX_VALUE;
		int bestIndex = -1;
		Coordinate c = this.geometry.getCoordinate();
		for(int i = startIndex; i < miniRound.size(); i++){
			double distToSpace = miniRound.get(i).deliveryLocation.distance(c);
			if(distToSpace < distance){
				distance = distToSpace;
				bestIndex = i;
			}
		}
		return bestIndex;
	}
	
	void cleanupAtDepot(){
			
		// write out the report
		double roundTime = world.schedule.getTime() - roundStartTime;
		history.add(this.toString() + "\t" + roundTime + "\t" + roundDriveDistance + "\t" + roundWalkDistance);
		System.out.println(this.toString() + " is done with the round! It took " + (world.schedule.getTime() - roundStartTime));
		
		recordCurrentRoundStats();
		resetRoundStats();
		
		// transfer all undelivered parcels
		Bag b = world.depotLayer.getObjectsWithinDistance(geometry, world.resolution);
		if(b.size() > 0){
			Depot d = (Depot) b.get(0);
			d.enterDepot(this);
			int numLeftovers = parcels.size();
			if(myVehicle != null)
				numLeftovers += myVehicle.parcels.size();
			System.out.println("Round finished - driver " + this.toString() + " has returned with " + numLeftovers);
			if(parcels.size() > 0)
				transferTo(parcels, d);
			if(myVehicle != null)
				myVehicle.transferTo(myVehicle.parcels, d);
		}
		
		// reset everything
		roundIndex = 0;
		miniRoundIndex = -1;
		parkingPerRound = null;
		myRound = null;
		currentDriverState = driverStates.DEFAULT;
	}
	
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

	public void updateRoundClustered(){

		myRound = new ArrayList <MasonGeometry> ();
		parkingPerRound = new HashMap <MasonGeometry, ArrayList <Parcel>> ();
		
		HashMap <MasonGeometry, ArrayList <Parcel>> parkingSpaceOptions = new HashMap <MasonGeometry, ArrayList <Parcel>> ();
		
		ArrayList <Parcel> allTempParcels = new ArrayList <Parcel> ();
		allTempParcels.addAll(parcels);
		if(this.myVehicle != null)
			allTempParcels.addAll(myVehicle.parcels);
		
		// go through Parcels and allocate parking spaces
		for(Parcel p: allTempParcels){
		
			Coordinate parkingOnRoad = world.snapPointToRoadNetwork(p.getDeliveryLocation());
			
			// look for covering parking spaces
			Bag b = world.parkingCatchmentLayer.getCoveringObjects(world.fa.createPoint(parkingOnRoad));
			
			// if there are no nearby parking spaces, we'll plan around the delivery itself
			if(b.size() == 0){
				
				MasonGeometry mg = new MasonGeometry(world.fa.createPoint(parkingOnRoad));
				if(parkingSpaceOptions.containsKey(mg))
					parkingSpaceOptions.get(mg).add(p);
				else {
					ArrayList <Parcel> ps = new ArrayList <Parcel>();
					ps.add(p);
					parkingSpaceOptions.put(mg, ps);					
				}
			}
			
			// if there ARE nearby parking spaces, we'll add this to the list of possible spaces!
			else{
				for(Object o: b){
					MasonGeometry mg = (MasonGeometry) o;

					if(parkingSpaceOptions.containsKey(mg))
						parkingSpaceOptions.get(mg).add(p);
					else {
						ArrayList <Parcel> ps = new ArrayList <Parcel>();
						ps.add(p);
						parkingSpaceOptions.put(mg, ps);
					}
				}
			}
		}
		
		// sort the list of parking spaces in order of descending number of parcels associated with it
		List dummyList = new LinkedList(parkingSpaceOptions.entrySet());
		Collections.sort(dummyList, new Comparator<Entry<MasonGeometry, ArrayList <Parcel>>>()
        {

			@Override
			public int compare(Entry<MasonGeometry, ArrayList <Parcel>> o1, Entry<MasonGeometry, ArrayList<Parcel>> o2) {
				if(o1.getValue().size() > o2.getValue().size()) return -1;
				else if (o1.getValue().size() == o2.getValue().size()) return 0;
				return 1;
			}
        });
		
		// now go through the set of parcels and add parking spaces to the route until all parcels are sorted
		Iterator iter = dummyList.iterator();
		double bestDist = Double.MAX_VALUE;
		while(allTempParcels.size() > 0 && iter.hasNext()){
			
			// get the next biggest one
			Entry <MasonGeometry, ArrayList <Parcel>> nextOne = (Entry <MasonGeometry, ArrayList <Parcel>>)
					iter.next();
			
			ArrayList <Parcel> toDeliver = nextOne.getValue();
			toDeliver.retainAll(allTempParcels);
			
			if(toDeliver.size() <= 0)
				continue;
			
			allTempParcels.removeAll(toDeliver);
			
			MasonGeometry parkingSpaceItself = ((MasonGeometry)nextOne.getKey());
			if(parkingSpaceItself.hasAttribute("parkingspace"))
				parkingSpaceItself = (MasonGeometry)((AttributeValue)parkingSpaceItself.getAttribute("parkingspace")).getValue();
	
			// start at the closest one!
			double thisDist = parkingSpaceItself.geometry.distance(this.geometry);
			if(thisDist < bestDist){
				bestDist = thisDist;
				myRound.add(0, parkingSpaceItself);
			}
			else
				myRound.add(parkingSpaceItself);
			
			// save the record
			parkingPerRound.put(parkingSpaceItself, toDeliver);
			
			firstDeliveryInRound = myRound.get(0);
		}
		if(!iter.hasNext() && allTempParcels.size() > 0){
			System.out.println("but whyyy");
			for(Parcel pot: allTempParcels)
				System.out.println(pot.deliveryLocation.toString());
		}
		
	}
	
	@Override
	public boolean transferTo(Object o, Burdenable b) {
		try{
			if(o instanceof ArrayList){
				ArrayList <Parcel> ps = (ArrayList <Parcel>) o;
				while(ps.size() > 0){
					Parcel p = ps.remove(0);
					p.transfer(this, b);
				}
					
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
	
	public void setNode(GeoNode n){
		node = n;
	}
	
	/**
	 * 
	 * @param resolution
	 * @return 1 for success, -1 for failure
	 */	
	public int navigate(double resolution){
		myLastSpeed = -1;
		
		if(path != null){
			double time = 1;//speed;
			while(path != null && time > 0){
				time = move(time, speed, resolution);
			}
			
			if(segment != null)
				updateLoc(segment.extractPoint(currentIndex));				

			if(time < 0){
				return -1;
			}
			else
				return 1;
		}
		return -1;		
	}
	
	public int walkTo(Coordinate c, double resolution){
		Coordinate myLoc = geometry.getCoordinate();
		double dx = c.x - myLoc.x, dy = c.y - myLoc.y;

		double theta = Math.atan2(dy, dx);
		
		double hypot = Math.sqrt(dx * dx + dy * dy);
		double moveFactor = Math.min(hypot, world.speed_pedestrian);		
		double cosFriend = moveFactor * Math.cos(theta), sinFriend = moveFactor * Math.sin(theta);
		
		Coordinate newLoc = new Coordinate(myLoc.x + cosFriend, myLoc.y + sinFriend);
		updateLoc(newLoc);
		return 1;
	}
	
	/**
	 * 
	 * @param time - a positive amount of time, representing the period of time agents 
	 * 				are allocated for movement
	 * @param obstacles - set of spaces which are obstacles to the agent
	 * @return the amount of time left after moving, negated if the movement failed
	 */
	protected double move(double time, double mySpeed, double resolution){
		
		// if we're at the end of the edge and we have more edges, move onto the next edge
		if(arrived() ){
			
			// clean up any edge we leave
			if(edge != null && edge.getClass().equals(ListEdge.class)){
				((ListEdge)edge).removeElement(this);
				
				// update the edge with how long you've spent on it
			//	double durationOnSegment = ((MasonGeometry)edge.info).getDoubleAttribute("MikeSim_timeOnRoad");
				
			//	if(enteredRoadSegment > 0) // if you began on the edge and never really entered it, don't consider this
			//		((MasonGeometry)edge.info).addDoubleAttribute("MikeSim_timeOnRoad", 
			//			durationOnSegment + world.schedule.getTime() - enteredRoadSegment);
			}

			// if we have arrived and there is no other edge in the path, we have finished our journey: 
			// reset the path and return the remaining time
			if(goalPoint == null && path.size() == 0 && (currentIndex <= startIndex || currentIndex >= endIndex )){
				path = null;
				return time;
			}
			
			// make sure that there is another edge in the path
			if(path.size() > 0) { 

				// take the next edge
				Edge newEdge = path.remove(path.size() - 1);				
				edge = newEdge;

				// make sure it's open
				// if it's not, return an error!
		/*		if(((MasonGeometry)newEdge.info).getStringAttribute("open").equals("CLOSED")){
					updateLoc(node.geometry.getCoordinate());
					edge = newEdge;
					path = null;
					return -1;
				}				
*/
				// change our positional node to be the Node toward which we're moving
				node = (GeoNode) edge.getOtherNode(node);
				
				// format the edge's geometry so that we can move along it conveniently
				LineString ls = (LineString)((MasonGeometry)edge.info).geometry;

				// set up the segment and coordinates
				segment = new LengthIndexedLine(ls);
				startIndex = segment.getStartIndex();
				endIndex = segment.getEndIndex();
				currentIndex = segment.project(this.geometry.getCoordinate());
				
				
				// if that was the last edge and we have a goal point, resize the expanse
				if(path.size() == 0 && goalPoint != null){ 
					double goalIndex = segment.project(goalPoint);
					if(currentIndex < goalIndex)
						endIndex = goalIndex;
					else
						startIndex = goalIndex;
				}
				
				// make sure we're moving in the correct direction along the Edge
				if(node.equals(edge.to())){
					direction = 1;
					currentIndex = Math.max(currentIndex, startIndex);
				} else {
					direction = -1;
					currentIndex = Math.min(currentIndex, endIndex);
				}

				if(edge.getClass().equals(ListEdge.class)){
					((ListEdge)edge).addElement(this);
				//	int numUsages = ((MasonGeometry)edge.info).getIntegerAttribute("MikeSim_useages");
				//	((MasonGeometry)edge.info).addIntegerAttribute("MikeSim_useages", numUsages + 1);

					enteredRoadSegment = world.schedule.getTime();
				}

			}
						

		}
		
		// otherwise, we're on an Edge and moving forward!

		// set our speed
		double speed;
		if(edge != null && edge.getClass().equals(ListEdge.class)){
			
			// Each car has a certain amount of space: wants to preserve a following distance. 
			// If the amount of following distance is less than 20 meters (~ 6 car lengths) it'll slow
			// proportionately
			double val = ((ListEdge)edge).lengthPerElement() / 5;
			if(val < 10 && this.speed == SimpleDrivers.speed_vehicle) {
				speed = mySpeed / val;//minSpeed);
				if(speed < 1){ // if my speed is super low, set it to some baseline to keep traffic moving at all
					int myIndexInEdge =((ListEdge)edge).returnMyIndex(this);
					if(myIndexInEdge == 0 || myIndexInEdge == ((ListEdge)edge).numElementsOnListEdge() - 1)
						speed = this.speed; // if I'm at the head or end of the line, move ahead at a fairly normal speed
				}
			}
			else
				speed = this.speed;
			
		}
		else
			speed = mySpeed;

		myLastSpeed = speed;
		
		// construct a new current index which reflects the speed and direction of travel
		double proposedCurrentIndex = currentIndex + time * speed * direction;
		
		// great! It works! Move along!
		currentIndex = proposedCurrentIndex;
				
		if( direction < 0 ){
			if(currentIndex < startIndex){
				time = (startIndex - currentIndex) / speed; // convert back to time
				currentIndex = startIndex;
			}
			else
				time = 0;
		}
		else if(currentIndex > endIndex){
			time = (currentIndex - endIndex) / speed; // convert back to time
			currentIndex = endIndex;
		}
		else
			time = 0;

		// don't overshoot if we're on the last bit!
		if(goalPoint != null && path.size() == 0){
			double idealIndex = segment.indexOf(goalPoint);
			if((direction == 1 && idealIndex <= currentIndex) || (direction == -1 && idealIndex >= currentIndex)){
				currentIndex = idealIndex;
				time = 0;
				startIndex = endIndex = currentIndex;
			}
		}

		updateLoc(segment.extractPoint(currentIndex));
		
		if(path.size() == 0 && arrived()){
			path = null;
			if(edge != null)
				((ListEdge)edge).removeElement(this);
		}
		return time;
	}
	
	/**
	 * Set up a course to take the Agent to the given coordinates
	 * 
	 * @param place - the target destination
	 * @return 1 for success, -1 for a failure to find a path, -2 for failure based on the provided destination or current position
	 */
	public int headFor(Coordinate place) {

		//TODO: MUST INCORPORATE ROAD NETWORK STUFF
		if(place == null){
			System.out.println("ERROR: can't move toward nonexistant location");
			return -1;
		}
		
		// first, record from where the agent is starting
		startPoint = this.geometry.getCoordinate();
		goalPoint = null;

		if(!(edge.getTo().equals(node) || edge.getFrom().equals(node))){
			System.out.println( (int)world.schedule.getTime() + "\tMOVE_ERROR_mismatch_between_current_edge_and_node");
			return -2;
		}

		// FINDING THE GOAL //////////////////

		// set up goal information
		targetDestination = world.snapPointToRoadNetwork(place);
		
		GeoNode destinationNode = world.snapPointToNode(targetDestination);
		if(destinationNode == null){
			System.out.println((int)world.schedule.getTime() + "\tMOVE_ERROR_invalid_destination_node");
			return -2;
		}

		// be sure that if the target location is not a node but rather a point along an edge, that
		// point is recorded
		if(destinationNode.geometry.getCoordinate().distance(targetDestination) > world.resolution)
			goalPoint = targetDestination;
		else
			goalPoint = null;


		// FINDING A PATH /////////////////////

		path = pathfinder.astarPath(node, destinationNode, world.roads);

		// if it fails, give up
		if (path == null){
			return -1;
		}

		// CHECK FOR BEGINNING OF PATH ////////

		// we want to be sure that we're situated on the path *right now*, and that if the path
		// doesn't include the link we're on at this moment that we're both
		// 		a) on a link that connects to the startNode
		// 		b) pointed toward that startNode
		// Then, we want to clean up by getting rid of the edge on which we're already located

		// Make sure we're in the right place, and face the right direction
		if (edge.getTo().equals(node))
			direction = 1;
		else if (edge.getFrom().equals(node))
			direction = -1;
		else {
			System.out.println((int)world.schedule.getTime() + "MOVE_ERROR_mismatch_between_current_edge_and_node_2");
			return -2;
		}

		// reset stuff
		if(path.size() == 0 && targetDestination.distance(geometry.getCoordinate()) > world.resolution){
			path.add(edge);
			node = (GeoNode) edge.getOtherNode(node); // because it will look for the other side in the navigation!!! Tricky!!
		}

		// CHECK FOR END OF PATH //////////////

		// we want to be sure that if the goal point exists and the Agent isn't already on the edge 
		// that contains it, the edge that it's on is included in the path
		if (goalPoint != null) {

			ListEdge myLastEdge = world.getClosestEdge(goalPoint, world.resolution, world.networkEdgeLayer, world.fa);
			
			if(myLastEdge == null){
				System.out.println((int)world.schedule.getTime() + "\tMOVE_ERROR_goal_point_is_too_far_from_any_edge");
				return -2;
			}
			
			// make sure the point is on the last edge
			Edge lastEdge;
			if (path.size() > 0)
				lastEdge = path.get(0);
			else
				lastEdge = edge;

			Point goalPointGeometry = world.fa.createPoint(goalPoint);
			if(!lastEdge.equals(myLastEdge) && ((MasonGeometry)lastEdge.info).geometry.distance(goalPointGeometry) > world.resolution){
				if(lastEdge.getFrom().equals(myLastEdge.getFrom()) || lastEdge.getFrom().equals(myLastEdge.getTo()) 
						|| lastEdge.getTo().equals(myLastEdge.getFrom()) || lastEdge.getTo().equals(myLastEdge.getTo()))
					path.add(0, myLastEdge);
				else{
					System.out.println((int)world.schedule.getTime() + "\tMOVE_ERROR_goal_point_edge_is_not_included_in_the_path");
					return -2;
				}
			}
			
		}

		// set up the coordinates
		this.startIndex = segment.getStartIndex();
		this.endIndex = segment.getEndIndex();

		return 1;
	}

	public void assignVehicle(Vehicle v){
		myVehicle = v;
		inVehicle = true;
	}
	
	public Vehicle getVehicle() {
		return myVehicle;
	}
	
	public ArrayList <String> getHistory() {return history; }
	public ArrayList <String> getWaypointsTrace() {return waypointsTrace; }
	public ArrayList <String> getRoundStats() {return roundStats; }
	
	void LogWaypoint() {
		//row format: ID, TIME, X, Y
		double t = world.schedule.getTime();
		Coordinate c = getLocation();
		
		String r = shortID + "," + t + "," + c.x + "," + c.y;
		waypointsTrace.add(r);
	}
	
	//this one should run every frame, as it is capturing times and distances
	void UpdateRoundStats() {
		double distanceFromPreviousStep = getLocation().distance(coordinateAtPreviousStep);
		switch (currentDriverState) {
			case DRIVING:
				timeSpentDriving++;
				distanceDriven += distanceFromPreviousStep;
				timeSinceRoundStarted ++;
				break;
			case DRIVING_FROM_DEPOT:
				timeSpentDriving++;
				timeSpentDrivingStem++;
				distanceDriven += distanceFromPreviousStep;
				distanceDrivenStem += distanceFromPreviousStep;
				timeSinceRoundStarted ++;
				break;
			case DRIVING_TO_DEPOT:
				timeSpentDriving++;
				timeSpentDrivingStem++;
				distanceDriven += distanceFromPreviousStep;
				distanceDrivenStem += distanceFromPreviousStep;
				timeSinceRoundStarted ++;
				break;
			case WALKING:
				timeSpentWalking++;
				timeSpentVehicleParked++;
				distanceWalked += distanceFromPreviousStep;
				timeSinceRoundStarted ++;
				break;
			case WALKING_TO_DELIVERY:
				timeSpentWalking++;
				timeSpentVehicleParked++;
				distanceWalked += distanceFromPreviousStep;
				timeSinceRoundStarted ++;
				break;
			case WALKING_TO_VEHICLE:
				timeSpentWalking++;
				timeSpentVehicleParked++;
				distanceWalked += distanceFromPreviousStep;
				timeSinceRoundStarted ++;
				break;
			case WALKING_TO_DEPOT:
				timeSpentWalking++;
				distanceWalked += distanceFromPreviousStep;
				timeSinceRoundStarted ++;
				break;
			case WALKING_FROM_DEPOT:
				timeSpentWalking++;
				distanceWalked += distanceFromPreviousStep;
				timeSinceRoundStarted ++;
				break;
			case DELIVERING:
				timeSpentVehicleParked++;
				distanceWalked += distanceFromPreviousStep;
				timeSinceRoundStarted ++;
				break;
			case LOADING:
				
				break;
			case DEFAULT:
				
				break;
		default:
			break;
		}
	}
	
	void resetRoundStats() {
		timeSpentDriving = 0;
		timeSpentWalking = 0;
		timeSpentDrivingStem = 0;
		distanceDriven = 0;
		distanceWalked = 0;
		distanceDrivenStem = 0;
		timeSpentVehicleParked = 0;
		timeSinceRoundStarted = 0;
	}
	
	void recordCurrentRoundStats() {
		/*
		 * row format:
		 * roundId, driverId, roundSequenceId, roundDuration (s), driveTime (s), walkTime (s), parkTime (s), driveDistance (m), walkDistance (m),
		 * totalDriverDistance (m), stemDistance (m), stemTime (s), succesfulJobs, unsuccessfulJobs 
		 */
		String roundId = shortID + "-" + overallRoundIndex;
		double totalDistanceCovered = distanceDriven+distanceWalked;
		int unsuccessfulJobs = parcels.size();
		if(myVehicle != null)
			unsuccessfulJobs += myVehicle.parcels.size();
		
		String r = roundId+","+shortID+","+overallRoundIndex+","+timeSinceRoundStarted+","+timeSpentDriving+","+timeSpentWalking+","+timeSpentVehicleParked;
		r += ","+distanceDriven+","+distanceWalked+","+totalDistanceCovered+","+distanceDrivenStem+","+timeSpentDrivingStem;
		r += ","+(roundParcelsCount - unsuccessfulJobs)+","+unsuccessfulJobs;
		roundStats.add(r);
		
	}
	
	public void setStatus(DriverUtilities.driverStates e){
		this.currentDriverState = e;
	}
}
