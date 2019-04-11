package objects;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Map.Entry;

import sim.SimpleDrivers;
import sim.engine.SimState;
import sim.engine.Steppable;
import sim.engine.Stoppable;
import sim.field.network.Edge;
import sim.util.Bag;
import sim.util.geo.MasonGeometry;
import swise.agents.TrafficAgent;
import swise.objects.network.GeoNode;
import swise.objects.network.ListEdge;

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
	
	public Driver(SimpleDrivers world, Coordinate c){
		super(c);
		homeBase = (Coordinate) c.clone();
		this.world = world;
		parcels = new ArrayList <Parcel> ();
		
		speed = world.speed_vehicle;
		
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
	}
	
	public void startRoundClock(){
		roundStartTime = world.schedule.getTime();
		roundDriveDistance = 0;
		roundWalkDistance = 0;
	}
	
	
	boolean attemptDelivery(double time){
		
		// failed delivery ):
		if (world.random.nextDouble() < world.probFailedDelivery) { 
			System.out.println(
					this.toString() + " has NOT been able to deliver parcel " + currentDelivery.toString());
			miniRoundIndex++;
		} 
		
		// successful delivery!
		else { 
			System.out.println("mydist: " + geometry.getCoordinate().distance(currentDelivery.deliveryLocation));
			currentDelivery.deliver(world.fa.createPoint(currentDelivery.deliveryLocation));
			//this.removeParcel(currentDelivery);
			miniRound.remove(miniRoundIndex);
			removeParcel(currentDelivery);
			System.out.println(this.toString() + " has delivered parcel " + currentDelivery.toString());
			//currentDelivery.geometry = world.fa.createPoint(currentDelivery.deliveryLocation);
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
	}
	
	void enterVehicle(){
		if(myVehicle == null){
			System.out.println("ERROR: do not have a vehicle!");
			return;
		}
		
		inVehicle = true;
		myVehicle.setDriver(this);
	}
	
	void scheduleNextGoal(Coordinate c){
		headFor(c);
		world.schedule.scheduleOnce(this);
	}
	
	@Override
	public void step(SimState arg0) {
		
		
		double time = world.schedule.getTime(); // find the current time
		
		// make sure the round has been defined, and update it if not
		if(this.myRound == null){
			updateRoundClustered();
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
				exitVehicle();
				world.schedule.scheduleOnce(this);
				return;
			}

			else { // a wild problem appears!
				System.out.println("ERROR: problem with driver behaviours for " + this.toString());
				return;
			}			
		}

		// otherwise if we've got to the next miniround starting point, transition to doing a miniround!
		else if(roundIndex < myRound.size() && 
				myRound.get(roundIndex).geometry.distance(this.geometry) <= world.resolution && 
				miniRoundIndex < 0){
			miniRound = parkingPerRound.get(myRound.get(roundIndex));
			//myRound.remove(roundIndex); // clean it up - we'll not come here again
			miniRoundIndex = 0;
			
			world.schedule.scheduleOnce(this);
			return;
		}
		
		// otherwise if you've been trying to get in the vehicle and are close enough, get in
		else if(miniRound == null && myVehicle != null && !inVehicle &&
				myVehicle.getLocation().distance(geometry.getCoordinate()) <= world.resolution){
			enterVehicle();
			world.schedule.scheduleOnce(this);
			return;
		}


		// HOUSEKEEPING ON MINIROUND
		
		// if anything left to deliver on this miniround, deliver it
		if(miniRound != null && miniRoundIndex < miniRound.size()){
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
			return;
		}
		
		// HOUSEKEEPING ON ROUND
		
		// otherwise, is there another organising point to hit up? If so, go to it
		if(roundIndex < myRound.size()){
			scheduleNextGoal(myRound.get(roundIndex).geometry.getCoordinate());
			return;
		}
		// the round is finished! Back to the depot with you!
		else if(homeBase.distance(geometry.getCoordinate()) > world.resolution) { 
			scheduleNextGoal(homeBase);
			return;
		}
		//  otherwise, you're at the Depot - enter it
		else {
			cleanupAtDepot();
		}

	}

	void cleanupAtDepot(){
		
		// write out the report
		double roundTime = world.schedule.getTime() - roundStartTime;
		history.add(this.toString() + "\t" + roundTime + "\t" + roundDriveDistance + "\t" + roundWalkDistance);
		System.out.println(this.toString() + " is done with the round! It took " + (world.schedule.getTime() - roundStartTime));
		
		// transfer all undelivered parcels
		Bag b = world.depotLayer.getObjectsWithinDistance(geometry, world.resolution);
		if(b.size() > 0){
			Depot d = (Depot) b.get(0);
			d.enterDepot(this);
			if(parcels.size() > 0){
				System.out.println("Round finished - driver " + this.toString() + " has returned with " + parcels.size());
				transferTo(parcels, d);
				if(myVehicle != null)
					myVehicle.transferTo(myVehicle.parcels, d);
			}
		}
		
		// reset everything
		roundIndex = 0;
		miniRoundIndex = -1;
		parkingPerRound = null;
		myRound = null;
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

	// really basic right now: start with the first one and greedily pick the next closest, until you have them all
/*	public void updateRound(){
		if(parcels.size() <= 1 && (myVehicle != null && myVehicle.parcels.size() <= 1)) return;
		
		//ArrayList <Parcel> tempParcels = new ArrayList <Parcel> (parcels);
		
		if(myVehicle != null){
			for(Parcel p: myVehicle.parcels){
				ArrayList <Parcel> vParcels = new ArrayList <Parcel> ();
				vParcels.add(p);
				tempParcels.put(p, vParcels);
			}
		}
		
		MasonGeometry [] tempys = tempParcels.keySet().toArray(new MasonGeometry [tempParcels.size()]);
		for(int i = 1; i < tempParcels.size(); i++){
			MasonGeometry p = tempys[i - 1];
			double dist = Double.MAX_VALUE;
			int best = -1;
			
			for(int j = i; j < tempParcels.size(); j++){
				Parcel pj = (Parcel) tempys[j];
				double pjdist = pj.deliveryLocation.distance(((Parcel) p).deliveryLocation);
				if(pjdist < dist){
					dist = pjdist;
					best = j;
				}
			}
			
			// store the next closest
			ArrayList <Parcel> tempPs = new ArrayList <Parcel> ();
			tempPs.add((Parcel) tempys[best]);
			parkingPerRound.put(tempys[best], tempPs);
			myRound.add(tempys[best]);
			
			// replace the elements
			MasonGeometry transfer = tempys[i];
			tempys[i] = tempys[best];
			tempys[best] = transfer;			
		}
	}*/
	
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
			
			// look for covering parking spaces
			Bag b = world.parkingCatchmentLayer.getCoveringObjects(world.fa.createPoint(p.getDeliveryLocation()));
			
			// if there are no nearby parking spaces, we'll plan around the delivery itself
			if(b.size() == 0){
				MasonGeometry mg = new MasonGeometry(world.fa.createPoint(p.getDeliveryLocation()));
				ArrayList <Parcel> ps = new ArrayList <Parcel>();
				ps.add(p);
				parkingSpaceOptions.put(mg, ps);
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
		while(allTempParcels.size() > 0 && iter.hasNext()){
			
			// get the next biggest one
			Entry <MasonGeometry, ArrayList <Parcel>> nextOne = (Entry <MasonGeometry, ArrayList <Parcel>>)
					iter.next();
			
			ArrayList <Parcel> toDeliver = nextOne.getValue();
			toDeliver.retainAll(allTempParcels);
			
			if(toDeliver.size() <= 0)
				continue;
			
//			for(Parcel p: toDeliver)
//				allTempParcels.remove(p);
			allTempParcels.removeAll(toDeliver);
			myRound.add(nextOne.getKey());
			parkingPerRound.put(nextOne.getKey(), toDeliver);
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
				parcels.removeAll(ps);
				b.addParcels(ps);
				for(Parcel p: ps)
					p.transfer(this, b);
			}
			else {
				parcels.remove((Parcel) o);
				b.addParcel((Parcel) o);
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

	
	public double calculateDistance(ArrayList <Edge> edges){
		double result = 0;
		for(Edge e: edges){
			result += ((MasonGeometry)e.info).geometry.getLength();
		}
		return result;
	}
	
	public void assignVehicle(Vehicle v){
		myVehicle = v;
		inVehicle = true;
	}
	
	public ArrayList <String> getHistory() {return history; }
}