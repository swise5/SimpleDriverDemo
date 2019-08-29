package utilities;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Dictionary;
import java.util.Enumeration;
import java.util.HashMap;
import java.util.Map;
import java.util.Random;
import java.util.logging.Level;

//import com.srbenoit.math.delaunay.Delaunay;
//import com.srbenoit.math.delaunay.Vertex;
import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Envelope;

import objects.Depot;
import objects.Parcel;
import sim.field.geo.GeomVectorField;

public class DepotUtilities {
	
	public static ArrayList <ArrayList <Parcel>> gridDistribution (ArrayList <Parcel> parcels, GeomVectorField space, int targetSize){
		
		// establish the parameters on the space
		Envelope e = space.MBR;
		double num = parcels.size(), width = e.getWidth(), height = e.getHeight(), minx = e.getMinX(), miny = e.getMinY();
		
		// establish the number of rounds necessary
		int approxNumRounds = (int)(Math.ceil( num / targetSize )); // the number of parcels relative to the size of a round
		double roundsPerSide = Math.ceil(Math.sqrt(approxNumRounds)); // the approx area covered per round
	
		// set up a round for each category
		ArrayList <ArrayList <Parcel>> rounds = new ArrayList <ArrayList<Parcel>> (); // for safety??
		// Collections.nCopies((int)Math.ceil(roundsPerSide * roundsPerSide)+1, new ArrayList <Parcel> ())
		
		for(int i = 0; i < (int)Math.ceil(roundsPerSide * roundsPerSide)+1; i++){
			rounds.add(new ArrayList <Parcel> ());
		}
		
		// define the limits on the distances		
		double dx = width / roundsPerSide; // the width cut into those chunks
		double dy = height / roundsPerSide; // the height cut into those relative chunks
		int xperRow = (int)Math.floor(roundsPerSide);
		for(Parcel p: parcels){
			Coordinate c = p.getDeliveryLocation();
			int myX = (int)Math.floor((c.x - minx) / dx), myY = (int)Math.floor((c.y - miny) / dy);
			rounds.get(myX + myY * xperRow).add(p);
			p.addIntegerAttribute("round", myX + myY * xperRow);
		}
		
		Collections.sort(rounds, (o1, o2) ->  ((Integer)o1.size()).compareTo(o2.size()));
		for(int i = 0; i < rounds.size(); i++){
			if(rounds.get(i).size() > 0)
				return new ArrayList <ArrayList<Parcel>> (rounds.subList(i, rounds.size()));
		}
		return new ArrayList <ArrayList<Parcel>> ();
	}
	
	public static ArrayList<ArrayList<Parcel>> definedDistribution(ArrayList<Parcel> parcels, Depot d){
		Map<String, ArrayList<Parcel>> roundsById = new HashMap<String, ArrayList<Parcel>>();
		
		for (Parcel p : parcels) {
			String rid = p.getRoundId();
			if(!rid.equals("")) {
				if(roundsById.containsKey(rid)) {	//roundId has been instantiated in the dictionary, add this parcel to the list
					roundsById.get(rid).add(p);
				}
				else {	//roundId has not been instantiated in the dictionary yet, create new entry and add this parcel
					roundsById.put(rid, new ArrayList<Parcel>());
					roundsById.get(rid).add(p);
				}
				
				int roundIdInt = java.util.Arrays.asList(java.util.Arrays.asList(roundsById.keySet()).get(0).toArray()).indexOf(p.getRoundId());
				roundIdInt += d.getId();
				p.addIntegerAttribute("round", roundIdInt);
			}
		}
		
		System.out.println(java.util.Arrays.asList(roundsById.keySet()).get(0));
		ArrayList<ArrayList<Parcel>> rounds = new ArrayList<ArrayList<Parcel>>();
		for(String key : roundsById.keySet()) {
			rounds.add(roundsById.get(key));
		}
		
		Collections.sort(rounds, (o1, o2) ->  ((Integer)o1.size()).compareTo(o2.size()));
		for(int i = 0; i < rounds.size(); i++){
			if(rounds.get(i).size() > 0)
				return new ArrayList <ArrayList<Parcel>> (rounds.subList(i, rounds.size()));
		}
		return new ArrayList <ArrayList<Parcel>> ();
	}
	
}