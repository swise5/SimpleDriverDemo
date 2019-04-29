package sim;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.io.InputStreamReader;
import java.nio.channels.FileLock;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map.Entry;

import sim.engine.SimState;
import sim.engine.Steppable;
import sim.field.geo.GeomGridField;
import sim.field.geo.GeomGridField.GridDataType;
import sim.field.geo.GeomVectorField;
import sim.field.grid.Grid2D;
import sim.field.grid.IntGrid2D;
import sim.field.network.Edge;
import sim.field.network.Network;
import sim.io.geo.ShapeFileImporter;
import sim.io.geo.ArcInfoASCGridImporter;
import sim.util.Bag;
import sim.util.geo.AttributeValue;
import sim.util.geo.GeomPlanarGraph;
import sim.util.geo.MasonGeometry;
import sim.util.geo.PointMoveTo;
import swise.agents.communicator.Communicator;
import swise.agents.communicator.Information;
import swise.disasters.Wildfire;
import swise.objects.AStar;
import swise.objects.NetworkUtilities;
import swise.objects.PopSynth;
import swise.objects.network.GeoNode;
import swise.objects.network.ListEdge;
import utilities.DepotUtilities;
import utilities.DriverUtilities;
import swise.objects.InputCleaning;
import swise.objects.RoadNetworkUtilities;

import com.vividsolutions.jts.awt.PointShapeFactory;
import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.CoordinateSequenceFilter;
import com.vividsolutions.jts.geom.Envelope;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.GeometryFactory;
import com.vividsolutions.jts.geom.LineString;
import com.vividsolutions.jts.geom.Point;
import com.vividsolutions.jts.linearref.LengthIndexedLine;
import com.vividsolutions.jts.util.GeometricShapeFactory;

import ec.util.MersenneTwisterFast;
import objects.Depot;
import objects.Driver;
import objects.Parcel;
import objects.Vehicle;

/**
 * SimpleDriversDemo is an example of a simple ABM framework to explore different
 * modes of delivery in Central London 
 * 
 * @author swise
 *
 */
public class SimpleDrivers extends SimState {

	/////////////// Model Parameters ///////////////////////////////////
	
	private static final long serialVersionUID = 1L;
	public static int grid_width = 800;
	public static int grid_height = 500;
	public static double resolution = 2;// the granularity of the simulation 
				// (fiddle around with this to merge nodes into one another)

	public static double speed_pedestrian = 30;
	public static double speed_vehicle = 100;

	public static int loadingTime = 20;
	public static int deliveryTime = 3;
	public static int approxManifestSize = 100;
	public static double parkingRadius = 300;

	public static int numParcels = 1000;
	public static double probFailedDelivery = .126;
	
	/////////////// Data Sources ///////////////////////////////////////
	
	String dirName = "data/";
	
	//// END Data Sources ////////////////////////
	
	/////////////// Containers ///////////////////////////////////////

//	public GeomVectorField baseLayer = new GeomVectorField(grid_width, grid_height);
	public GeomVectorField roadLayer = new GeomVectorField(grid_width, grid_height);
	public GeomVectorField depotLayer = new GeomVectorField(grid_width, grid_height);
	public GeomVectorField buildingLayer = new GeomVectorField(grid_width, grid_height);
	public GeomVectorField deliveryLocationLayer = new GeomVectorField(grid_width, grid_height);
	public GeomVectorField agentLayer = new GeomVectorField(grid_width, grid_height);
	public GeomVectorField parkingLayer = new GeomVectorField(grid_width, grid_height);
	public GeomVectorField parkingCatchmentLayer = new GeomVectorField(grid_width, grid_height);
	
	
	public GeomVectorField networkLayer = new GeomVectorField(grid_width, grid_height);
	public GeomVectorField networkEdgeLayer = new GeomVectorField(grid_width, grid_height);	
	public GeomVectorField majorRoadNodesLayer = new GeomVectorField(grid_width, grid_height);

	public GeomGridField heatmap = new GeomGridField();
	public Bag roadNodes = new Bag();
	public Network roads = new Network(false);

	/////////////// End Containers ///////////////////////////////////////

	/////////////// Objects //////////////////////////////////////////////

	public ArrayList <Driver> agents = new ArrayList <Driver> (10);
	ArrayList <ArrayList <Parcel>> rounds;
	
	public GeometryFactory fa = new GeometryFactory();
	
	long mySeed = 0;
	
	Envelope MBR = null;
	
	boolean verbose = false;
	
	/////////////// END Objects //////////////////////////////////////////

	///////////////////////////////////////////////////////////////////////////
	/////////////////////////// BEGIN functions ///////////////////////////////
	///////////////////////////////////////////////////////////////////////////	
	
	/**
	 * Default constructor function
	 * @param seed
	 */
	public SimpleDrivers(long seed) {
		super(seed);
		random = new MersenneTwisterFast(12345);
	}


	/**
	 * Read in data and set up the simulation
	 */
	public void start()
    {
		super.start();
		try {
			
			//////////////////////////////////////////////
			///////////// READING IN DATA ////////////////
			//////////////////////////////////////////////
		
			GeomVectorField dummyDepotLayer = new GeomVectorField(grid_width, grid_height);
			InputCleaning.readInVectorLayer(buildingLayer, dirName + "colBuildings.shp", "buildings", new Bag());
			InputCleaning.readInVectorLayer(dummyDepotLayer, dirName + "depots.shp", "depots", new Bag());
			InputCleaning.readInVectorLayer(roadLayer, dirName + "roadsCoL.shp", "road network", new Bag());
			
			GeomVectorField rawParkingLayer = new GeomVectorField(grid_width, grid_height);
			InputCleaning.readInVectorLayer(rawParkingLayer, dirName + "parkingBaysEC3_merged.shp", "road network", new Bag());
						
			//////////////////////////////////////////////
			////////////////// CLEANUP ///////////////////
			//////////////////////////////////////////////

			//MBR = roadLayer.getMBR();
			MBR = buildingLayer.getMBR();
//			MBR.init(525044, 535806, 178959, 186798);
			//MBR.init(531000, 534000, 180000, 182400);
			
			buildingLayer.setMBR(MBR);
			roadLayer.setMBR(MBR);			
			networkLayer.setMBR(MBR);
			networkEdgeLayer.setMBR(MBR);
			majorRoadNodesLayer.setMBR(MBR);
			deliveryLocationLayer.setMBR(MBR);
			agentLayer.setMBR(MBR);
			parkingLayer.setMBR(MBR);
			parkingCatchmentLayer.setMBR(MBR);
			
			System.out.println("done");

			
			// clean up the road network
			
			System.out.print("Cleaning the road network...");
			
			roads = NetworkUtilities.multipartNetworkCleanup(roadLayer, roadNodes, resolution, fa, random, 0);
			roadNodes = roads.getAllNodes();
			RoadNetworkUtilities.testNetworkForIssues(roads);
			
			// set up roads as being "open" and assemble the list of potential terminii
			roadLayer = new GeomVectorField(grid_width, grid_height);
			for(Object o: roadNodes){
				GeoNode n = (GeoNode) o;
				networkLayer.addGeometry(n);
				
				// check all roads out of the nodes
				for(Object ed: roads.getEdgesOut(n)){
					
					// set it as being (initially, at least) "open"
					ListEdge edge = (ListEdge) ed;
					((MasonGeometry)edge.info).addStringAttribute("open", "OPEN");
					networkEdgeLayer.addGeometry( (MasonGeometry) edge.info);
					roadLayer.addGeometry((MasonGeometry) edge.info);
					((MasonGeometry)edge.info).addAttribute("ListEdge", edge);
					
				}
			}
			
			roadLayer.setMBR(MBR);

			// set up depots
			setupDepots(dummyDepotLayer);
			
			//////////////////////////////////////////////
			////////////////// AGENTS ///////////////////
			//////////////////////////////////////////////


			// set up parking info
			GeometricShapeFactory shapeFactory = new GeometricShapeFactory();
			shapeFactory.setNumPoints(16);
			shapeFactory.setSize(parkingRadius);
			for(Object o: rawParkingLayer.getGeometries()){

				MasonGeometry mg = (MasonGeometry) o;

				// snap parking layer to road network
				Coordinate c = snapPointToRoadNetwork(mg.geometry.getCoordinate());
				MasonGeometry newPS = new MasonGeometry(fa.createPoint(c));
				parkingLayer.addGeometry(newPS);

				// set up the catchments
				shapeFactory.setCentre(newPS.geometry.getCoordinate());
				MasonGeometry myParkingArea = new MasonGeometry(shapeFactory.createCircle());
				myParkingArea.addAttribute("parkingspace", newPS);
				parkingCatchmentLayer.addGeometry(myParkingArea);
			}

			// generate parcels at each of those depots
			for(Object o: depotLayer.getGeometries()){
				Depot d = (Depot) o;
				generateRandomParcels(d);
				//generateRandomParcelsInArea(d, parkingArea);
				d.generateRounds();
			}

			agents.addAll(DriverUtilities.setupDriversAtDepots(this, fa, 1));
			for(Driver p: agents){
				agentLayer.addGeometry(p);
				Vehicle v = new Vehicle(p.geometry.getCoordinate(), p);
				p.assignVehicle(v);
			}


			// seed the simulation randomly
			//seedRandom(System.currentTimeMillis());


			MBR.init(530000, 534500, 179500, 182500);
			
			buildingLayer.setMBR(MBR);
			roadLayer.setMBR(MBR);			
			networkLayer.setMBR(MBR);
			networkEdgeLayer.setMBR(MBR);
			majorRoadNodesLayer.setMBR(MBR);
			deliveryLocationLayer.setMBR(MBR);
			agentLayer.setMBR(MBR);
			parkingLayer.setMBR(MBR);
			parkingCatchmentLayer.setMBR(MBR);
			
			
		} catch (Exception e) { e.printStackTrace();}
    }
	
	public void setupDepots(GeomVectorField dummyDepots){
		Bag depots = dummyDepots.getGeometries();
		for(Object o: depots){
			MasonGeometry mg = (MasonGeometry) o;
			int numbays = mg.getIntegerAttribute("loadbays");
			GeoNode gn = snapPointToNode(mg.geometry.getCoordinate());
			
			if(gn == null)
				continue;
			Depot d = new Depot(gn.geometry.getCoordinate(), numbays, this);
			d.setNode(gn);

			depotLayer.addGeometry(d);
			schedule.scheduleOnce(d);
		}
	}
	
	public Coordinate snapPointToRoadNetwork(Coordinate c) {
		ListEdge myEdge = null;
		double resolution = this.resolution;

		if (networkEdgeLayer.getGeometries().size() == 0)
			return null;

		while (myEdge == null && resolution < Double.MAX_VALUE) {
			myEdge = RoadNetworkUtilities.getClosestEdge(c, resolution, networkEdgeLayer, fa);
			resolution *= 10;
		}
		if (resolution == Double.MAX_VALUE)
			return null;

		LengthIndexedLine closestLine = new LengthIndexedLine(
				(LineString) (((MasonGeometry) myEdge.info).getGeometry()));
		double myIndex = closestLine.indexOf(c);
		return closestLine.extractPoint(myIndex);
	}
	
	public GeoNode snapPointToNode(Coordinate c){
		ListEdge myEdge = null;
		double resolution = this.resolution;

		if (networkEdgeLayer.getGeometries().size() == 0)
			return null;

		while (myEdge == null && resolution < Double.MAX_VALUE) {
			myEdge = RoadNetworkUtilities.getClosestEdge(c, resolution, networkEdgeLayer, fa);
			resolution *= 10;
		}
		if (resolution == Double.MAX_VALUE)
			return null;

		double distFrom = c.distance(((GeoNode)myEdge.from()).geometry.getCoordinate()),
				distTo = c.distance(((GeoNode)myEdge.to()).geometry.getCoordinate());
		if(distFrom <= distTo)
			return (GeoNode) myEdge.from();
		else
			return (GeoNode) myEdge.to();
	}
	
	public static ListEdge getClosestEdge(Coordinate c, double resolution, GeomVectorField networkEdgeLayer, GeometryFactory fa){

		// find the set of all edges within *resolution* of the given point
		Bag objects = networkEdgeLayer.getObjectsWithinDistance(fa.createPoint(c), resolution);
		if(objects == null || networkEdgeLayer.getGeometries().size() <= 0) 
			return null; // problem with the network edge layer
		
		Point point = fa.createPoint(c);
		
		// find the closest edge among the set of edges
		double bestDist = resolution;
		ListEdge bestEdge = null;
		for(Object o: objects){
			double dist = ((MasonGeometry)o).getGeometry().distance(point);
			if(dist < bestDist){
				bestDist = dist;
				bestEdge = (ListEdge) ((AttributeValue) ((MasonGeometry) o).getAttribute("ListEdge")).getValue();
			}
		}
		
		// if it exists, return it
		if(bestEdge != null)
			return bestEdge;
		
		// otherwise return failure
		else
			return null;
	}
	
	public void generateRandomParcels(Depot d){
		
		ArrayList <Parcel> myParcels = new ArrayList <Parcel> ();
		Bag buildings = buildingLayer.getGeometries();
		
		for(int i = 0; i < numParcels; i++){
			
			Point deliveryLoc = ((MasonGeometry)buildings.get(random.nextInt(buildings.size()))).geometry.getCentroid();
			Coordinate myc = deliveryLoc.getCoordinate();
			Coordinate snappedC = snapPointToRoadNetwork(myc);
			
//			GeoNode gn = (GeoNode) roadNodes.get(random.nextInt(roadNodes.size()));
//			Coordinate myc = gn.getGeometry().getCoordinate();
			
//			if(!MBR.contains(myc)){
			if(!MBR.contains(snappedC)){
				i--;
				continue;
			}
			//Coordinate myc = new Coordinate(random.nextInt(myw) + myminx, random.nextInt(myh) + myminy);
					
			Parcel p = new Parcel(d);
			p.setDeliveryLocation(new Coordinate(snappedC.x, snappedC.y));
			myParcels.add(p);
		}		
	}

	public void generateRandomParcelsInArea(Depot d, Geometry g){
		
		ArrayList <Parcel> myParcels = new ArrayList <Parcel> ();
		Bag buildings = buildingLayer.getGeometries();
		
		for(int i = 0; i < numParcels; i++){
			
			Geometry buildingGeom = ((MasonGeometry)buildings.get(random.nextInt(buildings.size()))).geometry;
			Point deliveryLoc = buildingGeom.getCentroid();
			Coordinate myc = deliveryLoc.getCoordinate();
			
//			GeoNode gn = (GeoNode) roadNodes.get(random.nextInt(roadNodes.size()));
//			Coordinate myc = gn.getGeometry().getCoordinate();
			
			if(!MBR.contains(myc)){
				i--;
				continue;
			}
			else if(g.distance(buildingGeom) > 200){
				i--;
				continue;
			}
			//Coordinate myc = new Coordinate(random.nextInt(myw) + myminx, random.nextInt(myh) + myminy);
					
			Parcel p = new Parcel(d);
			p.setDeliveryLocation(myc);
			myParcels.add(p);			
		}		
	}

	/**
	 * Finish the simulation and clean up
	 */
	public void finish(){
		super.finish();
		try{
			
			// save the history
			BufferedWriter output = new BufferedWriter(new FileWriter(dirName + "output" + mySeed + ".txt"));
			
			for(Driver a: agents){
				for(String s: a.getHistory())
				output.write(s + "\n");
			}
			output.close();

		} catch (IOException e){
			e.printStackTrace();
		}
	}
	

	

	/**
	 * RoadClosure structure holds information about a road closure
	 */
	public class RoadClosure extends Information {
		public RoadClosure(Object o, long time, Object source) {
			super(o, time, source, 5);
		}
	}
	
	
	/** set the seed of the random number generator */
	void seedRandom(long number){
		random = new MersenneTwisterFast(number);
		mySeed = number;
	}

	/**
	 * To run the model without visualization
	 */
	public static void main(String[] args)
    {
		
		if(args.length < 0){
			System.out.println("usage error");
			System.exit(0);
		}
		
		SimpleDrivers simpleDrivers = new SimpleDrivers(System.currentTimeMillis());
		
		System.out.println("Loading...");

		simpleDrivers.start();

		System.out.println("Running...");

		for(int i = 0; i < 288 * 3; i++){
			simpleDrivers.schedule.step(simpleDrivers);
		}
		
		simpleDrivers.finish();
		
		System.out.println("...run finished");

		System.exit(0);
    }
}