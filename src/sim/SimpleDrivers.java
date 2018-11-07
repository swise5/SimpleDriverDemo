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

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.CoordinateSequenceFilter;
import com.vividsolutions.jts.geom.Envelope;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.GeometryFactory;
import com.vividsolutions.jts.geom.LineString;
import com.vividsolutions.jts.geom.Point;
import com.vividsolutions.jts.linearref.LengthIndexedLine;

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
	public static double resolution = 5;// the granularity of the simulation 
				// (fiddle around with this to merge nodes into one another)

	public static double speed_pedestrian = 7;
	public static double speed_vehicle = 10;

	public static int loadingTime = 20;
	public static int deliveryTime = 3;
	public static int approxManifestSize = 100;

	public static int numParcels = 10000;
	public static double probFailedDelivery = .1;
	
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
			InputCleaning.readInVectorLayer(buildingLayer, dirName + "buildings.shp", "buildings", new Bag());
			InputCleaning.readInVectorLayer(dummyDepotLayer, dirName + "depots.shp", "depots", new Bag());
			InputCleaning.readInVectorLayer(roadLayer, dirName + "roadsMajor.shp", "road network", new Bag());
			InputCleaning.readInVectorLayer(parkingLayer, dirName + "parking.shp", "road network", new Bag());
						
			//////////////////////////////////////////////
			////////////////// CLEANUP ///////////////////
			//////////////////////////////////////////////

			// standardize the MBRs so that the visualization lines up
			
			MBR = buildingLayer.getMBR();
			MBR.init(525044, 535806, 176259, 184098);

			heatmap = new GeomGridField();
			heatmap.setMBR(MBR);
			heatmap.setGrid(new IntGrid2D((int)(MBR.getWidth() / 100), (int)(MBR.getHeight() / 100), 0));

			
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


			/////////////////////
			///////// Clean up roads for Persons to use ///////////
			/////////////////////
						
			Network majorRoads = RoadNetworkUtilities.extractMajorRoads(roads);
			RoadNetworkUtilities.testNetworkForIssues(majorRoads);

			// assemble list of secondary versus local roads
			ArrayList <Edge> myEdges = new ArrayList <Edge> ();
			GeomVectorField secondaryRoadsLayer = new GeomVectorField(grid_width, grid_height);
			GeomVectorField localRoadsLayer = new GeomVectorField(grid_width, grid_height);
			for(Object o: majorRoads.allNodes){
				
				majorRoadNodesLayer.addGeometry((GeoNode)o);
				
				for(Object e: roads.getEdges(o, null)){
					Edge ed = (Edge) e;
					
					myEdges.add(ed);
										
					String type = ((MasonGeometry)ed.getInfo()).getStringAttribute("class");
					if(type.equals("secondary"))
							secondaryRoadsLayer.addGeometry((MasonGeometry) ed.getInfo());
					else if(type.equals("local"))
							localRoadsLayer.addGeometry((MasonGeometry) ed.getInfo());					
				}
			}

			System.gc();
			

			// set up depots
			setupDepots(dummyDepotLayer);
			
			// reset MBRs in case they got messed up during all the manipulation
		
			buildingLayer.setMBR(MBR);
			roadLayer.setMBR(MBR);			
			networkLayer.setMBR(MBR);
			networkEdgeLayer.setMBR(MBR);
			majorRoadNodesLayer.setMBR(MBR);
			deliveryLocationLayer.setMBR(MBR);
			agentLayer.setMBR(MBR);
			parkingLayer.setMBR(MBR);

			
			System.out.println("done");

			
			//////////////////////////////////////////////
			////////////////// AGENTS ///////////////////
			//////////////////////////////////////////////

			for(Object o: depotLayer.getGeometries()){
				Depot d = (Depot) o;
				generateRandomParcels(d);
				d.generateRounds();
			}

			agents.addAll(DriverUtilities.setupDriversAtDepots(this, fa, 10));
			for(Driver p: agents){
				agentLayer.addGeometry(p);
				Vehicle v = new Vehicle(p.geometry.getCoordinate(), p);
				p.assignVehicle(v);
			}

		

			// set up the agents in the simulation
/*			setupPersonsFromFile(dirName + agentFilename);
			agentsLayer.setMBR(MBR);
			
			// for each of the Persons, set up relevant, environment-specific information
			int aindex = 0;
			for(Person a: agents){
				
				if(a.familiarRoadNetwork == null){
					
					// the Person knows about major roads
					Network familiar = majorRoads.cloneGraph();

					// connect the major network to the Person's location
					connectToMajorNetwork(a.getNode(), familiar);
					
					a.familiarRoadNetwork = familiar;

					// add local roads into the network
					for(Object o: agentsLayer.getObjectsWithinDistance(a, 50)){
						Person b = (Person) o;
						if(b == a || b.familiarRoadNetwork != null || b.getNode() != a.getNode()) continue;
						b.familiarRoadNetwork = familiar.cloneGraph();
					}

				}
				
				// connect the Person's work into its personal network
				if(a.getWork() != null)
					connectToMajorNetwork(getClosestGeoNode(a.getWork()), a.familiarRoadNetwork);
				
				// set up its basic paths (fast and quicker and recomputing each time)
				a.setupPaths();

				if(aindex % 100 == 0){ // print report of progress
					System.out.println("..." + aindex + " of " + agents.size());
				}
				aindex++;
			}
*/

			// seed the simulation randomly
			seedRandom(System.currentTimeMillis());

		} catch (Exception e) { e.printStackTrace();}
    }
	
	public void setupDepots(GeomVectorField dummyDepots){
		Bag depots = dummyDepots.getGeometries();
		for(Object o: depots){
			MasonGeometry mg = (MasonGeometry) o;
			int numbays = mg.getIntegerAttribute("loadbays");
			GeoNode gn = snapPointToNode(mg.geometry.getCoordinate());
			
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
			
//			GeoNode gn = (GeoNode) roadNodes.get(random.nextInt(roadNodes.size()));
//			Coordinate myc = gn.getGeometry().getCoordinate();
			
			if(!MBR.contains(myc)){
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