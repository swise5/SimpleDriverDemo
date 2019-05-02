package sim;

import java.awt.Color;
import java.awt.Paint;

import javax.swing.JFrame;

import sim.display.Console;
import sim.display.Controller;
import sim.display.Display2D;
import sim.display.GUIState;
import sim.engine.SimState;
import sim.portrayal.SimplePortrayal2D;
import sim.portrayal.geo.GeomPortrayal;
import sim.portrayal.geo.GeomVectorFieldPortrayal;
import sim.portrayal.grid.ObjectGridPortrayal2D;
import sim.portrayal.grid.SparseGridPortrayal2D;
import sim.portrayal.simple.CircledPortrayal2D;
import sim.portrayal.simple.LabelledPortrayal2D;
import sim.portrayal.simple.OvalPortrayal2D;
import sim.portrayal3d.SimplePortrayal3D;
import sim.portrayal3d.simple.LabelledPortrayal3D;
import sim.util.gui.ColorMap;
import sim.util.gui.SimpleColorMap;
import swise.visualization.AttributePolyPortrayal;
import swise.visualization.SegmentedColorMap;

public class SimpleDriversWithUI extends GUIState {

	private GeomVectorFieldPortrayal roads = new GeomVectorFieldPortrayal();
	private GeomVectorFieldPortrayal buildings = new GeomVectorFieldPortrayal();
	private GeomVectorFieldPortrayal drivers = new GeomVectorFieldPortrayal();
	private GeomVectorFieldPortrayal deliveryLocations = new GeomVectorFieldPortrayal();
	private GeomVectorFieldPortrayal parkingLocations = new GeomVectorFieldPortrayal();
	
	private GeomVectorFieldPortrayal vehicles = new GeomVectorFieldPortrayal();

	private GeomVectorFieldPortrayal parkingCatchmentLocations = new GeomVectorFieldPortrayal();
	
	//SparseGridPortrayal2D driversPortrayal = new SparseGridPortrayal2D ();
	public Display2D display;
	public JFrame displayFrame;

	
	
	public SimpleDriversWithUI(SimState state) {
		super(state);
	}
	
	public SimpleDriversWithUI(){
		super(new SimpleDrivers(1234));//System.currentTimeMillis()));
	}
	
	public Object getSimulationInspectedObject() { return state; }
	
	/** Begins the simulation */
	public void start() {
		super.start();
		
		// set up portrayals
		setupPortrayals();
	}

	/** Loads the simulation from a point */
	public void load(SimState state) {
		super.load(state);
		
		// we now have new grids. Set up the portrayals to reflect that
		setupPortrayals();
	}
	
	public void setupPortrayals(){
		
		SimpleDrivers world = (SimpleDrivers) state;
		
		roads.setField(world.roadLayer);
		roads.setPortrayalForAll(new GeomPortrayal(new Color(100,100,100, 100), 2, false));
		roads.setImmutableField(true);
		
		buildings.setField(world.buildingLayer);
		buildings.setPortrayalForAll(new GeomPortrayal(new Color(150,150,150, 100), true));
		buildings.setImmutableField(true);
		
		deliveryLocations.setField(world.deliveryLocationLayer);
		double [] levels = new double [100];
		Color [] colors = new Color [100];
		for(int i = 0; i < 100; i++){
			levels[i] = i;
			Color mycol = Color.getHSBColor(world.random.nextFloat(), 1, 1);
			colors[i] = new Color(mycol.getRed(), mycol.getGreen(), mycol.getBlue(), 80);
		}
		SegmentedColorMap scm = new SegmentedColorMap(levels, colors);
		deliveryLocations.setPortrayalForAll(new AttributePolyPortrayal(
				scm,//new SimpleColorMap(0,100, Color.red, Color.green), 
				"round", new Color(0,0,0,0), true, 10));
		//agents.setImmutableField(true);
		
		parkingLocations.setField(world.parkingLayer);
		parkingLocations.setPortrayalForAll(new GeomPortrayal(new Color(50,200,50,200), 30, true));
		parkingLocations.setImmutableField(true);
		
		parkingCatchmentLocations.setField(world.parkingCatchmentLayer);
		parkingCatchmentLocations.setPortrayalForAll(new GeomPortrayal(new Color(250,100,100,50), 50, true));
		parkingCatchmentLocations.setImmutableField(true);
		
		drivers.setField(world.agentLayer);
//		drivers.setPortrayalForAll(new GeomPortrayal(new Color(255,150,150), 20));
		drivers.setPortrayalForAll(new CircledPortrayal2D(
				new LabelledPortrayal2D(
						new OvalPortrayal2D(
								new Color(255,150,150), 5),
						null, Color.white, false)));
		
		vehicles.setField(world.vehiclesLayer);
		vehicles.setPortrayalForAll(new GeomPortrayal(new Color(150,150,255), 15));
		
		display.reset();
		display.setBackdrop(new Color(10,10,10));

		// redraw the display
		display.repaint();
	}
	
	public void init(Controller c) {
		super.init(c);

		// the map visualization
		display = new Display2D((int)(SimpleDrivers.grid_width), (int)(SimpleDrivers.grid_height), this);

		display.attach(buildings, "Buildings");
		display.attach(roads, "Roads");
		display.attach(parkingCatchmentLocations, "Parkingthingy", false);
		display.attach(parkingLocations, "Parking", false);
		
		display.attach(vehicles, "Vehicles");
		
		display.attach(deliveryLocations, "Delivery Locations");
		display.attach(drivers, "Drivers");
		
		displayFrame = display.createFrame();
		c.registerFrame(displayFrame); // register the frame so it appears in the "Display" list
		displayFrame.setVisible(true);		
	}
	
	public void quit() {
		super.quit();

		if (displayFrame != null)
			displayFrame.dispose();
		displayFrame = null; // let gc
		display = null; // let gc
	}

	/** Returns the name of the simulation */
	public static String getName() { return "SimpleDriver"; }
	
	public static void main(String [] args){
		(new SimpleDriversWithUI()).createController();
	}
}