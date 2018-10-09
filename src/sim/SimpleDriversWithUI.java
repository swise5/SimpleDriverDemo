package sim;

import java.awt.Color;

import javax.swing.JFrame;

import sim.display.Console;
import sim.display.Controller;
import sim.display.Display2D;
import sim.display.GUIState;
import sim.engine.SimState;
import sim.portrayal.geo.GeomPortrayal;
import sim.portrayal.geo.GeomVectorFieldPortrayal;
import sim.portrayal.grid.ObjectGridPortrayal2D;
import sim.portrayal.grid.SparseGridPortrayal2D;
import sim.portrayal.simple.OvalPortrayal2D;

public class SimpleDriversWithUI extends GUIState {

	private GeomVectorFieldPortrayal roads = new GeomVectorFieldPortrayal();
	private GeomVectorFieldPortrayal buildings = new GeomVectorFieldPortrayal();
	private GeomVectorFieldPortrayal agents = new GeomVectorFieldPortrayal();

	//SparseGridPortrayal2D driversPortrayal = new SparseGridPortrayal2D ();
	public Display2D display;
	public JFrame displayFrame;

	
	
	public SimpleDriversWithUI(SimState state) {
		super(state);
	}
	
	public SimpleDriversWithUI(){
		super(new SimpleDrivers(System.currentTimeMillis()));
	}
	
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
		roads.setPortrayalForAll(new GeomPortrayal(new Color(100,100,100, 50), 5, false));
		
		buildings.setField(world.buildingLayer);
		buildings.setPortrayalForAll(new GeomPortrayal(new Color(150,150,150), true));
		buildings.setImmutableField(true);
		
		
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
		display.attach(agents, "Agents");
		
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
	public static String getName() { return "Steve"; }
	
	public static void main(String [] args){
		(new SimpleDriversWithUI()).createController();
	}
}