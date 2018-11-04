package utilities;

import java.io.BufferedReader;
import java.io.FileInputStream;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map.Entry;

import com.vividsolutions.jts.geom.Point;

import objects.Driver;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.GeometryFactory;

import sim.SimpleDrivers;
import sim.engine.Schedule;
import sim.engine.SimState;
import sim.engine.Steppable;
import sim.field.geo.GeomVectorField;
import sim.util.Bag;
import sim.util.geo.MasonGeometry;
import swise.objects.InputCleaning;
import swise.objects.PopSynth;

public class DriverUtilities {
	
	public static synchronized ArrayList<Driver> setupDriversAtRandom(GeomVectorField buildings, Schedule schedule, SimpleDrivers world, 
			GeometryFactory fa, int numDrivers){
		
		ArrayList <Driver> agents = new ArrayList <Driver> ();
		Bag myBuildings = buildings.getGeometries();
		int myBuildingsSize = myBuildings.numObjs;
		
		for(int i = 0; i < numDrivers; i++){
			
			Object o = myBuildings.get(world.random.nextInt(myBuildingsSize));
			MasonGeometry mg = (MasonGeometry) o;
			while(mg.geometry.getArea() > 1000){
				o = myBuildings.get(world.random.nextInt(myBuildingsSize));
				mg = (MasonGeometry) o;
			}
			//Point myPoint = mg.geometry.getCentroid();
			//Coordinate myC = new Coordinate(myPoint.getX(), myPoint.getY());
			Coordinate myC = (Coordinate) mg.geometry.getCoordinate().clone();
			Driver a = new Driver(myC);
			agents.add(a);
			
			world.schedule.scheduleOnce(a);
		}
		
		return agents;
	}
	

	
}