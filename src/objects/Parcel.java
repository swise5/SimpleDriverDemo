package objects;

import java.util.ArrayList;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;

import sim.SimpleDrivers;
import swise.agents.MobileAgent;

public class Parcel extends MobileAgent {

	Burdenable carryingUnit = null;
	Coordinate deliveryLocation;
	double dim_x, dim_y, dim_z, weight;
	ArrayList <String> history;
	int status; // 0 = undelivered, 1 = failed delivery attempt, 2 = out for delivery, 3 = delivered
	
	public Parcel(Burdenable carrier){
		super((Coordinate) carrier.getLocation());
		carryingUnit = carrier;
		history = new ArrayList <String> ();
		carrier.addParcel(this);
		isMovable = true;
	}
	
	public void setDeliveryLocation(Coordinate c){
		deliveryLocation = (Coordinate) c.clone();
	}
	
	public Coordinate getDeliveryLocation(){ return deliveryLocation; }
	
	public Coordinate getLocation(){
		if(carryingUnit != null) return carryingUnit.getLocation();
		else return this.geometry.getCoordinate();
	}
	
	public boolean transfer(Burdenable from, Burdenable to){
		try {
			from.removeParcel(this);
			to.addParcel(this);
			carryingUnit = to;
			return true;			
		} catch (Exception e){
			e.printStackTrace();
			return false;			
		}
	}
	
	public boolean deliver(Geometry targetGeo){
		if(carryingUnit.getLocation().distance(deliveryLocation) <= SimpleDrivers.resolution){
			// TODO make it move away!!
			this.geometry = targetGeo;
			carryingUnit.removeParcel(this);
			carryingUnit = null;
			status = 3;
			updateLoc(targetGeo.getCoordinate());
			return true;
		}
		status = 1;
		return false;
	}
	
}