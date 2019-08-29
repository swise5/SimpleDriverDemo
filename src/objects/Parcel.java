package objects;

import java.util.ArrayList;
import java.util.UUID;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;

import sim.SimpleDrivers;
import swise.agents.MobileAgent;

public class Parcel extends MobileAgent {

	UUID parcelUID; //agent unique ID

	Burdenable carryingUnit = null;
	Coordinate deliveryLocation;
	double dim_x, dim_y, dim_z, weight;
	ArrayList <String> history;
	int status; // 0 = undelivered, 1 = failed delivery attempt, 2 = out for delivery, 3 = delivered
	
	String roundId = ""; // For when we want to assign parcels to specific rounds
	
	public Parcel(Burdenable carrier){
		super((Coordinate) carrier.getLocation());
		carryingUnit = carrier;
		history = new ArrayList <String> ();
		carrier.addParcel(this);
		isMovable = true;		
		parcelUID = UUID.randomUUID();
	}
	
	public void setDeliveryLocation(Coordinate c){
		deliveryLocation = (Coordinate) c.clone();
	}
	
	public Coordinate getDeliveryLocation(){ return deliveryLocation; }
	
	public void setRoundId(String _roundId) {
		roundId = _roundId;
	}
	
	public String getRoundId() { return roundId; }
	
	public Coordinate getLocation(){
		if(carryingUnit != null) return carryingUnit.getLocation();
		else return this.geometry.getCoordinate();
	}
	
	public boolean transfer(Burdenable from, Burdenable to){
		try {
			from.removeParcel(this);
			to.addParcel(this);
			carryingUnit = to;
			if(carryingUnit == null)
				System.out.println("transferred to null");
			return true;			
		} catch (Exception e){
			e.printStackTrace();
			return false;			
		}
	}
	
	public boolean deliver(Geometry targetGeo){
		if(carryingUnit == null)
			System.out.println("ERROR: delivered parcel has no assigned carrying unit - " + toString());
		else if(carryingUnit.getLocation().distance(deliveryLocation) <= SimpleDrivers.resolution){
			this.geometry = targetGeo;
			carryingUnit.removeParcel(this);
			carryingUnit = null;
			status = 3;
//			System.out.println("delivered: " + myId);
			updateLoc(targetGeo.getCoordinate());
			return true;
		}
		System.out.println("ERROR: delivered parcel too far from the delivery location - " + toString());
		status = 1;
		return false;
	}
	
	public boolean equals(Object o){
		if(!(o instanceof Parcel))
			return false;
		Parcel p = (Parcel) o;
		return parcelUID == p.parcelUID;
	}
}