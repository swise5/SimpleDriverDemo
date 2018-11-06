package objects;

import java.util.ArrayList;

import com.vividsolutions.jts.geom.Coordinate;

import sim.util.geo.MasonGeometry;
import swise.agents.MobileAgent;

public class Vehicle extends MobileAgent implements Burdenable {

	Driver owner;
	ArrayList <Parcel> parcels = new ArrayList <Parcel> ();
	
	public Vehicle(Coordinate c, Driver d){
		super(c);
		isMovable = true;
		owner = d;
	}
	
	@Override
	public void addParcel(Parcel p) {
		parcels.add(p);
	}

	@Override
	public boolean removeParcel(Parcel p) {
		return parcels.remove(p);
	}

	@Override
	public void addParcels(ArrayList<Parcel> ps) {
		parcels.addAll(ps);
	}

	@Override
	public boolean removeParcels(ArrayList<Parcel> ps) {
		return parcels.removeAll(ps);
	}

	@Override
	public Coordinate getLocation() {
		return geometry.getCoordinate();
	}

	@Override
	public boolean transferTo(Object o, Burdenable b) {
		try{
			if(o instanceof ArrayList){
				parcels.removeAll((ArrayList <Parcel>) o);
				b.addParcels((ArrayList <Parcel>) o);
			}
			else {
				parcels.remove((Parcel) o);
				b.addParcel((Parcel) o);
			}
			return true;
		} catch (Exception e){
			return false;
		}
	}
	
}