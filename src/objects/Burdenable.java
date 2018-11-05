package objects;

import java.util.ArrayList;

import com.vividsolutions.jts.geom.Coordinate;

public interface Burdenable {

	public void addParcel(Parcel p);
	public boolean removeParcel(Parcel p);
	public void addParcels(ArrayList <Parcel> ps);
	public boolean removeParcels(ArrayList <Parcel> ps);
	public Coordinate getLocation();
	public boolean transferTo(Object o, Burdenable b);
}
