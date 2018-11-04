package objects;

import com.vividsolutions.jts.geom.Coordinate;

public interface Burdenable {

	public void addParcel(Parcel p);
	public boolean removeParcel(Parcel p);
	public Coordinate getLocation();
}
