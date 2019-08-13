package sim;

import umontreal.ssj.hups.*;
import umontreal.ssj.rng.WELL512;

public class LhcTest {
	public static void main(String [] args){
		int n = 4;
		int d = 2;
		LatinHypercube lhc = new LatinHypercube(n, d);
		lhc.randomize(new WELL512());
		lhc.randomize(new WELL512());
//		lhc.addRandomShift();
//		lhc.se
		System.out.println(lhc.toString());
		for (int i = 0; i < d; i++) {
			for (int j = 0; j < n; j++) {
				double bob = lhc.getCoordinate(j, i);
				System.out.println("(" + i + "," + j + ") : " + bob);
			}
		}

	}

}
