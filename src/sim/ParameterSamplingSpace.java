package sim;

import java.util.ArrayList;

import umontreal.ssj.hups.LatinHypercube;
import umontreal.ssj.rng.WELL512;

public class ParameterSamplingSpace {
	
	// using the LHC implementation from the SSJ library,
	// more details here:
	// http://simul.iro.umontreal.ca/ssjlab/doc/html/umontreal/iro/lecuyer/hups/LatinHypercube.html
	
	// in the context of this implementation, a parameter is essentially one of the LHC's dimensions
	
	LatinHypercube lhc;
	
	public ArrayList<SamplingParameter> parameterSpace;
	public ParameterSamplingSpace() {
		// TODO Auto-generated constructor stub
		parameterSpace = new ArrayList<SamplingParameter>(0);
	}
	
	public void AddParameter(SamplingParameter sp) {
		parameterSpace.add(sp);
	}
	
	public void SetupLatinHypercube(int numOfSamples) {
		lhc = new LatinHypercube(numOfSamples, parameterSpace.size());
		System.out.println(lhc.toString());
		lhc.randomize(new WELL512());
	}
	
	public void RandomizeLatinHypercube() {
		lhc.randomize(new WELL512());
	}
	
	public ArrayList<Double> GetRunParameters(int sampleIndex) {
		ArrayList<Double> values = new ArrayList<Double>(0);
		
		for (int i = 0; i < parameterSpace.size(); i++) {
			double coord = lhc.getCoordinate(sampleIndex, i);
			double vtemp = parameterSpace.get(i).GetParameterValueInSamplingRange(coord);
			values.add(vtemp);
		}
		
		return values;
	}

}
