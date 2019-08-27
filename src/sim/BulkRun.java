package sim;

import java.util.ArrayList;

import utilities.ParameterSamplingSpace;
import utilities.SamplingParameter;

public class BulkRun{
	
	public static void main(String [] args){
		
		int reps = 2;	// sim repetitions for parameter set
		int latinHypercubeSampleSize = 5; // number of subdivisions for each cube dimension/parameter
		
		ParameterSamplingSpace paramSamplingSpace = new ParameterSamplingSpace();
		
		paramSamplingSpace.AddParameter(new SamplingParameter("loadingTime", true, 10*4, 30*4));
		paramSamplingSpace.AddParameter(new SamplingParameter("approxManifestSize", true, 15, 30));
		paramSamplingSpace.AddParameter(new SamplingParameter("parkingRadius", false, 150, 500));
		paramSamplingSpace.AddParameter(new SamplingParameter("numParcels", true, 2000, 4000));
		
		paramSamplingSpace.SetupLatinHypercube(latinHypercubeSampleSize);
		
		for(int i = 0; i < reps; i++){
			for (int j = 0; j < latinHypercubeSampleSize; j++) {
				ArrayList<Double> paramSet = paramSamplingSpace.GetRunParameters(j);
				System.out.println(paramSet);
				
				SimpleDrivers simpleDrivers = new SimpleDrivers(System.currentTimeMillis());
				
				simpleDrivers.writeModelStatsToFile = false;
				
				simpleDrivers.loadingTime = (int)Math.round(paramSet.get(0));
				simpleDrivers.approxManifestSize = (int)Math.round(paramSet.get(1));
				simpleDrivers.parkingRadius = paramSet.get(2);
				simpleDrivers.numParcels = (int)Math.round(paramSet.get(3));
				
				System.out.println("Loading...");

				simpleDrivers.start();

				System.out.println("Running...");

				while(simpleDrivers.schedule.getTime() < 5760 && simpleDrivers.schedule.getTime() < simpleDrivers.schedule.AFTER_SIMULATION){
					simpleDrivers.schedule.step(simpleDrivers);
				}
				
				simpleDrivers.finish();
				
				System.out.println("...run finished");
			}
			
			paramSamplingSpace.RandomizeLatinHypercube();
		}
	}
}