package sim;

public class BulkRun{
	
	public static void main(String [] args){
		for(int j = 0; j < 10; j++){
			SimpleDrivers simpleDrivers = new SimpleDrivers(System.currentTimeMillis());
			
			System.out.println("Loading...");

			simpleDrivers.start();

			System.out.println("Running...");

			for(int i = 0; i < 5760; i++){//288 * 3; i++){
				simpleDrivers.schedule.step(simpleDrivers);
			}
			
			simpleDrivers.finish();
			
			System.out.println("...run finished");
		}
	}
}