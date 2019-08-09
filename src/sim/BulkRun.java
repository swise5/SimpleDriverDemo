package sim;

public class BulkRun{
	
	public static void main(String [] args){
		for(int j = 0; j < 10; j++){
			SimpleDrivers simpleDrivers = new SimpleDrivers(System.currentTimeMillis());
			
			System.out.println("Loading...");

			simpleDrivers.start();

			System.out.println("Running...");

			while(simpleDrivers.schedule.getTime() < 5760 && simpleDrivers.schedule.getTime() < simpleDrivers.schedule.AFTER_SIMULATION){
				simpleDrivers.schedule.step(simpleDrivers);
			}
			
			simpleDrivers.finish();
			
			System.out.println("...run finished");
		}
	}
}