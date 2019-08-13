package sim;

public class SamplingParameter {
	String id = "";
	boolean isInt = false;
	float minValue;
	float maxValue;
//	int dimInLHC;
	
	public SamplingParameter(String _id, boolean _isInt, float _minValue, float _maxValue) {
		// TODO Auto-generated constructor stub
		id = _id;
		isInt = _isInt;
		minValue = _minValue;
		maxValue = _maxValue;
//		dimInLHC = _dimInLHC;
		
		if(isInt) {
			minValue = (float) Math.floor(minValue);
			maxValue = (float) Math.floor(maxValue);
		}
	}
	
	public double getParameterValueInSamplingRange(double v) {
		double out;
		double vtemp = v * (maxValue - minValue);
		if(isInt) {
			vtemp = Math.floor(vtemp);
		}
		out = minValue + vtemp;
		return out;
	}
}
