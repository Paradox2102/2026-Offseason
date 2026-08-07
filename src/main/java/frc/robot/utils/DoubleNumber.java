package frc.robot.utils;

public class DoubleNumber {
    private final double minBound;
    private final double maxBound;
    
    
    public DoubleNumber(double minimum, double maximum){
        minBound = minimum;
        maxBound = maximum;
    }

    public static boolean isBetween(double value, DoubleNumber bounds){
        return ((value > bounds.minBound) && (value < bounds.maxBound)); 
    }
    
}
