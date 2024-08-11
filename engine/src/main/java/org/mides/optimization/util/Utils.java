package org.mides.optimization.util;

public class Utils {
    public static double convertDistanceBack(long distance) {
        /* Converts @distance from the internal format to km */
        double result = (double) distance / Constants.DISTANCE_MULTIPLIER / 1000;
        String formattedResult = String.format("%.3f", result);
        return Double.parseDouble(formattedResult);
    }
}
