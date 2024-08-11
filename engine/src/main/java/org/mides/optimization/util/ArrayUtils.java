package org.mides.optimization.util;

import java.util.List;

public class ArrayUtils {

    public static long[][] convertTo2DLongArray(List<List<Double>> list) {
        if (list == null || list.isEmpty()) {
            throw new IllegalArgumentException("Input list cannot be null or empty");
        }

        int rows = list.size();
        int cols = list.get(0).size();
        long[][] result = new long[rows][cols];

        for (int i = 0; i < rows; i++) {
            if (list.get(i).size() != cols) {
                throw new IllegalArgumentException("All inner lists must have the same number of elements");
            }
            for (int j = 0; j < cols; j++) {
                result[i][j] = (long) (list.get(i).get(j) * 1);
            }
        }

        return result;
    }

    public static long[][] convertTo2DLongArray(List<List<Double>> list, int multiplier) {
        if (list == null || list.isEmpty()) {
            throw new IllegalArgumentException("Input list cannot be null or empty");
        }

        int rows = list.size();
        int cols = list.get(0).size();
        long[][] result = new long[rows][cols];

        for (int i = 0; i < rows; i++) {
            if (list.get(i).size() != cols) {
                throw new IllegalArgumentException("All inner lists must have the same number of elements");
            }
            for (int j = 0; j < cols; j++) {
                result[i][j] = (long) (list.get(i).get(j) * multiplier);
            }
        }

        return result;
    }
}
