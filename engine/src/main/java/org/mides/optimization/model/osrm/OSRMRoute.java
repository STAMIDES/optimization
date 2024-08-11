package org.mides.optimization.model.osrm;

import com.fasterxml.jackson.annotation.JsonInclude;
import com.fasterxml.jackson.annotation.JsonProperty;
import lombok.Data;

import java.util.ArrayList;
import java.util.List;

@Data
@JsonInclude(JsonInclude.Include.NON_NULL)
public class OSRMRoute {
    private Double distance;
    private Double duration;
    private Double weight;
    private String geometry;

    @JsonProperty("weight_name")
    private String weightName;

    public List<List<Double>> decodeGeometry() {
        if (geometry == null || geometry.isEmpty()) {
            throw new IllegalArgumentException("geometry");
        }

        char[] polylineChars = geometry.toCharArray();
        int index = 0;

        int currentLat = 0;
        int currentLng = 0;

        List<List<Double>> result = new ArrayList<>();

        while (index < polylineChars.length) {
            int sum = 0;
            int shifter = 0;
            int nextFiveBits;
            do {
                nextFiveBits = polylineChars[index++] - 63;
                sum |= (nextFiveBits & 31) << shifter;
                shifter += 5;
            } while (nextFiveBits >= 32 && index < polylineChars.length);

            if (index >= polylineChars.length) {
                break;
            }

            currentLat += (sum & 1) == 1 ? ~(sum >> 1) : (sum >> 1);

            sum = 0;
            shifter = 0;
            do {
                nextFiveBits = polylineChars[index++] - 63;
                sum |= (nextFiveBits & 31) << shifter;
                shifter += 5;
            } while (nextFiveBits >= 32 && index < polylineChars.length);

            if (index >= polylineChars.length && nextFiveBits >= 32) {
                break;
            }

            currentLng += (sum & 1) == 1 ? ~(sum >> 1) : (sum >> 1);

            List<Double> point = new ArrayList<>();
            point.add((double) currentLng / 1E5);
            point.add((double) currentLat / 1E5);
            result.add(point);
        }

        return result;
    }
}
