package org.mides.optimization.model;

import com.fasterxml.jackson.annotation.JsonInclude;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.databind.annotation.JsonSerialize;
import lombok.Data;
import lombok.NoArgsConstructor;
import org.mides.optimization.converter.DurationSerializer;

import java.time.Duration;
import java.util.ArrayList;
import java.util.List;

@Data
@NoArgsConstructor
public class Route {

    @JsonProperty("vehicle_id")
    private String vehicleId = "";

    @JsonProperty("distance")
    private double distance;

    @JsonProperty("duration")
    @JsonSerialize(using = DurationSerializer.class)
    private Duration duration;

    @JsonProperty("visits")
    private List<Visit> visits = new ArrayList<>();

    @JsonProperty("geometry")
    private List<List<Double>> geometry = new ArrayList<>();

    @JsonProperty("time_window")
    private TimeWindow timeWindow = new TimeWindow();

    @JsonProperty("rest_time_window")
    @JsonInclude(JsonInclude.Include.NON_NULL) // Only include in JSON if not null
    private TimeWindow restTimeWindow;

}