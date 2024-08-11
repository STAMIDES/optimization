package org.mides.optimization.model;

import com.fasterxml.jackson.annotation.JsonProperty;
import lombok.Data;
import lombok.NoArgsConstructor;

@Data
@NoArgsConstructor
public class Vehicle {

    @JsonProperty("id")
    private String id;

    @JsonProperty("capacity")
    private double capacity;

    @JsonProperty("time_window")
    private TimeWindow timeWindow = new TimeWindow();
}
