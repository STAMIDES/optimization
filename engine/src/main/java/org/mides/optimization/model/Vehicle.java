package org.mides.optimization.model;

import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.databind.annotation.JsonDeserialize;
import jakarta.validation.constraints.NotNull;
import lombok.Data;
import lombok.NoArgsConstructor;

import java.util.HashSet;
import java.util.Set;

@Data
@NoArgsConstructor
public class Vehicle {

    @JsonProperty("id")
    private String id;

    @JsonProperty("seat_capacity")
    private double seatCapacity;

    @JsonProperty("wheelchair_capacity")
    private double wheelchairCapacity;

    @JsonProperty("time_window")
    private TimeWindow timeWindow = new TimeWindow();

    @NotNull
    @JsonProperty("depot_start")
    private Depot depotStart;

    @NotNull
    @JsonProperty("depot_end")
    private Depot depotEnd;

    @JsonProperty("supported_characteristics")
    @JsonDeserialize(as = HashSet.class)
    private Set<String> supportedCharacteristics = new HashSet<>();

    @JsonProperty("active_ride_id_pre_boarded")
    private String activeRideIdPreBoarded;
}
