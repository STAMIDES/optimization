package org.mides.optimization.model;

import com.fasterxml.jackson.annotation.JsonProperty;
import jakarta.validation.constraints.NotNull;
import lombok.Data;
import lombok.NoArgsConstructor;

@Data
@NoArgsConstructor
public class Vehicle {

    @JsonProperty("id")
    private String id;

    @JsonProperty("seat_capacity")
    private double seatCapacity;

    @JsonProperty("wheelchair_capacity")
    private double wheelchairCapacity;

    @NotNull
    @JsonProperty("depot_start")
    private Depot depotStart;

    @NotNull
    @JsonProperty("depot_end")
    private Depot depotEnd;
}
