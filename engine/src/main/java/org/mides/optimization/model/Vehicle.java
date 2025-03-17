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

    @JsonProperty("capacity")
    private double capacity;

    @NotNull
    @JsonProperty("depot_start")
    private Depot depotStart;

    @NotNull
    @JsonProperty("depot_end")
    private Depot depotEnd;
}
