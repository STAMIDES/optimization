// engine/src/main/java/org/mides/optimization/model/Vehicle.java
package org.mides.optimization.model;

import com.fasterxml.jackson.annotation.JsonProperty;
import jakarta.validation.constraints.Min; // Import Min constraint
import jakarta.validation.constraints.NotNull;
import lombok.Data;
import lombok.NoArgsConstructor;

@Data
@NoArgsConstructor
public class Vehicle {

    @JsonProperty("id")
    private String id;

    @NotNull // Keep validation for total capacity
    @Min(0) // Ensure capacity is not negative
    @JsonProperty("capacity")
    private double capacity; // Total capacity (seats + potential wheelchair spots used by non-wheelchair)

    @NotNull // Add validation if wheelchair_capacity is mandatory
    @Min(0) // Ensure wheelchair capacity is not negative
    @JsonProperty("wheel_chair_capacity")
    private double wheelchairCapacity = 0; // Default to 0 if not provided in JSON

    @NotNull
    @JsonProperty("depot_start")
    private Depot depotStart;

    @NotNull
    @JsonProperty("depot_end")
    private Depot depotEnd;
}