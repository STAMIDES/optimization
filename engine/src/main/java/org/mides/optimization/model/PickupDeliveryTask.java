package org.mides.optimization.model;

import com.fasterxml.jackson.annotation.JsonIgnore;
import com.fasterxml.jackson.annotation.JsonProperty;
import jakarta.validation.constraints.NotNull;
import lombok.Data;
import lombok.NoArgsConstructor;

@Data
@NoArgsConstructor
public class PickupDeliveryTask {

    @NotNull
    @JsonProperty("coordinates")
    private Coordinate coordinates;

    @JsonProperty("address")
    private String address;

    @JsonProperty("time_window")
    private TimeWindow timeWindow = new TimeWindow();

    @JsonIgnore
    private int index;

    @JsonIgnore
    private RideRequest ride;

    public PickupDeliveryTask(int index, String address, TimeWindow timeWindow, Coordinate coordinates) {
        this.index = index;
        this.address = address;
        this.timeWindow = timeWindow;
        this.coordinates = coordinates;
    }
}
