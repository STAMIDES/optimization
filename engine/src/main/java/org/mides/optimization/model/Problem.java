package org.mides.optimization.model;

import com.fasterxml.jackson.annotation.JsonIgnore;
import com.fasterxml.jackson.annotation.JsonProperty;
import jakarta.validation.constraints.NotNull;
import lombok.AccessLevel;
import lombok.Data;
import lombok.NoArgsConstructor;
import lombok.Setter;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

@Data
@NoArgsConstructor
public class Problem {

    @NotNull
    @JsonProperty("depot")
    private Depot depot;

    @NotNull
    @JsonProperty("vehicles")
    private List<Vehicle> vehicles = new ArrayList<>();

    @NotNull
    @JsonProperty("ride_requests")
    private List<RideRequest> rideRequests = new ArrayList<>();

    @JsonIgnore
    @Setter(AccessLevel.PRIVATE)
    private int numberOfNodes;

    @JsonIgnore
    @Setter(AccessLevel.PRIVATE)
    private Map<Integer, PickupDeliveryTask> tasksByIndex = new HashMap<>();

    public void initialize() {
        depot.setIndex(0);
        tasksByIndex.put(0, new PickupDeliveryTask(
            0,
            depot.getAddress(),
            depot.getTimeWindow(),
            depot.getCoordinates()
        ));

        int index = 1;
        for (RideRequest ride : rideRequests) {
            ride.getPickup().setRide(ride);
            ride.getDelivery().setRide(ride);

            ride.getPickup().setIndex(index);
            ride.getDelivery().setIndex(index + 1);

            tasksByIndex.put(index, ride.getPickup());
            tasksByIndex.put(index + 1, ride.getDelivery());

            index += 2;

            if (ride.getPickup().getTimeWindow().startSeconds() > ride.getPickup().getTimeWindow().endSeconds())
                throw new RuntimeException(
                    String.format("Error with time window from Ride ID %s, Pickup Item", ride.getId())
                );

            if (ride.getDelivery().getTimeWindow().startSeconds() > ride.getDelivery().getTimeWindow().endSeconds())
                throw new RuntimeException(
                    String.format("Error with time window from Ride ID %s, Delivery Item", ride.getId())
                );
        }

        numberOfNodes = rideRequests.size() * 2 + 1;
    }

    public List<Coordinate> getAllCoordinates() {
        return new ArrayList<>(tasksByIndex.values().stream()
            .map(PickupDeliveryTask::getCoordinates)
            .toList());
    }
}
