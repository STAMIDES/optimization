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

    // No longer needed at the problem level, as it's per-vehicle now
    // @NotNull
    // @JsonProperty("depot")
    // private Depot depot;

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
        // depot.setIndex(0);  // No longer a single depot
        // tasksByIndex.put(0, new PickupDeliveryTask( // No longer needed
        //     0,
        //     depot.getAddress(),
        //     depot.getTimeWindow(),
        //     depot.getCoordinates(),
        //     depot.getId()
        // ));

        int index = 0; // Start from 0, as we don't have a global depot anymore
        for (Vehicle vehicle : vehicles) {
            //Initialize Start Depot
            vehicle.getDepotStart().setIndex(index);
            tasksByIndex.put(index, new PickupDeliveryTask(
                index,
                vehicle.getDepotStart().getAddress(),
                vehicle.getDepotStart().getTimeWindow(),
                vehicle.getDepotStart().getCoordinates(),
                vehicle.getDepotStart().getId()
            ));
            index++;

              //Initialize End Depot
            vehicle.getDepotEnd().setIndex(index);
            tasksByIndex.put(index, new PickupDeliveryTask(
                index,
                vehicle.getDepotEnd().getAddress(),
                vehicle.getDepotEnd().getTimeWindow(),
                vehicle.getDepotEnd().getCoordinates(),
                vehicle.getDepotEnd().getId()
            ));
            index++;

        }


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

        numberOfNodes = index; // rideRequests.size() * 2 + 1; // Updated for per-vehicle depots
    }

    public List<Coordinate> getAllCoordinates() {
        return new ArrayList<>(tasksByIndex.values().stream()
            .map(PickupDeliveryTask::getCoordinates)
            .toList());
    }
}