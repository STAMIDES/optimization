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

        int index = 0;
        for (Vehicle vehicle : vehicles) {
            vehicle.getDepotStart().setIndex(index);
            tasksByIndex.put(index, new PickupDeliveryTask(
                index,
                vehicle.getDepotStart().getAddress(),
                vehicle.getDepotStart().getTimeWindow(),
                vehicle.getDepotStart().getCoordinates(),
                vehicle.getDepotStart().getId()
            ));
            index = index + 1;

            vehicle.getDepotEnd().setIndex(index);
            tasksByIndex.put(index, new PickupDeliveryTask(
                index,
                vehicle.getDepotEnd().getAddress(),
                vehicle.getDepotEnd().getTimeWindow(),
                vehicle.getDepotEnd().getCoordinates(),
                vehicle.getDepotEnd().getId()
            ));
            index = index + 1;
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

        numberOfNodes = rideRequests.size() * 2 + vehicles.size() * 2;
    }

    public List<Coordinate> getAllCoordinates() {
        return new ArrayList<>(tasksByIndex.values().stream()
            .map(PickupDeliveryTask::getCoordinates)
            .toList());
    }

    public long[] getDemands() {
        if (numberOfNodes == 0)
            throw new RuntimeException("Problem not initialized");

        var vehicleCapacities = new long[tasksByIndex.size()];

        /* Set demand for depots to 0 */
        for (int i = 0; i < vehicles.size() * 2; i++) {
            vehicleCapacities[i] = 0;
        }

        /* Pickup adds 1 and delivery subtracts 1. If it has companion add/subtract 2 */
        int counter = vehicles.size() * 2;
        for (RideRequest ride : rideRequests) {
            if (ride.isHasCompanion()) {
                vehicleCapacities[counter] = 2; // Pickup
                vehicleCapacities[counter + 1] = -2; // Delivery
            } else {
                vehicleCapacities[counter] = 1; // Pickup
                vehicleCapacities[counter + 1] = -1; // Delivery
            }
            counter += 2;
        }

        return vehicleCapacities;
    }

    public List<Integer> getWheelchairIndexes() {
        if (numberOfNodes == 0)
            throw new RuntimeException("Problem not initialized");

        var result = new ArrayList<Integer>();
        int counter = vehicles.size() * 2;
        for (RideRequest ride : rideRequests) {
            if (ride.isWheelchairRequired()) {
                result.add(counter); // Pickup
                result.add(counter + 1); // Delivery
            }
            counter += 2;
        }
        return result;
    }

    public long[] getNoWheelchairVehicleIndexes() {
        if (numberOfNodes == 0)
            throw new RuntimeException("Problem not initialized");

        var result = new ArrayList<Integer>();
        int counter = 0;
        for (var vehicle : vehicles) {
            if (!vehicle.isHasWheelchair())
                result.add(counter);
        }
        long[] resultArray = new long[result.size()];
        for (int i = 0; i < result.size(); i++) {
            resultArray[i] = result.get(i);
        }

        return resultArray;
    }
}
