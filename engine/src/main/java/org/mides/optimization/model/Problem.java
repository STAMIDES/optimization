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
                vehicle.getTimeWindow(),
                vehicle.getDepotStart().getCoordinates(),
                vehicle.getDepotStart().getId()
            ));
            index = index + 1;

            vehicle.getDepotEnd().setIndex(index);
            tasksByIndex.put(index, new PickupDeliveryTask(
                index,
                vehicle.getDepotEnd().getAddress(),
                vehicle.getTimeWindow(),
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

    public long[] getSeatDemands() {
        var seatDemands = new long[tasksByIndex.size()];

        /* Set demand for depots to 0 */
        for (int i = 0; i < vehicles.size() * 2; i++) {
            seatDemands[i] = 0;
        }

        /* Pickup adds and delivery subtracts. */
        int counter = vehicles.size() * 2;
        for (RideRequest ride : rideRequests) {
            int demand = 0;
            if (!ride.isWheelchairRequired())
                demand += 1;
            if (ride.isHasCompanion())
                demand += 1;

            seatDemands[counter] = demand; // Pickup
            seatDemands[counter + 1] = -demand; // Delivery

            counter += 2;
        }

        return seatDemands;
    }

    public long[] getWheelchairDemands() {
        var wheelchairDemands = new long[tasksByIndex.size()];

        /* Set demand for depots to 0 */
        for (int i = 0; i < vehicles.size() * 2; i++) {
            wheelchairDemands[i] = 0;
        }

        /* Pickup adds and delivery subtracts. */
        int counter = vehicles.size() * 2;
        for (RideRequest ride : rideRequests) {
            int demand = 0;
            if (ride.isWheelchairRequired())
                demand = 1;

            wheelchairDemands[counter] = demand; // Pickup
            wheelchairDemands[counter + 1] = -demand; // Delivery

            counter += 2;
        }

        return wheelchairDemands;
    }

    public long[] getSeatCapacities() {
        var seatCapacities = new long[vehicles.size()];

        for (int i = 0; i < vehicles.size(); i++) {
            seatCapacities[i] = (long)vehicles.get(i).getSeatCapacity();
        }

        return seatCapacities;
    }

    public long[] getWheelchairCapacities() {
        var wheelchairCapacities = new long[vehicles.size()];

        for (int i = 0; i < vehicles.size(); i++) {
            wheelchairCapacities[i] = (long)vehicles.get(i).getWheelchairCapacity();
        }

        return wheelchairCapacities;
    }
}
