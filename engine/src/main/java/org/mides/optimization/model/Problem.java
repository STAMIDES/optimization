// engine/src/main/java/org/mides/optimization/model/Problem.java
package org.mides.optimization.model;

import com.fasterxml.jackson.annotation.JsonIgnore;
import com.fasterxml.jackson.annotation.JsonProperty;
import jakarta.validation.constraints.NotNull;
import lombok.AccessLevel;
import lombok.Data;
import lombok.NoArgsConstructor;
import lombok.Setter;

import java.util.ArrayList;
import java.util.Comparator; // <--- ADD THIS IMPORT
import java.util.HashMap;
import java.util.List;
import java.util.Map;

@Data
@NoArgsConstructor
public class Problem {

    // Task Types Constants (recommended over raw strings)
    public static final String TASK_TYPE_DEPOT_START = "DEPOT_START";
    public static final String TASK_TYPE_DEPOT_END = "DEPOT_END";
    public static final String TASK_TYPE_PICKUP = "PICKUP";
    public static final String TASK_TYPE_DELIVERY = "DELIVERY";


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
        tasksByIndex.clear(); // Clear map before initializing
        int index = 0;

        // Process Vehicle Depots
        for (Vehicle vehicle : vehicles) {
            // Start Depot
            vehicle.getDepotStart().setIndex(index);
            PickupDeliveryTask startDepotTask = new PickupDeliveryTask(
                index,
                vehicle.getDepotStart().getAddress(),
                vehicle.getDepotStart().getTimeWindow(),
                vehicle.getDepotStart().getCoordinates(),
                vehicle.getDepotStart().getId() // Use depot ID as stop ID?
            );
            startDepotTask.setType(TASK_TYPE_DEPOT_START); // Set type
            tasksByIndex.put(index, startDepotTask);
            index++; // Increment index

            // End Depot
            vehicle.getDepotEnd().setIndex(index);
             PickupDeliveryTask endDepotTask = new PickupDeliveryTask(
                index,
                vehicle.getDepotEnd().getAddress(),
                vehicle.getDepotEnd().getTimeWindow(),
                vehicle.getDepotEnd().getCoordinates(),
                vehicle.getDepotEnd().getId() // Use depot ID as stop ID?
            );
            endDepotTask.setType(TASK_TYPE_DEPOT_END); // Set type
            tasksByIndex.put(index, endDepotTask);
            index++; // Increment index
        }

        // Process Ride Requests
        for (RideRequest ride : rideRequests) {
            // Link task back to ride
            ride.getPickup().setRide(ride);
            ride.getDelivery().setRide(ride);

            // Assign indices
            ride.getPickup().setIndex(index);
            ride.getDelivery().setIndex(index + 1);

            // Set Task Types
            ride.getPickup().setType(TASK_TYPE_PICKUP);
            ride.getDelivery().setType(TASK_TYPE_DELIVERY);

            // Add to map
            tasksByIndex.put(index, ride.getPickup());
            tasksByIndex.put(index + 1, ride.getDelivery());

            index += 2; // Increment index by 2 (for pickup and delivery)

            // Validate time windows (existing logic is good)
            if (ride.getPickup().getTimeWindow() != null && // Add null check
                ride.getPickup().getTimeWindow().startSeconds() > ride.getPickup().getTimeWindow().endSeconds())
                throw new RuntimeException(
                    String.format("Error with time window from Ride ID %s, Pickup Item: start > end", ride.getId())
                );

            if (ride.getDelivery().getTimeWindow() != null && // Add null check
                ride.getDelivery().getTimeWindow().startSeconds() > ride.getDelivery().getTimeWindow().endSeconds())
                throw new RuntimeException(
                    String.format("Error with time window from Ride ID %s, Delivery Item: start > end", ride.getId())
                );
        }

        numberOfNodes = index; // Total number of nodes is the final index value
    }

    public List<Coordinate> getAllCoordinates() {
        // Ensure consistent ordering based on index for matrix generation
        List<PickupDeliveryTask> sortedTasks = new ArrayList<>(tasksByIndex.values());
        // Sort by index - Requires Comparator import
        sortedTasks.sort(Comparator.comparingInt(PickupDeliveryTask::getIndex));

        return sortedTasks.stream()
            .map(PickupDeliveryTask::getCoordinates)
            .toList();
    }

    // Helper method to get task by index (already present via getTasksByIndex())

    // Helper method to get the first index after depots (useful for loops)
    @JsonIgnore
    public int getFirstTaskNodeIndex() {
        return vehicles.size() * 2;
    }
}
