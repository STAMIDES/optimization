package org.mides.optimization.model;

import lombok.Builder;
import lombok.Data;
import java.time.Duration;

@Data
@Builder
public class DepotDroppedRideInfo {
    private String rideId;
    private String userId;
    private Coordinate originalPickupCoordinates;
    private String originalPickupAddress;
    private Coordinate originalDeliveryCoordinates;
    private String originalDeliveryAddress;
    private String droppedAtDepotId; // The ID of the depot where the drop occurred
    private Coordinate droppedAtDepotCoordinates;
    private String vehicleIdDroppedBy;
    private Duration timeOfDropAtDepot; // Arrival time at the depot
}
