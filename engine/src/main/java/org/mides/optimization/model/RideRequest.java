package org.mides.optimization.model;

import jakarta.validation.constraints.NotNull;
import lombok.NoArgsConstructor;
import org.mides.optimization.converter.RideDirectionDeserializer;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.databind.annotation.JsonDeserialize;
import lombok.Data;

@Data
@NoArgsConstructor
public class RideRequest {

    @NotNull
    @JsonProperty("id")
    private String id;

    @NotNull
    @JsonProperty("user_id")
    private String userId;

    @NotNull
    @JsonProperty("has_companion")
    private boolean hasCompanion;

    @NotNull
    @JsonProperty("wheelchair_required")
    private boolean wheelchairRequired;

    @NotNull
    @JsonProperty("pickup")
    private PickupDeliveryTask pickup;

    @NotNull
    @JsonProperty("delivery")
    private PickupDeliveryTask delivery;

    @NotNull
    @JsonProperty("direction")
    @JsonDeserialize(using = RideDirectionDeserializer.class)
    private RideDirection direction;

    @Override
    public String toString() {
        return id;
    }
}
