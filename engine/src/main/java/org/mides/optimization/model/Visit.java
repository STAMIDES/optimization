package org.mides.optimization.model;

import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.databind.annotation.JsonDeserialize;
import com.fasterxml.jackson.databind.annotation.JsonSerialize;
import lombok.Data;
import lombok.NoArgsConstructor;
import org.mides.optimization.converter.DurationSerializer;
import org.mides.optimization.converter.RideDirectionDeserializer;
import org.mides.optimization.converter.RideDirectionSerializer;

import java.time.Duration;

@Data
@NoArgsConstructor
public class Visit {

    @JsonProperty("position")
    private int position;

    @JsonProperty("ride_id")
    private String rideId;

    @JsonProperty("user_id")
    private String userId;

    @JsonProperty("ride_direction")
    @JsonSerialize(using = RideDirectionSerializer.class)
    @JsonDeserialize(using = RideDirectionDeserializer.class)
    private RideDirection rideDirection;

    @JsonProperty("address")
    private String address;

    @JsonProperty("coordinates")
    private Coordinate coordinates;

    @JsonProperty("arrival_time")
    @JsonSerialize(using = DurationSerializer.class)
    private Duration arrivalTime;

    @JsonProperty("waiting_time")
    @JsonSerialize(using = DurationSerializer.class)
    private Duration waitingTime;

    @JsonProperty("solution_window")
    private TimeWindow solutionWindow = new TimeWindow();
}
