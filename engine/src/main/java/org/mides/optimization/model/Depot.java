package org.mides.optimization.model;

import com.fasterxml.jackson.annotation.JsonIgnore;
import com.fasterxml.jackson.annotation.JsonProperty;
import jakarta.validation.constraints.NotNull;
import lombok.Data;
import lombok.NoArgsConstructor;
import org.springframework.validation.annotation.Validated;

@Data
@NoArgsConstructor
@Validated
public class Depot {

    @NotNull
    @JsonProperty("id")
    private String id;

    @NotNull
    @JsonProperty("coordinates")
    private Coordinate coordinates;

    @JsonProperty("time_window")
    private TimeWindow timeWindow = new TimeWindow();

    @JsonProperty("address")
    private String address;

    @JsonIgnore
    private int index;
}