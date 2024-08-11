package org.mides.optimization.model;

import com.fasterxml.jackson.annotation.JsonProperty;
import lombok.Data;
import lombok.NoArgsConstructor;

import java.util.ArrayList;
import java.util.List;

@Data
@NoArgsConstructor
public class Solution {

    @JsonProperty("routes")
    private List<Route> routes = new ArrayList<>();

    @JsonProperty("dropped_rides")
    private List<String> droppedRides = new ArrayList<>();
}