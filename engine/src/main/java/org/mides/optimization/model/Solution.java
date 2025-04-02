// engine/src/main/java/org/mides/optimization/model/Solution.java
package org.mides.optimization.model;

import com.fasterxml.jackson.annotation.JsonInclude; // Import this
import com.fasterxml.jackson.annotation.JsonProperty;
import lombok.Data;
import lombok.NoArgsConstructor;

import java.util.ArrayList;
import java.util.List;

@Data
@NoArgsConstructor
@JsonInclude(JsonInclude.Include.NON_NULL) // Prevent null errorMessage from appearing in JSON
public class Solution {

    @JsonProperty("routes")
    private List<Route> routes = new ArrayList<>();

    @JsonProperty("dropped_rides")
    private List<String> droppedRides = new ArrayList<>();

    @JsonProperty("error_message") // Add this field
    private String errorMessage = null; // Initialize to null
}