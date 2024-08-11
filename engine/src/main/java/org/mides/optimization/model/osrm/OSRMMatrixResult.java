package org.mides.optimization.model.osrm;

import com.fasterxml.jackson.annotation.JsonInclude;
import lombok.Data;
import java.util.List;

@Data
@JsonInclude(JsonInclude.Include.NON_NULL)
public class OSRMMatrixResult {
    private String code;
    private String message;
    private List<List<Double>> distances;
    private List<List<Double>> durations;
}
