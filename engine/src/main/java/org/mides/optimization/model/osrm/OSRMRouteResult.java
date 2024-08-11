package org.mides.optimization.model.osrm;

import com.fasterxml.jackson.annotation.JsonInclude;
import lombok.Data;
import java.util.List;

@Data
@JsonInclude(JsonInclude.Include.NON_NULL)
public class OSRMRouteResult {
    private String code;
    private String message;
    private List<OSRMRoute> routes;
}
