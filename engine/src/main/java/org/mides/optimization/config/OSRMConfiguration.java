package org.mides.optimization.config;

import lombok.Data;
import org.springframework.boot.context.properties.ConfigurationProperties;
import org.springframework.stereotype.Component;

@Data
@Component
@ConfigurationProperties(prefix = "osrm")
public class OSRMConfiguration {
    private String baseUrl;
    private String matrixEndpoint;
    private String matrixParams;
    private String routeEndpoint;
    private String routeParams;
}
