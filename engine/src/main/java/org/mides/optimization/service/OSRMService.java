package org.mides.optimization.service;

import org.mides.optimization.config.OSRMConfiguration;
import org.mides.optimization.exception.osrm.QueryMatrixException;
import org.mides.optimization.exception.osrm.QueryRouteException;
import org.mides.optimization.model.Coordinate;
import org.mides.optimization.model.osrm.OSRMMatrixResult;
import org.mides.optimization.model.osrm.OSRMRouteResult;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.http.MediaType;
import org.springframework.stereotype.Service;
import org.springframework.web.client.RestClient;

import java.util.List;
import java.util.Objects;
import java.util.stream.Collectors;

@Service
public class OSRMService implements IOSRMService {

    private final RestClient restClient;
    private final OSRMConfiguration osrmConfig;

    @Autowired
    public OSRMService(RestClient restClient, OSRMConfiguration osrmConfig) {
        this.restClient = restClient;
        this.osrmConfig = osrmConfig;
    }

    private String parseCoordinates(List<Coordinate> coordinates) {
        return coordinates
            .stream()
            .map(Coordinate::toString)
            .collect(Collectors.joining(";"));
    }

    private String generateRouteRequestUri(List<Coordinate> coordinates) {
        return String.format("%s/%s/%s?%s",
            osrmConfig.getBaseUrl(),
            osrmConfig.getRouteEndpoint(),
            parseCoordinates(coordinates),
            osrmConfig.getRouteParams());
    }

    private String generateMatrixRequestUri(List<Coordinate> coordinates) {
        return String.format("%s/%s/%s?%s",
            osrmConfig.getBaseUrl(),
            osrmConfig.getMatrixEndpoint(),
            parseCoordinates(coordinates),
            osrmConfig.getMatrixParams());
    }

    @Override
    public OSRMRouteResult queryRoute(List<Coordinate> coordinates) {
        try {
            var result = restClient.get()
                .uri(generateRouteRequestUri(coordinates))
                .accept(MediaType.APPLICATION_JSON)
                .retrieve()
                .body(OSRMRouteResult.class);

            if (result == null) {
                throw new QueryRouteException("OSRM route returned null");
            }

            if (!Objects.equals(result.getCode(), "Ok")) {
                throw new QueryRouteException("OSRM route returned error response");
            }

            return result;
        }
        catch (RuntimeException ex) {
            throw new QueryRouteException(ex.getLocalizedMessage());
        }
    }

    @Override
    public OSRMMatrixResult queryMatrix(List<Coordinate> coordinates) {
        try {
            var result = restClient.get()
                .uri(generateMatrixRequestUri(coordinates))
                .accept(MediaType.APPLICATION_JSON)
                .retrieve()
                .body(OSRMMatrixResult.class);

            if (result == null) {
                throw new QueryMatrixException("OSRM table returned null");
            }

            if (!Objects.equals(result.getCode(), "Ok")) {
                throw new QueryMatrixException("OSRM table returned error response");
            }

            return result;
        }
        catch (RuntimeException ex) {
            throw new QueryMatrixException(ex.getLocalizedMessage());
        }
    }
}
