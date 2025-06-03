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

import java.util.ArrayList;
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

    @Override
    public OSRMMatrixResult queryMatrixInBatches(List<Coordinate> coordinates, int batchSize) {
        if (coordinates.size() <= batchSize) {
            return queryMatrix(coordinates);
        }

        int size = coordinates.size();
        List<List<Double>> fullDistanceMatrix = new ArrayList<>(size);
        List<List<Double>> fullDurationMatrix = new ArrayList<>(size);
        
        // Initialize the full matrices
        for (int i = 0; i < size; i++) {
            List<Double> distanceRow = new ArrayList<>(size);
            List<Double> durationRow = new ArrayList<>(size);
            for (int j = 0; j < size; j++) {
                distanceRow.add(0.0);
                durationRow.add(0.0);
            }
            fullDistanceMatrix.add(distanceRow);
            fullDurationMatrix.add(durationRow);
        }

        // Process the matrix in batches
        for (int sourceStart = 0; sourceStart < size; sourceStart += batchSize) {
            int sourceEnd = Math.min(sourceStart + batchSize, size);
            
            for (int destStart = 0; destStart < size; destStart += batchSize) {
                int destEnd = Math.min(destStart + batchSize, size);
                
                // Prepare sub-list of coordinates for this batch
                List<Coordinate> batchCoords = new ArrayList<>();
                
                // Add source coordinates
                for (int i = sourceStart; i < sourceEnd; i++) {
                    batchCoords.add(coordinates.get(i));
                }
                
                // Add destination coordinates if they're not already included
                for (int i = destStart; i < destEnd; i++) {
                    if (i < sourceStart || i >= sourceEnd) {
                        batchCoords.add(coordinates.get(i));
                    }
                }

                // Calculate sources and destinations indices for table request
                String sources = "";
                String destinations = "";
                
                for (int i = 0; i < sourceEnd - sourceStart; i++) {
                    if (!sources.isEmpty()) sources += ";";
                    sources += i;
                }

                int destIndex = sourceEnd - sourceStart;
                for (int i = destStart; i < destEnd; i++) {
                    if (i < sourceStart || i >= sourceEnd) {
                        if (!destinations.isEmpty()) destinations += ";";
                        destinations += destIndex;
                        destIndex++;
                    }
                }

                try {
                    // Use custom URI for batch request with sources and destinations
                    String uri = String.format("%s/%s/%s?%s&sources=%s&destinations=%s",
                        osrmConfig.getBaseUrl(),
                        osrmConfig.getMatrixEndpoint(),
                        parseCoordinates(batchCoords),
                        osrmConfig.getMatrixParams(),
                        sources,
                        destinations.isEmpty() ? sources : destinations);

                    var result = restClient.get()
                        .uri(uri)
                        .accept(MediaType.APPLICATION_JSON)
                        .retrieve()
                        .body(OSRMMatrixResult.class);

                    if (result == null) {
                        throw new QueryMatrixException("OSRM table returned null");
                    }

                    if (!Objects.equals(result.getCode(), "Ok")) {
                        throw new QueryMatrixException("OSRM table returned error response");
                    }

                    // Map batch results back to the full matrix
                    int sourceIdx = 0;
                    for (int i = sourceStart; i < sourceEnd; i++) {
                        int destIdx = 0;
                        for (int j = destStart; j < destEnd; j++) {
                            if (i >= sourceStart && i < sourceEnd && j >= destStart && j < destEnd) {
                                // Determine the correct index in the batch result
                                int batchDestIdx = (j < sourceStart || j >= sourceEnd) ? 
                                    destIdx : j - sourceStart;
                                
                                fullDistanceMatrix.get(i).set(j, result.getDistances().get(sourceIdx).get(batchDestIdx));
                                fullDurationMatrix.get(i).set(j, result.getDurations().get(sourceIdx).get(batchDestIdx));
                                
                                if (j < sourceStart || j >= sourceEnd) {
                                    destIdx++;
                                }
                            }
                        }
                        sourceIdx++;
                    }
                } catch (RuntimeException ex) {
                    throw new QueryMatrixException("Error processing batch matrix: " + ex.getLocalizedMessage());
                }
            }
        }

        // Create final result
        OSRMMatrixResult finalResult = new OSRMMatrixResult();
        finalResult.setCode("Ok");
        finalResult.setDistances(fullDistanceMatrix);
        finalResult.setDurations(fullDurationMatrix);
        return finalResult;
    }
}
