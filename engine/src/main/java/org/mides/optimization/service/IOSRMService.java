package org.mides.optimization.service;

import org.mides.optimization.model.Coordinate;
import org.mides.optimization.model.osrm.OSRMMatrixResult;
import org.mides.optimization.model.osrm.OSRMRouteResult;

import java.util.List;

public interface IOSRMService {
    OSRMRouteResult queryRoute(List<Coordinate> coordinates);
    OSRMMatrixResult queryMatrix(List<Coordinate> coordinates);
}
