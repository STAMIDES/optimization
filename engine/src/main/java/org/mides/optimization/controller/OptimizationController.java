package org.mides.optimization.controller;

import jakarta.validation.Valid;
import org.mides.optimization.exception.osrm.QueryMatrixException;
import org.mides.optimization.model.Problem;
import org.mides.optimization.model.Route;
import org.mides.optimization.model.Solution;
import org.mides.optimization.model.Visit;
import org.mides.optimization.service.IORToolsService;
import org.mides.optimization.service.IOSRMService;
import org.mides.optimization.util.ArrayUtils;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.http.ResponseEntity;
import org.springframework.validation.annotation.Validated;
import org.springframework.web.bind.annotation.*;

import java.util.List;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ExecutorService;

@Validated
@RestController
@CrossOrigin(origins = "*")
@RequestMapping("optimization/v1")
public class OptimizationController {

    private final IOSRMService osrmService;
    private final IORToolsService orToolsService;
    private final ExecutorService executorService;

    @Autowired
    public OptimizationController(
        IOSRMService osrmService,
        IORToolsService orToolsService,
        ExecutorService executorService)
    {
        this.osrmService = osrmService;
        this.orToolsService = orToolsService;
        this.executorService = executorService;
    }

    @PostMapping("/solve")
    public CompletableFuture<ResponseEntity<Solution>> solve(@RequestBody @Valid Problem problem) {
        problem.initialize();

        var matrixFuture = CompletableFuture.supplyAsync(() -> {
            try {
                return osrmService.queryMatrix(problem.getAllCoordinates());
            } catch (Exception e) {
                throw new QueryMatrixException("Failed to query matrix");
            }
        }, executorService);

        return matrixFuture.thenCompose(matrix -> {
            long[][] distanceMatrix = ArrayUtils.convertTo2DLongArray(matrix.getDistances(), 100);
            long[][] timeMatrix = ArrayUtils.convertTo2DLongArray(matrix.getDurations());

            return CompletableFuture.supplyAsync(() -> {
                try {
                    return orToolsService.solve(problem, distanceMatrix, timeMatrix);
                } catch (Exception e) {
                    throw new RuntimeException("Constraint solver failed", e);
                }
            }, executorService).thenApply(solution -> {
                var routeFutures = solution.getRoutes().stream()
                    .map(route -> CompletableFuture.supplyAsync(() -> {
                        try {
                            route.setGeometry(generateRouteGeometry(route));
                            return route;
                        } catch (Exception e) {
                            throw new RuntimeException("Failed to query route geometry", e);
                        }
                    }, executorService))
                    .toList();

                CompletableFuture.allOf(routeFutures.toArray(new CompletableFuture[0])).join();
                solution.setRoutes(routeFutures.stream()
                    .map(CompletableFuture::join)
                    .toList()
                );

                return ResponseEntity.ok(solution);
            });
        });
    }

    private List<List<Double>> generateRouteGeometry(Route route) {
        var visitsCoordinates = route.getVisits().stream().map(Visit::getCoordinates).toList();
        var osrmRouteResult = osrmService.queryRoute(visitsCoordinates);

        if (osrmRouteResult.getRoutes().size() != 1)
            throw new IllegalArgumentException(
                String.format(
                    "Exactly one route expected from OSRM but received %d",
                    osrmRouteResult.getRoutes().size()
                ));

        return osrmRouteResult.getRoutes().get(0).decodeGeometry();
    }
}
