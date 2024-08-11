package org.mides.optimization.service;

import com.google.ortools.constraintsolver.RoutingIndexManager;
import com.google.ortools.constraintsolver.RoutingModel;
import com.google.ortools.constraintsolver.Assignment;
import com.google.ortools.constraintsolver.FirstSolutionStrategy;
import com.google.ortools.constraintsolver.LocalSearchMetaheuristic;
import com.google.ortools.constraintsolver.main;
import org.mides.optimization.model.*;
import org.mides.optimization.util.Utils;
import org.springframework.stereotype.Service;

import java.time.Duration;

@Service
public class ORToolsService implements IORToolsService {

    /* Max approx. distance between nodes * SpanCostCoefficient */
    private static final long DROP_PENALTY = 1000000 * 100;

    @Override
    public Solution solve(Problem problem, long[][] distanceMatrix, long[][] timeMatrix) {
        var manager = new RoutingIndexManager(
            problem.getNumberOfNodes(),
            problem.getVehicles().size(),
            0
        );
        var routing = new RoutingModel(manager);

        /* Allow dropping nodes */
        for (int i = 1; i < problem.getNumberOfNodes(); i++) {
            routing.addDisjunction(new long[] { manager.nodeToIndex(i) }, DROP_PENALTY);
        }

        /* Add distance dimension */
        var distanceCallbackIndex = routing.registerTransitCallback((fromIndex, toIndex) -> {
            var fromNode = manager.indexToNode(fromIndex);
            var toNode = manager.indexToNode(toIndex);
            return distanceMatrix[fromNode][toNode];
        });
        routing.setArcCostEvaluatorOfAllVehicles(distanceCallbackIndex);
        routing.addDimension(
            distanceCallbackIndex,
            0,
            Long.MAX_VALUE,
            true,
            "distance"
        );
        var distanceDimension = routing.getMutableDimension("distance");
        distanceDimension.setGlobalSpanCostCoefficient(100);

        /* Add time dimension */
        var timeCallbackIndex = routing.registerTransitCallback((fromIndex, toIndex) ->
        {
            var fromNode = manager.indexToNode(fromIndex);
            var toNode = manager.indexToNode(toIndex);
            return timeMatrix[fromNode][toNode];
        });
        routing.addDimension(
            timeCallbackIndex,
            Long.MAX_VALUE, /* Allow waiting time */
            Long.MAX_VALUE, /* Max duration per vehicle */
            false, /* Don't force to start cumulative var at zero*/
            "time"
        );
        var timeDimension = routing.getMutableDimension("time");
        for (var i = 1; i < problem.getNumberOfNodes(); i++)
        {
            var index = manager.nodeToIndex(i);
            timeDimension.cumulVar(index).setRange(
                problem.getTasksByIndex().get(i).getTimeWindow().startSeconds(),
                problem.getTasksByIndex().get(i).getTimeWindow().endSeconds()
            );
            if (problem.getTasksByIndex().get(i).getRide().getPickup().getIndex() == i)
                timeDimension.slackVar(index).setValue(0); /* No waiting time after pickup */
            routing.addToAssignment(timeDimension.slackVar(index));
        }

        for (var vehicle = 0; vehicle < problem.getVehicles().size(); vehicle++)
        {
            var index = routing.start(vehicle);
            timeDimension.cumulVar(index).setRange(
                problem.getDepot().getTimeWindow().startSeconds(),
                problem.getDepot().getTimeWindow().endSeconds()
            );
        }

        for (var vehicle = 0; vehicle < problem.getVehicles().size(); vehicle++)
        {
            routing.addVariableMinimizedByFinalizer(
                timeDimension.cumulVar(routing.start(vehicle))
            );
            routing.addVariableMinimizedByFinalizer(
                timeDimension.cumulVar(routing.end(vehicle))
            );
        }

        /* TO DO: Add capacity constraints */

        /* Add pickup-delivery constraints */
        var solver = routing.solver();
        for (var ride : problem.getRideRequests()) {
            var pickupIndex = manager.nodeToIndex(ride.getPickup().getIndex());
            var deliveryIndex = manager.nodeToIndex(ride.getDelivery().getIndex());
            routing.addPickupAndDelivery(pickupIndex, deliveryIndex);
            solver.addConstraint(solver.makeEquality(
                routing.vehicleVar(pickupIndex),
                routing.vehicleVar(deliveryIndex)
            ));
            solver.addConstraint(solver.makeLessOrEqual(
                distanceDimension.cumulVar(pickupIndex),
                distanceDimension.cumulVar(deliveryIndex)
            ));
        }

        var searchParams = main.defaultRoutingSearchParameters()
            .toBuilder()
            .setFirstSolutionStrategy(FirstSolutionStrategy.Value.PATH_CHEAPEST_ARC)
            .setLocalSearchMetaheuristic(LocalSearchMetaheuristic.Value.GUIDED_LOCAL_SEARCH)
            .setTimeLimit(com.google.protobuf.Duration.newBuilder().setNanos(500000000).build())
            .build();

        var assignment = routing.solveWithParameters(searchParams);

        return buildSolution(problem, routing, manager, assignment, timeMatrix);
    }

    private static Solution buildSolution(
        Problem problem,
        RoutingModel routing,
        RoutingIndexManager manager,
        Assignment assignment,
        long[][] timeMatrix
    ) {
        if (assignment == null) {
            throw new IllegalArgumentException("Unfeasible problem");
        }

        Solution solution = new Solution();

        /* Dropped rides */
        for (int node = 0; node < routing.size(); node++) {
            if (routing.isStart(node) || routing.isEnd(node)) {
                continue;
            }
            if (assignment.value(routing.nextVar(node)) != node) {
                continue;
            }

            int droppedNode = manager.indexToNode(node);
            var ride = problem.getTasksByIndex().get(droppedNode).getRide();
            var rideId = ride != null ? ride.getId() : null;

            if (rideId != null && !solution.getDroppedRides().contains(rideId)) {
                solution.getDroppedRides().add(rideId);
            }
        }

        /* Routes */
        var timeDimension = routing.getMutableDimension("time");
        var distanceDimension = routing.getMutableDimension("distance");

        for (int vehicle = 0; vehicle < problem.getVehicles().size(); vehicle++) {
            var route = new Route();
            route.setVehicleId(problem.getVehicles().get(vehicle).getId());

            int nodeIndex;
            int previousNodeIndex = -1;
            int position = 0;
            long index = routing.start(vehicle);

            while (!routing.isEnd(index)) {
                nodeIndex = manager.indexToNode(index);
                Duration travelTime = previousNodeIndex == -1
                    ? Duration.ZERO
                    : Duration.ofSeconds(timeMatrix[previousNodeIndex][nodeIndex]);

                var ride = problem.getTasksByIndex().get(nodeIndex).getRide();
                Visit visit = new Visit();
                visit.setPosition(position++);
                visit.setRideId(ride != null ? ride.getId() : null);
                visit.setUserId(ride != null ? ride.getUserId() : null);
                visit.setRideDirection(ride != null ? ride.getDirection() : null);
                visit.setAddress(problem.getTasksByIndex().get(nodeIndex).getAddress());
                visit.setCoordinates(problem.getTasksByIndex().get(nodeIndex).getCoordinates());
                visit.setArrivalTime(Duration.ofSeconds(assignment.min(timeDimension.cumulVar(index))));
                visit.setWaitingTime(travelTime);
                visit.setSolutionWindow(new TimeWindow(
                    Duration.ofSeconds(assignment.min(timeDimension.cumulVar(index))),
                    Duration.ofSeconds(assignment.max(timeDimension.cumulVar(index)))
                ));

                route.getVisits().add(visit);
                index = assignment.value(routing.nextVar(index));

                previousNodeIndex = nodeIndex;
            }

            nodeIndex = manager.indexToNode(index);

            var ride = problem.getTasksByIndex().get(nodeIndex).getRide();

            Visit lastVisit = new Visit();
            lastVisit.setPosition(position);
            lastVisit.setRideId(ride != null ? ride.getId() : null);
            lastVisit.setUserId(ride != null ? ride.getUserId() : null);
            lastVisit.setRideDirection(ride != null ? ride.getDirection() : null);
            lastVisit.setAddress(problem.getTasksByIndex().get(nodeIndex).getAddress());
            lastVisit.setCoordinates(problem.getTasksByIndex().get(nodeIndex).getCoordinates());
            lastVisit.setArrivalTime(Duration.ofSeconds(assignment.min(timeDimension.cumulVar(index))));
            lastVisit.setWaitingTime(Duration.ofSeconds(timeMatrix[previousNodeIndex][nodeIndex]));
            lastVisit.setSolutionWindow(new TimeWindow(
                Duration.ofSeconds(assignment.min(timeDimension.cumulVar(index))),
                Duration.ofSeconds(assignment.max(timeDimension.cumulVar(index)))
            ));

            route.getVisits().add(lastVisit);

            var endTime = route.getVisits().get(route.getVisits().size() - 1).getArrivalTime();
            var startTime = route.getVisits().get(0).getArrivalTime();
            route.setDuration(endTime.minus(startTime));
            route.setDistance(Utils.convertDistanceBack(assignment.value(distanceDimension.cumulVar(index))));

            solution.getRoutes().add(route);
        }

        return solution;
    }
}
