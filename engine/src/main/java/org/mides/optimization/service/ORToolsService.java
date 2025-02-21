package org.mides.optimization.service;

import com.google.ortools.constraintsolver.*;
import org.mides.optimization.model.*;
import org.mides.optimization.util.Utils;
import org.springframework.stereotype.Service;

import java.time.Duration;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

@Service
public class ORToolsService implements IORToolsService {

    /* Max approx. distance between nodes * SpanCostCoefficient */
    private static final long DROP_PENALTY = 10000000 * 100;
    private static final long MAX_RIDE_TIME = 3600;

    @Override
    public Solution solve(Problem problem, long[][] distanceMatrix, long[][] timeMatrix) {

        Objects.requireNonNull(problem, "Problem cannot be null");
        Objects.requireNonNull(distanceMatrix, "Distance matrix cannot be null");
        Objects.requireNonNull(timeMatrix, "Time matrix cannot be null");


        int numVehicles = problem.getVehicles().size();
        List<Integer> startIndices = new ArrayList<>();
        List<Integer> endIndices = new ArrayList<>();
        System.out.println("Num Vehicles: " + numVehicles);
        for (int i = 0; i < numVehicles; i++) {
            Vehicle vehicle = problem.getVehicles().get(i);
            Objects.requireNonNull(vehicle, "Vehicle cannot be null");
            Objects.requireNonNull(vehicle.getDepotStart(), "Vehicle depotStart cannot be null");
            Objects.requireNonNull(vehicle.getDepotEnd(), "Vehicle depotEnd cannot be null");

            startIndices.add(vehicle.getDepotStart().getIndex());
            endIndices.add(vehicle.getDepotEnd().getIndex());
        }
        System.out.println("Start Indices: " + startIndices);


        var manager = new RoutingIndexManager(
                problem.getNumberOfNodes(),
                numVehicles,
                startIndices.stream().mapToInt(Integer::intValue).toArray(),
                endIndices.stream().mapToInt(Integer::intValue).toArray()
        );
        System.out.println("Manager: " + manager);
        var routing = new RoutingModel(manager);
        System.out.println("Routing: " + routing);
        /* Allow dropping nodes */
        for (int i = 0; i < problem.getNumberOfNodes(); i++) {
            //Skip Start and End Depots
            final int currentNodeIndex = i;
            PickupDeliveryTask task = problem.getTasksByIndex().get(currentNodeIndex);
            if (task != null &&
                    task.getStopId() != null &&
                    problem.getVehicles().stream().anyMatch(v ->
                            v.getDepotStart().getId().equals(task.getStopId()) ||
                                    v.getDepotEnd().getId().equals(task.getStopId())))
            {
                continue;
            }
            routing.addDisjunction(new long[] { manager.nodeToIndex(currentNodeIndex) }, DROP_PENALTY);
        }
        System.out.println("Routing after drop: " + routing);
        /* Add distance dimension */
        var distanceCallbackIndex = routing.registerTransitCallback((fromIndex, toIndex) -> {
            var fromNode = manager.indexToNode(fromIndex);
            var toNode = manager.indexToNode(toIndex);
             if (fromNode < 0 || fromNode >= distanceMatrix.length || toNode < 0 || toNode >= distanceMatrix[fromNode].length) {
                System.err.println("ERROR: Index out of bounds in distance callback. fromNode: " + fromNode + ", toNode: " + toNode);
                return 0; // Or some other appropriate error handling
            }
            return distanceMatrix[fromNode][toNode];
        });
        System.out.println("Distance Callback Index: " + distanceCallbackIndex);
        routing.addDimension(
                distanceCallbackIndex,
                0,
                Long.MAX_VALUE,
                true,
                "distance"
        );
        routing.setArcCostEvaluatorOfAllVehicles(distanceCallbackIndex);
        var distanceDimension = routing.getMutableDimension("distance");
        distanceDimension.setGlobalSpanCostCoefficient(100);
        System.out.println("Distance Dimension: " + distanceDimension);
        /* Add time dimension */
        var timeCallbackIndex = routing.registerTransitCallback((fromIndex, toIndex) ->
        {
            var fromNode = manager.indexToNode(fromIndex);
            var toNode = manager.indexToNode(toIndex);
            if (fromNode < 0 || fromNode >= timeMatrix.length || toNode < 0 || toNode >= timeMatrix[fromNode].length) {
                System.err.println("ERROR: Index out of bounds in time callback. fromNode: " + fromNode + ", toNode: " + toNode + ", Size: " +timeMatrix.length);
                return 0;
            }
            return timeMatrix[fromNode][toNode];
        });
        System.out.println("Time Callback Index: " + timeCallbackIndex);
        routing.addDimension(
                timeCallbackIndex,
                Long.MAX_VALUE, /* Allow waiting time */
                Long.MAX_VALUE, /* Max duration per vehicle */
                false, /* Don't force to start cumulative var at zero*/
                "time"
        );
        var timeDimension = routing.getMutableDimension("time");
        System.out.println("Time Dimension: " + timeDimension);
        // Set time windows for all nodes, including start and end depots
        for (int i = 0; i < problem.getNumberOfNodes(); i++) {
            PickupDeliveryTask task = problem.getTasksByIndex().get(i);
            System.out.println("task: " + task + " i: " + i);
            if (task != null) {
                //var index = manager.nodeToIndex(i);
                if (task.getTimeWindow().startSeconds() > task.getTimeWindow().endSeconds()) {
                     System.err.println("ERROR: Invalid time window for task index " + i +
                        ". Start: " + task.getTimeWindow().startSeconds() +
                        ", End: " + task.getTimeWindow().endSeconds());
                }
                System.out.println("Setting time window for index: " + i);
                try {
                    timeDimension.cumulVar(i).setRange(
                        task.getTimeWindow().startSeconds(),
                        task.getTimeWindow().endSeconds()
                    );
                 } catch (Exception e) {
                        System.out.println("ERROR setting time window for index: " + i);
                        System.err.println("ERROR setting time window for i: " + i +
                            ", Node Index: " + i +
                            ", Start: " + task.getTimeWindow().startSeconds() +
                            ", End: " + task.getTimeWindow().endSeconds() +
                            ", Exception: " + e.getMessage());
                        throw e;
                }
                System.out.println("Setting slack variable for index: " + i);
                var solverIndex = manager.nodeToIndex(i);  // Get the solver's internal index
                if (task.getRide() != null && task.getRide().getPickup().getIndex() == i) {
                    System.out.println("Setting slack variable for index: " + i);
                    timeDimension.slackVar(solverIndex).setValue(0); // Use the solver's index
                }
                System.out.println("Added to assignment for i: " + i);
                routing.addToAssignment(timeDimension.slackVar(solverIndex));
            }
            else{
                 System.err.println("WARNING: Task at index " + i + " is null.");
            }
        }

        System.out.println("Routing after time: " + routing);
        for (int vehicle = 0; vehicle < numVehicles; vehicle++)
        {
            int startIndex = (int) manager.nodeToIndex(startIndices.get(vehicle));
            int endIndex = (int) manager.nodeToIndex(endIndices.get(vehicle));

             if (problem.getVehicles().get(vehicle).getDepotStart().getTimeWindow().startSeconds() >  problem.getVehicles().get(vehicle).getDepotStart().getTimeWindow().endSeconds()) {
                    System.err.println("ERROR: Invalid time window for Depot Start " + vehicle );
             }
             if (problem.getVehicles().get(vehicle).getDepotEnd().getTimeWindow().startSeconds() >  problem.getVehicles().get(vehicle).getDepotEnd().getTimeWindow().endSeconds()) {
                    System.err.println("ERROR: Invalid time window for Depot End " + vehicle );
             }
            try{
            timeDimension.cumulVar(startIndex).setRange(
                problem.getVehicles().get(vehicle).getDepotStart().getTimeWindow().startSeconds(),
                problem.getVehicles().get(vehicle).getDepotStart().getTimeWindow().endSeconds()
            );
            } catch (Exception e) {
                System.err.println("ERROR setting time window for index: " + vehicle +
                    ", Node Index: " + startIndex +
                    ", Start: " + problem.getVehicles().get(vehicle).getDepotStart().getTimeWindow().startSeconds() +
                    ", End: " + problem.getVehicles().get(vehicle).getDepotStart().getTimeWindow().endSeconds() +
                    ", Exception: " + e.getMessage());
                throw e; // Re-throwing is often a good choice in this situation.
            }

            routing.addVariableMaximizedByFinalizer(
                    timeDimension.cumulVar(startIndex)
            );
            routing.addVariableMinimizedByFinalizer(
                    timeDimension.cumulVar(endIndex)
            );
        }
        System.out.println("Routing after depot: " + routing);
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
        System.out.println("Routing after pickup-delivery: " + routing);
        /* Add maximum ride time for a single pickup-delivery constraint */
        for (var ride : problem.getRideRequests()) {
            var pickupIndex = manager.nodeToIndex(ride.getPickup().getIndex());
            var deliveryIndex = manager.nodeToIndex(ride.getDelivery().getIndex());
            solver.addConstraint(solver.makeLessOrEqual(
                    timeDimension.cumulVar(deliveryIndex),
                    solver.makeSum(timeDimension.cumulVar(pickupIndex), MAX_RIDE_TIME)
            ));
        }
        System.out.println("Routing after max ride time: " + routing);
        var searchParams = main.defaultRoutingSearchParameters()
                .toBuilder()
                .setFirstSolutionStrategy(FirstSolutionStrategy.Value.PATH_CHEAPEST_ARC)
                .setLocalSearchMetaheuristic(LocalSearchMetaheuristic.Value.GUIDED_LOCAL_SEARCH)
                .setTimeLimit(com.google.protobuf.Duration.newBuilder().setNanos(500000000).build())
                .build();
        System.out.println("Search params: " + searchParams);
        var assignment = routing.solveWithParameters(searchParams);
        System.out.println("Assignment: " + assignment);
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
            var problemVehicle = problem.getVehicles().get(vehicle);
            route.setVehicleId(problemVehicle.getId());
            // Remove setting the route's time window from the vehicle:
            // route.setTimeWindow(problemVehicle.getTimeWindow());
            int nodeIndex;
            int position = 0;
            long index = routing.start(vehicle);


            while (!routing.isEnd(index)) {
                nodeIndex = manager.indexToNode(index);
                var taskNodeIndex = problem.getTasksByIndex().get(nodeIndex);
                int nextNodeIndex = manager.indexToNode(assignment.value(routing.nextVar(index)));

                var ride = taskNodeIndex.getRide();
                Visit visit = new Visit();
                visit.setPosition(position++);
                visit.setRideId(ride != null ? ride.getId() : null);
                visit.setUserId(ride != null ? ride.getUserId() : null);
                visit.setRideDirection(ride != null ? ride.getDirection() : null);
                visit.setAddress(taskNodeIndex.getAddress());
                visit.setCoordinates(taskNodeIndex.getCoordinates());
                visit.setArrivalTime(Duration.ofSeconds(assignment.min(timeDimension.cumulVar(index))));
                visit.setTravelTimeToNextVisit(Duration.ofSeconds(timeMatrix[nodeIndex][nextNodeIndex]));
                visit.setSolutionWindow(new TimeWindow(
                        Duration.ofSeconds(assignment.min(timeDimension.cumulVar(index))),
                        Duration.ofSeconds(assignment.max(timeDimension.cumulVar(index)))
                ));
                visit.setType(taskNodeIndex.getType());
                visit.setStopId(taskNodeIndex.getStopId());

                route.getVisits().add(visit);
                index = assignment.value(routing.nextVar(index));
            }

            nodeIndex = manager.indexToNode(index);
            var taskNodeIndex = problem.getTasksByIndex().get(nodeIndex);
            var ride = taskNodeIndex.getRide();

            Visit lastVisit = new Visit();
            lastVisit.setPosition(position);
            lastVisit.setRideId(ride != null ? ride.getId() : null);
            lastVisit.setUserId(ride != null ? ride.getUserId() : null);
            lastVisit.setRideDirection(ride != null ? ride.getDirection() : null);
            lastVisit.setAddress(taskNodeIndex.getAddress());
            lastVisit.setCoordinates(taskNodeIndex.getCoordinates());
            lastVisit.setArrivalTime(Duration.ofSeconds(assignment.min(timeDimension.cumulVar(index))));
            lastVisit.setTravelTimeToNextVisit(Duration.ZERO);
            lastVisit.setSolutionWindow(new TimeWindow(
                    Duration.ofSeconds(assignment.min(timeDimension.cumulVar(index))),
                    Duration.ofSeconds(assignment.max(timeDimension.cumulVar(index)))
            ));
            lastVisit.setType(taskNodeIndex.getType());
            lastVisit.setStopId(taskNodeIndex.getStopId());

            route.getVisits().add(lastVisit);

            var endTime = route.getVisits().get(route.getVisits().size() - 1).getArrivalTime();
            var startTime = route.getVisits().get(0).getArrivalTime();
            route.setDuration(endTime.minus(startTime));
            route.setDistance(Utils.convertDistanceBack(assignment.value(distanceDimension.cumulVar(index))));
            if (route.getVisits().size() > 2) solution.getRoutes().add(route);
        }

        return solution;
    }
}