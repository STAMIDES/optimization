package org.mides.optimization.service;

import com.google.ortools.constraintsolver.*;
import org.mides.optimization.model.Coordinate;
import org.mides.optimization.model.Depot;
import org.mides.optimization.model.DepotDroppedRideInfo;
import org.mides.optimization.model.Problem;
import org.mides.optimization.model.RideRequest;
import org.mides.optimization.model.Route;
import org.mides.optimization.model.Solution;
import org.mides.optimization.model.Task;
import org.mides.optimization.model.TaskType;
import org.mides.optimization.model.TimeWindow;
import org.mides.optimization.model.Vehicle;
import org.mides.optimization.model.Visit;
import org.mides.optimization.util.Constants.ALLOW_DEPOT_DROP;
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
    private static final long DEPOT_DROP_PENALTY = DROP_PENALTY / 2; // Prefer depot drop over full drop
    private static final long MAX_RIDE_TIME = 5000;

    @Override
    public Solution solve(Problem problem, long[][] distanceMatrix, long[][] timeMatrix) {
        var starts = new int[problem.getVehicles().size()];
        var ends = new int[problem.getVehicles().size()];

        for (int i = 0; i < problem.getVehicles().size(); i++) {
            var vehicle = problem.getVehicles().get(i);
            starts[i] = vehicle.getDepotStart().getIndex();
            ends[i] = vehicle.getDepotEnd().getIndex();
        }

        var manager = new RoutingIndexManager(
            problem.getNumberOfNodes(),
            problem.getVehicles().size(),
            starts,
            ends
        );
        var routing = new RoutingModel(manager);
        var solver = routing.solver(); // Moved solver initialization up

        /* Allow dropping nodes */
        if (ALLOW_DEPOT_DROP) {
            for (var ride : problem.getRideRequests()) {
                var pickupNodeIndex = manager.nodeToIndex(ride.getPickup().getIndex());
                var deliveryNodeIndex = manager.nodeToIndex(ride.getDelivery().getIndex());

                // Allow dropping the pickup (which means the whole ride is dropped)
                routing.addDisjunction(new long[]{pickupNodeIndex}, DROP_PENALTY);

                // Allow dropping just the delivery at a depot (implies pickup happened)
                // The cost DEPOT_DROP_PENALTY should be less than DROP_PENALTY
                routing.addDisjunction(new long[]{deliveryNodeIndex}, DEPOT_DROP_PENALTY);
            }
        } else {
            // Original logic for dropping nodes if ALLOW_DEPOT_DROP is false
            int taskIndex = problem.getVehicles().size() * 2;
            while (taskIndex < problem.getNumberOfNodes()) {
                var numberOfStops = problem.getNumberOfStopsInRide(taskIndex);
                var indices = new long[numberOfStops];
                for (int i = 0; i < numberOfStops; i++) {
                    indices[i] = manager.nodeToIndex(taskIndex + i);
                }
                routing.addDisjunction(indices, DROP_PENALTY, numberOfStops);
                taskIndex += numberOfStops;
            }
        }

        /* Add distance dimension */
        var distanceCallbackIndex = routing.registerTransitCallback((fromIndex, toIndex) -> {
            var fromNode = manager.indexToNode(fromIndex);
            var toNode = manager.indexToNode(toIndex);
            return distanceMatrix[fromNode][toNode];
        });
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
        for (var i = problem.getVehicles().size() * 2; i < problem.getNumberOfNodes(); i++)
        {
            var index = manager.nodeToIndex(i);
            timeDimension.cumulVar(index).setRange(
                problem.getTasksByIndex().get(i).getTimeWindow().startSeconds(),
                problem.getTasksByIndex().get(i).getTimeWindow().endSeconds()
            );
            routing.addToAssignment(timeDimension.slackVar(index));
        }

        int vehicleIndex = 0;
        for (var vehicle : problem.getVehicles()) {
            var index = routing.start(vehicleIndex);
            timeDimension.cumulVar(index).setRange(
                vehicle.getTimeWindow().startSeconds(),
                vehicle.getTimeWindow().endSeconds()
            );
            vehicleIndex++;
        }

        for (var vehicle = 0; vehicle < problem.getVehicles().size(); vehicle++)
        {
            routing.addVariableMaximizedByFinalizer(
                timeDimension.cumulVar(routing.start(vehicle))
            );
            routing.addVariableMinimizedByFinalizer(
                timeDimension.cumulVar(routing.end(vehicle))
            );
        }

        /* Add seat capacity constraints */
        var seatDemands = problem.getSeatDemands();
        final int seatDemandCallbackIndex = routing.registerUnaryTransitCallback((long fromIndex) -> {
            int fromNode = manager.indexToNode(fromIndex);
            return seatDemands[fromNode];
        });

        var seatCapacities = problem.getSeatCapacities();

        routing.addDimensionWithVehicleCapacity(
            seatDemandCallbackIndex,
            0,
            seatCapacities,
            true,
            "seat"
        );

        /* Add wheelchair capacity constraints */
        var wheelchairDemands = problem.getWheelchairDemands();
        final int wheelchairDemandCallbackIndex = routing.registerUnaryTransitCallback((long fromIndex) -> {
            int fromNode = manager.indexToNode(fromIndex);
            return wheelchairDemands[fromNode];
        });

        var wheelchairCapacities = problem.getWheelchairCapacities();

        routing.addDimensionWithVehicleCapacity(
            wheelchairDemandCallbackIndex,
            0,
            wheelchairCapacities,
            true,
            "wheelchair"
        );

        /* Add pickup-delivery constraints */
        for (var ride : problem.getRideRequests()) {
            var pickupIndex = manager.nodeToIndex(ride.getPickup().getIndex());
            var deliveryIndex = manager.nodeToIndex(ride.getDelivery().getIndex());
            routing.addPickupAndDelivery(pickupIndex, deliveryIndex);

            if (ALLOW_DEPOT_DROP) {
                IntVar deliveryActiveVar = routing.activeVar(deliveryIndex);
                // Constraint: deliveryActiveVar == 0 OR vehicleVar(P) == vehicleVar(D)
                // This is equivalent to: (deliveryActiveVar == 0) + (vehicleVar(P) == vehicleVar(D)) >= 1
                // Let deliveryDroppedVar = (deliveryActiveVar == 0)
                IntVar deliveryDroppedVar = solver.makeIsEqualCstVar(deliveryActiveVar, 0);
                // Let sameVehicleVar = (vehicleVar(P) == vehicleVar(D))
                IntVar sameVehicleVar = solver.makeIsEqualVar(routing.vehicleVar(pickupIndex), routing.vehicleVar(deliveryIndex));

                solver.addConstraint(
                    solver.makeSumGreaterOrEqual(new IntVar[]{deliveryDroppedVar, sameVehicleVar}, 1)
                );
            } else {
                solver.addConstraint(solver.makeEquality(
                    routing.vehicleVar(pickupIndex),
                    routing.vehicleVar(deliveryIndex)
                ));
            }
            solver.addConstraint(solver.makeLessOrEqual(
                distanceDimension.cumulVar(pickupIndex),
                distanceDimension.cumulVar(deliveryIndex)
            ));
        }

        /* Add maximum ride time for a single pickup-delivery constraint */
        for (var ride : problem.getRideRequests()) {
            var pickupIndex = manager.nodeToIndex(ride.getPickup().getIndex());
            var deliveryIndex = manager.nodeToIndex(ride.getDelivery().getIndex());

            Constraint maxRideTimeConstraint = solver.makeLessOrEqual(
                timeDimension.cumulVar(deliveryIndex),
                solver.makeSum(timeDimension.cumulVar(pickupIndex), MAX_RIDE_TIME)
            );

            if (ALLOW_DEPOT_DROP) {
                IntVar deliveryActiveVar = routing.activeVar(deliveryIndex);
                // Enforce constraint only when deliveryActiveVar is 1 (delivery is performed).
                // If deliveryActiveVar is 0 (delivery is dropped/depot-dropped), the constraint is ignored.
                solver.addConstraint(maxRideTimeConstraint).when(deliveryActiveVar.isEqualTo(1));
            } else {
                solver.addConstraint(maxRideTimeConstraint);
            }
        }

        /* Add constraint for rides to be contained on vehicles depot start and end */
        vehicleIndex = 0;
        for (var vehicle : problem.getVehicles()) {
            long vehicleStartTime = vehicle.getTimeWindow().startSeconds();
            long vehicleEndTime = vehicle.getTimeWindow().endSeconds();

            for (var ride : problem.getRideRequests()) {
                // Pickup constraints
                long pickupIndex = manager.nodeToIndex(ride.getPickup().getIndex());
                IntVar vehicleVar = routing.vehicleVar(pickupIndex);
                IntVar pickupTime = timeDimension.cumulVar(pickupIndex);

                // Create a boolean variable: isVehicleUsedForPickup = (vehicleVar == vehicleIndex)
                IntVar isVehicleUsedForPickup = solver.makeIsEqualCstVar(vehicleVar, vehicleIndex);

                // Create boolean variables for the time window constraints
                IntVar isTimeGreaterThanStart = solver.makeIsGreaterOrEqualCstVar(pickupTime, vehicleStartTime);
                IntVar isTimeLessThanEnd = solver.makeIsLessOrEqualCstVar(pickupTime, vehicleEndTime);

                // Enforce the implications A => B  <=> A <= B
                solver.addConstraint(solver.makeLessOrEqual(isVehicleUsedForPickup, isTimeGreaterThanStart));
                solver.addConstraint(solver.makeLessOrEqual(isVehicleUsedForPickup, isTimeLessThanEnd));

                // Delivery constraints (same logic, using the *same* isVehicleUsedForPickup)
                long deliveryIndex = manager.nodeToIndex(ride.getDelivery().getIndex());
                IntVar deliveryTime = timeDimension.cumulVar(deliveryIndex);

                IntVar isDeliveryTimeGreaterThanStart = solver.makeIsGreaterOrEqualCstVar(deliveryTime, vehicleStartTime);
                IntVar isDeliveryTimeLessThanEnd = solver.makeIsLessOrEqualCstVar(deliveryTime, vehicleEndTime);

                Constraint deliveryFitsAfterVehicleStartConstraint = solver.makeLessOrEqual(isVehicleUsedForPickup, isDeliveryTimeGreaterThanStart);
                Constraint deliveryFitsBeforeVehicleEndConstraint = solver.makeLessOrEqual(isVehicleUsedForPickup, isDeliveryTimeLessThanEnd);

                if (ALLOW_DEPOT_DROP) {
                    IntVar deliveryActiveVar = routing.activeVar(deliveryIndex);
                    solver.addConstraint(deliveryFitsAfterVehicleStartConstraint).when(deliveryActiveVar.isEqualTo(1));
                    solver.addConstraint(deliveryFitsBeforeVehicleEndConstraint).when(deliveryActiveVar.isEqualTo(1));
                } else {
                    solver.addConstraint(deliveryFitsAfterVehicleStartConstraint);
                    solver.addConstraint(deliveryFitsBeforeVehicleEndConstraint);
                }
            }
            vehicleIndex++;
        }

        /* Compatibility */
        for (var ride : problem.getRideRequests()) {
            var pickupNodeIndex = manager.nodeToIndex(ride.getPickup().getIndex());
            var deliveryNodeIndex = manager.nodeToIndex(ride.getDelivery().getIndex());

            for (int vehIndex = 0; vehIndex < problem.getVehicles().size(); vehIndex++) {
                var vehicle = problem.getVehicles().get(vehIndex);

                if (!problem.isRideCompatibleWithVehicle(ride, vehicle)) {
                    // Create boolean variables that equals 1 if this vehicle is used
                    IntVar isVehicleUsedPickup = solver.makeIsEqualCstVar(
                        routing.vehicleVar(pickupNodeIndex),
                        vehIndex
                    );
                    IntVar isVehicleUsedDelivery = solver.makeIsEqualCstVar(
                        routing.vehicleVar(deliveryNodeIndex),
                        vehIndex
                    );

                    // Force these variables to be 0 (meaning this vehicle cannot be used for this ride)
                    solver.addConstraint(solver.makeEquality(isVehicleUsedPickup, 0));
                    solver.addConstraint(solver.makeEquality(isVehicleUsedDelivery, 0));
                }
            }
        }

        var searchParams = main.defaultRoutingSearchParameters()
            .toBuilder()
            .setFirstSolutionStrategy(FirstSolutionStrategy.Value.PATH_CHEAPEST_ARC)
            .setLocalSearchMetaheuristic(LocalSearchMetaheuristic.Value.GUIDED_LOCAL_SEARCH)
            .setTimeLimit(com.google.protobuf.Duration.newBuilder().setSeconds(2).build())
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
        // This section now identifies rides where the PICKUP task was dropped.
        // If only delivery is dropped (depot drop), it won't be added here.
        for (int i = 0; i < problem.getNumberOfNodes(); ++i) {
            long nodeIndexInRoutingModel = manager.nodeToIndex(i);
            if (routing.isStart(nodeIndexInRoutingModel) || routing.isEnd(nodeIndexInRoutingModel)) {
                continue;
            }
            // A node is considered "dropped" by the solver if its activeVar is 0,
            // or if nextVar(node) == node (meaning it's not part of a route).
            // We are interested in tasks that are not performed.
            if (assignment.value(routing.activeVar(nodeIndexInRoutingModel)) == 0) {
                Task task = problem.getTasksByIndex().get(i);
                if (task != null && task.getType() == TaskType.PICKUP) {
                    RideRequest ride = task.getRide();
                    if (ride != null && !solution.getDroppedRides().contains(ride.getId())) {
                        solution.getDroppedRides().add(ride.getId());
                    }
                }
            }
        }

        /* Populate Depot Dropped Rides Information */
        if (ALLOW_DEPOT_DROP) {
            for (RideRequest rideRequest : problem.getRideRequests()) {
                long pickupNodeOriginalIdx = manager.nodeToIndex(rideRequest.getPickup().getIndex());
                long deliveryNodeOriginalIdx = manager.nodeToIndex(rideRequest.getDelivery().getIndex());

                boolean pickupPerformed = assignment.value(routing.activeVar(pickupNodeOriginalIdx)) == 1;
                boolean deliveryPerformed = assignment.value(routing.activeVar(deliveryNodeOriginalIdx)) == 1;

                if (pickupPerformed && !deliveryPerformed) {
                    // This is a depot drop
                    String vehicleId = null;
                    Depot actualDropDepot = null;
                    Duration timeOfDropAtDepot = null;
                    int vehicleIdxAssignedToPickup = -1;

                    if (assignment.bound(routing.vehicleVar(pickupNodeOriginalIdx))) {
                        vehicleIdxAssignedToPickup = (int) assignment.value(routing.vehicleVar(pickupNodeOriginalIdx));
                    }

                    if (vehicleIdxAssignedToPickup != -1) {
                        Vehicle assignedVehicle = problem.getVehicles().get(vehicleIdxAssignedToPickup);
                        vehicleId = assignedVehicle.getId();
                        actualDropDepot = assignedVehicle.getDepotEnd(); // Assuming drop at assigned vehicle's end depot

                        // Time of drop is the arrival time at the pickup node that is then taken to depot.
                        // TODO: Refine this time to be actual depot arrival time of the vehicle.
                        // This currently represents the service time of the pickup itself.
                        timeOfDropAtDepot = Duration.ofSeconds(assignment.min(timeDimension.cumulVar(pickupNodeOriginalIdx)));

                        DepotDroppedRideInfo depotDropInfo = DepotDroppedRideInfo.builder()
                                .rideId(rideRequest.getId())
                                .userId(rideRequest.getUserId())
                                .originalPickupCoordinates(rideRequest.getPickup().getCoordinates())
                                .originalPickupAddress(rideRequest.getPickup().getAddress())
                                .originalDeliveryCoordinates(rideRequest.getDelivery().getCoordinates())
                                .originalDeliveryAddress(rideRequest.getDelivery().getAddress())
                                .droppedAtDepotId(actualDropDepot != null ? actualDropDepot.getId() : "UNKNOWN_DEPOT")
                                .droppedAtDepotCoordinates(actualDropDepot != null ? actualDropDepot.getCoordinates() : null)
                                .vehicleIdDroppedBy(vehicleId)
                                .timeOfDropAtDepot(timeOfDropAtDepot)
                                .build();
                        solution.getDepotDroppedRides().add(depotDropInfo);
                    }
                }
            }
        }

        /* Routes */
        var timeDimension = routing.getMutableDimension("time");
        var distanceDimension = routing.getMutableDimension("distance");

        for (int vehicle = 0; vehicle < problem.getVehicles().size(); vehicle++) {
            var route = new Route();
            var problemVehicle = problem.getVehicles().get(vehicle);
            route.setVehicleId(problemVehicle.getId());
            route.setTimeWindow(new TimeWindow(
                Duration.ofSeconds(problemVehicle.getTimeWindow().startSeconds()),
                Duration.ofSeconds(problemVehicle.getTimeWindow().endSeconds()))
            );
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

                if (ALLOW_DEPOT_DROP && ride != null && taskNodeIndex.getType() == TaskType.PICKUP) {
                    long deliveryNodeOriginalIdx = manager.nodeToIndex(ride.getDelivery().getIndex());
                    boolean deliveryPerformed = assignment.value(routing.activeVar(deliveryNodeOriginalIdx)) == 1;
                    if (!deliveryPerformed) {
                        // This pickup is part of a ride where the original delivery was not performed (depot drop).
                        visit.setDepotDropPickup(true);
                    }
                }

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
