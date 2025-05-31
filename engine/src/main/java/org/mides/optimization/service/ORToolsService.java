package org.mides.optimization.service;

import com.google.ortools.constraintsolver.*;
import org.mides.optimization.model.*;
import org.mides.optimization.util.Constants;
import org.mides.optimization.util.Utils;
import org.springframework.stereotype.Service;

import java.time.Duration;
import java.util.Objects;

@Service
public class ORToolsService implements IORToolsService {

    /* Max approx. distance between nodes * SpanCostCoefficient */
    private static final long DROP_PENALTY = 10000000 * 100;
    private static final long DEPOT_DROP_PENALTY = DROP_PENALTY / 10;
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

        /* Allow dropping rides and depot drops */
        for (var ride : problem.getRideRequests()) {
            int pickupNodeOriginalIndex = ride.getPickup().getIndex();
            int deliveryNodeOriginalIndex = ride.getDelivery().getIndex();

            long pickupIndex = manager.nodeToIndex(pickupNodeOriginalIndex);
            long deliveryIndex = manager.nodeToIndex(deliveryNodeOriginalIndex);

            // Option 1: Perform the full ride (Pickup and Delivery). Penalty = 0.
            // This is implicitly handled by the pickup-delivery constraints if nodes are not dropped.

            // Option 2: Drop the entire ride (both Pickup and Delivery). Penalty = DROP_PENALTY.
            // This disjunction means if these 2 nodes are not performed, the penalty is incurred.
            // The '2' means we expect 2 nodes (pickup and delivery) to be active for this ride.
            // If less than 2 are active (i.e. 0, since P/D constraint links them or they are both dropped), penalty applies.
            routing.addDisjunction(new long[]{pickupIndex, deliveryIndex}, DROP_PENALTY, 2);

            if (Constants.ALLOW_DEPOT_DROP) {
                // Option 3: Perform Pickup, but drop original Delivery (for depot drop). Penalty = DEPOT_DROP_PENALTY.
                // This disjunction means if the deliveryNode is not performed, DEPOT_DROP_PENALTY is incurred.
                // The '1' means we expect 1 node (the delivery node) to be active. If it's not, penalty applies.
                // This penalty should be less than DROP_PENALTY.
                // If this disjunction is chosen (deliveryNode is dropped), the AddPickupAndDelivery constraint
                // for this pair will be skipped by OR-Tools, allowing pickup to occur without this specific delivery.
                routing.addDisjunction(new long[]{deliveryIndex}, DEPOT_DROP_PENALTY, 1);
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
        var solver = routing.solver();
        var seatDemands = problem.getSeatDemands();
        final int seatDemandCallbackIndex = routing.registerUnaryTransitCallback((long fromIndex) -> {
            int fromNode = manager.indexToNode(fromIndex);
            return seatDemands[fromNode];
        });

        var baseSeatCapacities = problem.getSeatCapacities(); // Get original capacities
        long[] adjustedSeatCapacities = new long[baseSeatCapacities.length];
        System.arraycopy(baseSeatCapacities, 0, adjustedSeatCapacities, 0, baseSeatCapacities.length);

        var baseWheelchairCapacities = problem.getWheelchairCapacities();
        long[] adjustedWheelchairCapacities = new long[baseWheelchairCapacities.length];
        System.arraycopy(baseWheelchairCapacities, 0, adjustedWheelchairCapacities, 0, baseWheelchairCapacities.length);

        for (int i = 0; i < problem.getVehicles().size(); i++) {
            Vehicle vehicle = problem.getVehicles().get(i);
            if (vehicle.getActiveRideIdPreBoarded() != null) {
                RideRequest preBoardedRide = problem.getRideRequestById(vehicle.getActiveRideIdPreBoarded());
                if (preBoardedRide != null) {
                    long seatDemandOfPreBoarded = 0;
                    long wheelchairDemandOfPreBoarded = 0;

                    if (preBoardedRide.getPickup() != null) {
                        // The problem statement implies pickup node original index is valid for demand arrays
                        int pickupNodeOriginalIndex = preBoardedRide.getPickup().getIndex();
                        // Ensure index is within bounds for safety, though problem.getSeatDemands() should be sized for all nodes
                        if (pickupNodeOriginalIndex >= 0 && pickupNodeOriginalIndex < seatDemands.length) {
                             seatDemandOfPreBoarded = seatDemands[pickupNodeOriginalIndex];
                        }
                        if (pickupNodeOriginalIndex >= 0 && pickupNodeOriginalIndex < problem.getWheelchairDemands().length) {
                            wheelchairDemandOfPreBoarded = problem.getWheelchairDemands()[pickupNodeOriginalIndex];
                        }
                    }

                    adjustedSeatCapacities[i] -= seatDemandOfPreBoarded;
                    adjustedWheelchairCapacities[i] -= wheelchairDemandOfPreBoarded;
                }
            }
        }

        routing.addDimensionWithVehicleCapacity(
            seatDemandCallbackIndex,
            0, /* no slack */
            adjustedSeatCapacities, /* vehicle_capacities */
            true, /* fix_start_cumul_to_zero */
            "seat"
        );

        /* Add wheelchair capacity constraints */
        var wheelchairDemands = problem.getWheelchairDemands(); // This is already defined above, but needed for context if we split the diff. Keeping for clarity.
        final int wheelchairDemandCallbackIndex = routing.registerUnaryTransitCallback((long fromIndex) -> { // Also defined above.
            int fromNode = manager.indexToNode(fromIndex);
            return wheelchairDemands[fromNode];
        });

        // var wheelchairCapacities = problem.getWheelchairCapacities(); // This was the original line for baseWheelchairCapacities

        routing.addDimensionWithVehicleCapacity(
            wheelchairDemandCallbackIndex,
            0,
            adjustedWheelchairCapacities,
            true,
            "wheelchair"
        );

        /* Enforce Delivery by the Assigned Vehicle for Pre-Boarded Rides */
        // Note: solver is already initialized before capacity dimensions.
        for (int currentVehicleIndex = 0; currentVehicleIndex < problem.getVehicles().size(); currentVehicleIndex++) {
            Vehicle vehicle = problem.getVehicles().get(currentVehicleIndex);
            if (vehicle.getActiveRideIdPreBoarded() != null) {
                RideRequest preBoardedRide = problem.getRideRequestById(vehicle.getActiveRideIdPreBoarded());
                if (preBoardedRide != null && preBoardedRide.getDelivery() != null) {
                    long deliveryNodeIndex = manager.nodeToIndex(preBoardedRide.getDelivery().getIndex());

                    solver.addConstraint(solver.makeEquality(routing.vehicleVar(deliveryNodeIndex), currentVehicleIndex));
                    solver.addConstraint(solver.makeEquality(routing.activeVar(deliveryNodeIndex), 1));
                }
            }
        }

        /* Add pickup-delivery constraints */
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

        /* Add maximum ride time for a single pickup-delivery constraint */
        for (var ride : problem.getRideRequests()) {
            var pickupIndex = manager.nodeToIndex(ride.getPickup().getIndex());
            var deliveryIndex = manager.nodeToIndex(ride.getDelivery().getIndex());
            solver.addConstraint(solver.makeLessOrEqual(
                timeDimension.cumulVar(deliveryIndex),
                solver.makeSum(timeDimension.cumulVar(pickupIndex), MAX_RIDE_TIME)
            ));
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
                solver.addConstraint(solver.makeLessOrEqual(isVehicleUsedForPickup, isDeliveryTimeGreaterThanStart));
                solver.addConstraint(solver.makeLessOrEqual(isVehicleUsedForPickup, isDeliveryTimeLessThanEnd));
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

        /* Dropped rides and Depot Drops */
        solution.getDroppedRides().clear(); // Clear if it was initialized with data or ensure it's empty
        solution.getDepotDroppedRides().clear(); // Ensure the new list is also empty

        for (var rideRequest : problem.getRideRequests()) {
            int pickupNodeOriginalIndex = rideRequest.getPickup().getIndex();
            int deliveryNodeOriginalIndex = rideRequest.getDelivery().getIndex();

            // Convert original node indices to routing model indices
            long pickupRoutingIndex = manager.nodeToIndex(pickupNodeOriginalIndex);
            long deliveryRoutingIndex = manager.nodeToIndex(deliveryNodeOriginalIndex);

            // Check if pickup and delivery nodes are active in the solution
            // An activeVar(index) is 1 if the node is performed, 0 otherwise (dropped).
            boolean pickupIsActive = assignment.value(routing.activeVar(pickupRoutingIndex)) == 1;
            boolean deliveryIsActive = assignment.value(routing.activeVar(deliveryRoutingIndex)) == 1;

            String rideId = rideRequest.getId();

            if (Constants.ALLOW_DEPOT_DROP && pickupIsActive && !deliveryIsActive) {
                // Case: Pickup happened, but delivery didn't (and depot drops are allowed)
                // This is a depot drop.
                if (rideId != null && !solution.getDepotDroppedRides().contains(rideId)) {
                    solution.getDepotDroppedRides().add(rideId);
                }
            } else if (!pickupIsActive) {
                // Case: Pickup didn't happen. This means the entire ride is dropped,
                // regardless of delivery status (which should also be inactive).
                // Also covers cases where pickup is inactive and delivery is somehow active (should not happen).
                if (rideId != null && !solution.getDroppedRides().contains(rideId)) {
                    solution.getDroppedRides().add(rideId);
                }
            }
            // If pickupIsActive && deliveryIsActive, the ride is fully served, so no addition to either list.
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
