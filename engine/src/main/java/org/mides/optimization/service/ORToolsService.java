package org.mides.optimization.service;

import com.google.ortools.constraintsolver.*;
import org.mides.optimization.model.*;
import org.mides.optimization.util.Utils;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.springframework.stereotype.Service;

import java.time.Duration;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;

@Service
public class ORToolsService implements IORToolsService {

    private static final Logger logger = LoggerFactory.getLogger(ORToolsService.class);

    /* Max approx. distance between nodes * SpanCostCoefficient */
    private static final long DROP_PENALTY = 10000000 * 100;
    private static final long MAX_RIDE_TIME = 5000; // seconds, approx 1h23m

    // Constants for vehicle rests
    private static final long REST_TIME_SECONDS = 30 * 60; // 30 minutes
    private static final long REST_TIME_MIN_START_AFTER_RIDE_START_SECONDS = 1 * 60 * 60; // 1 hour
    private static final long REST_TIME_MAX_START_BEFORE_RIDE_END_SECONDS = 1 * 60 * 60; // 1 hour


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
        Solver solver = routing.solver();

        // Map to store break interval variables for each vehicle that has a rest
        Map<Integer, IntervalVar> vehicleBreakIntervals = new HashMap<>();

        /* Allow dropping nodes */
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

        // Add break constraints for vehicles with with_rest = true
        for (int i = 0; i < problem.getVehicles().size(); i++) {
            Vehicle vehicle = problem.getVehicles().get(i);
            if (vehicle.isWithRest()) {
                long vehicleStartTwSeconds = vehicle.getTimeWindow().startSeconds();
                long vehicleEndTwSeconds = vehicle.getTimeWindow().endSeconds();

                long earliestBreakStart = vehicleStartTwSeconds + REST_TIME_MIN_START_AFTER_RIDE_START_SECONDS;
                long latestBreakStartByRule = vehicleEndTwSeconds - REST_TIME_MAX_START_BEFORE_RIDE_END_SECONDS;
                long latestBreakStartByVehicleEnd = vehicleEndTwSeconds - REST_TIME_SECONDS;

                long actualMinBreakStart = earliestBreakStart;
                long actualMaxBreakStart = Math.min(latestBreakStartByRule, latestBreakStartByVehicleEnd);

                if (actualMinBreakStart <= actualMaxBreakStart) {
                    String breakName = "Rest_V" + vehicle.getId() + "_idx" + i;
                    IntervalVar breakInterval = solver.makeFixedDurationIntervalVar(
                            actualMinBreakStart,
                            actualMaxBreakStart,
                            REST_TIME_SECONDS,
                            false, // Break is not optional
                            breakName
                    );
                    vehicleBreakIntervals.put(i, breakInterval); // Store for later retrieval

                    List<IntervalVar> vehicleBreaksList = new ArrayList<>();
                    vehicleBreaksList.add(breakInterval);
                    
                    List<Long> breakDurationsList = new ArrayList<>();
                    breakDurationsList.add(REST_TIME_SECONDS); 

                    IntervalVar[] vehicleBreaksArray = vehicleBreaksList.toArray(new IntervalVar[0]);
                    long[] breakDurationsArray = breakDurationsList.stream().mapToLong(l -> l).toArray();

                    logger.info("Setting break for vehicle {} (idx {}): Start window [{}, {}], Duration {}",
                        vehicle.getId(), i,
                        Duration.ofSeconds(actualMinBreakStart), 
                        Duration.ofSeconds(actualMaxBreakStart), 
                        Duration.ofSeconds(REST_TIME_SECONDS));
                    
                    timeDimension.setBreakIntervalsOfVehicle(vehicleBreaksArray, i, breakDurationsArray);
                } else {
                    logger.warn("Cannot schedule mandatory rest for vehicle {} (idx {}, ID: {}). Calculated break start window is invalid: minStart={}, maxStart={}. Vehicle TW: [{}, {}]",
                            i, vehicle.getId(),
                            Duration.ofSeconds(actualMinBreakStart), Duration.ofSeconds(actualMaxBreakStart),
                            vehicle.getTimeWindow().getStart(), vehicle.getTimeWindow().getEnd());
                }
            }
        }

        var searchParams = main.defaultRoutingSearchParameters()
            .toBuilder()
            .setFirstSolutionStrategy(FirstSolutionStrategy.Value.PATH_CHEAPEST_ARC)
            .setLocalSearchMetaheuristic(LocalSearchMetaheuristic.Value.GUIDED_LOCAL_SEARCH)
            .setTimeLimit(com.google.protobuf.Duration.newBuilder().setSeconds(5).build())
            .build();

        var assignment = routing.solveWithParameters(searchParams);

        return buildSolution(problem, routing, manager, assignment, timeMatrix, vehicleBreakIntervals);
    }


    private static Solution buildSolution(
        Problem problem,
        RoutingModel routing,
        RoutingIndexManager manager,
        Assignment assignment,
        long[][] timeMatrix,
        Map<Integer, IntervalVar> vehicleBreakIntervals 
    ) {
        if (assignment == null) {
            logger.error("No solution found. Problem might be infeasible.");
            // Return an empty solution or throw, depending on desired behavior for infeasibility
            Solution emptySolution = new Solution();
            emptySolution.setDroppedRides(
                problem.getRideRequests().stream().map(RideRequest::getId).toList()
            ); // Mark all rides as dropped
            return emptySolution;
            // Or: throw new IllegalArgumentException("Unfeasible problem or solver failed to find a solution.");
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
            route.setTimeWindow(new TimeWindow(
                Duration.ofSeconds(problemVehicle.getTimeWindow().startSeconds()),
                Duration.ofSeconds(problemVehicle.getTimeWindow().endSeconds()))
            );
            int nodeIndex;
            int position = 0;
            long index = routing.start(vehicle);
            if (assignment.value(routing.nextVar(index)) == routing.end(vehicle)) {
                // Skip adding this route if it's empty based on current logic
                continue; 
            }
            
            while (!routing.isEnd(index)) {
                nodeIndex = manager.indexToNode(index);
                var taskNodeIndex = problem.getTasksByIndex().get(nodeIndex);
                long nextVarValue = assignment.value(routing.nextVar(index));
                int nextNodeIndex = manager.indexToNode(nextVarValue);

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
                index = nextVarValue;
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

            // Populate rest time window if applicable
            if (problemVehicle.isWithRest() && vehicleBreakIntervals.containsKey(vehicle)) {
                IntervalVar breakVar = vehicleBreakIntervals.get(vehicle);
                if (breakVar != null) { // Ensure breakVar was created
                    long breakStart = assignment.startValue(breakVar);
                    long breakEnd = assignment.endValue(breakVar); // For fixed duration: start + duration
                    if (breakStart >=0 && breakEnd >=0 && breakStart < breakEnd) { // Basic sanity check
                        route.setRestTimeWindow(new TimeWindow(Duration.ofSeconds(breakStart), Duration.ofSeconds(breakEnd)));
                         logger.info("Vehicle {} (idx {}) assigned rest: Start: {}, End: {}",
                            problemVehicle.getId(), vehicle,
                            Duration.ofSeconds(breakStart), Duration.ofSeconds(breakEnd));
                    } else {
                        logger.warn("Vehicle {} (idx {}) had a break defined, but couldn't retrieve valid start/end times from assignment. Start: {}, End: {}",
                                problemVehicle.getId(), vehicle, breakStart, breakEnd);
                    }
                }
            }

            var endTime = route.getVisits().get(route.getVisits().size() - 1).getArrivalTime();
            var startTime = route.getVisits().get(0).getArrivalTime();
            route.setDuration(endTime.minus(startTime));
            route.setDistance(Utils.convertDistanceBack(assignment.value(distanceDimension.cumulVar(index))));
            solution.getRoutes().add(route);
        }
        return solution;
    }
}
