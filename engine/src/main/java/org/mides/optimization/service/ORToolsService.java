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

    // Constants for stop times
    private static final long TIME_STOP_COMMON = 2 * 60; // 2 minute
    private static final long TIME_STOP_WHEELCHAIR = 5 * 60; // 5 minutes
    private static final long TIME_STOP_ELECTRIC_RAMP = 5 * 60; // 5 minutes


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
            
            // Base travel time between nodes
            long travelTime = timeMatrix[fromNode][toNode];
            
            // Add stop time when leaving a pickup/delivery node (not depot nodes)
            long stopTime = 0;
            if (fromNode >= problem.getVehicles().size() * 2) { // Not a depot node
                var taskNode = problem.getTasksByIndex().get(fromNode);
                var ride = taskNode.getRide();
                
                if (ride != null) {
                    // Check if ride requires electric ramp
                    if (ride.getCharacteristics() != null && ride.getCharacteristics().contains("rampa_electrica")) {
                        stopTime = TIME_STOP_ELECTRIC_RAMP;
                    } else if (ride.isWheelchairRequired()) {
                        stopTime = TIME_STOP_WHEELCHAIR;
                    } else {
                        stopTime = TIME_STOP_COMMON;
                    }
                } else {
                    stopTime = TIME_STOP_COMMON;
                }
            }
            
            return travelTime + stopTime;
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
            
            // Ensure vehicle end depot respects time window for ALL vehicles
            var endIndex = routing.end(vehicleIndex);
            timeDimension.cumulVar(endIndex).setRange(
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

        // Register a callback that returns 0 for pre/post travel time for breaks.
        // This means breaks don't add any extra travel time beyond their own duration.
        final int zeroTransitCallbackIndex = routing.registerTransitCallback(
            (long fromIndex, long toIndex) -> 0L
        );

        // Add break constraints for vehicles with with_rest = true
        for (int i = 0; i < problem.getVehicles().size(); i++) {
            Vehicle vehicle = problem.getVehicles().get(i);
            if (vehicle.isWithRest()) {
                long vehicleStartTwSeconds = vehicle.getTimeWindow().startSeconds();
                long vehicleEndTwSeconds = vehicle.getTimeWindow().endSeconds();

                // Define the IntervalVar for the break with the widest possible valid window
                // based on the vehicle's absolute Time Window.
                // The break must be able to complete within the vehicle's time window.
                long domainMinBreakStart = vehicleStartTwSeconds;
                long domainMaxBreakStart = vehicleEndTwSeconds - REST_TIME_SECONDS;

                if (domainMinBreakStart <= domainMaxBreakStart) { // Check if vehicle TW is long enough for a break
                    String breakName = "Rest_V" + vehicle.getId() + "_idx" + i;
                    IntervalVar breakInterval = solver.makeFixedDurationIntervalVar(
                            domainMinBreakStart,
                            domainMaxBreakStart,
                            REST_TIME_SECONDS,
                            false, // Break is not optional
                            breakName
                    );
                    vehicleBreakIntervals.put(i, breakInterval); // Store for later retrieval

                    // Add dynamic constraints to tie the break to actual vehicle operation times
                    IntVar actualDepotStartTime = timeDimension.cumulVar(routing.start(i));
                    IntVar actualDepotEndTime = timeDimension.cumulVar(routing.end(i));

                    // Constraint 1: Break must start at least REST_TIME_MIN_START_AFTER_RIDE_START_SECONDS
                    // after the vehicle actually leaves the depot.
                    solver.addConstraint(solver.makeGreaterOrEqual(
                        breakInterval.startExpr(),
                        solver.makeSum(actualDepotStartTime, REST_TIME_MIN_START_AFTER_RIDE_START_SECONDS)
                    ));

                    // Constraint 2: Break must be completed at least REST_TIME_MAX_START_BEFORE_RIDE_END_SECONDS
                    // before the vehicle actually arrives at the end depot.
                    // breakInterval.endExpr() <= actualDepotEndTime - REST_TIME_MAX_START_BEFORE_RIDE_END_SECONDS
                    solver.addConstraint(solver.makeLessOrEqual(
                        breakInterval.endExpr(),
                        solver.makeSum(actualDepotEndTime, -REST_TIME_MAX_START_BEFORE_RIDE_END_SECONDS) // actualDepotEndTime - offset
                    ));

                    // Constraint 3: Ensure actual depot end time respects vehicle time window
                    // This prevents the vehicle from arriving at depot later than its allowed time window
                    solver.addConstraint(solver.makeLessOrEqual(
                        actualDepotEndTime,
                        vehicleEndTwSeconds
                    ));

                    // Add constraint: break cannot happen during a ride on this vehicle
                    for (RideRequest ride : problem.getRideRequests()) {
                        var pickupNodeForRide = ride.getPickup().getIndex();
                        var deliveryNodeForRide = ride.getDelivery().getIndex();

                        IntVar pickupTime = timeDimension.cumulVar(manager.nodeToIndex(pickupNodeForRide));
                        IntVar deliveryTime = timeDimension.cumulVar(manager.nodeToIndex(deliveryNodeForRide));

                        // isRideOnThisVehicle = (vehicleVar(pickup of ride) == current vehicle index i)
                        IntVar isRideOnThisVehicle = solver.makeIsEqualCstVar(routing.vehicleVar(manager.nodeToIndex(pickupNodeForRide)), i);

                        // breakEndsBeforePickup = (breakInterval.endExpr() <= pickupTime)
                        IntVar breakEndsBeforePickup = solver.makeIsLessOrEqualVar(breakInterval.endExpr(), pickupTime);
                        // breakStartsAfterDelivery = (breakInterval.startExpr() >= deliveryTime)
                        IntVar breakStartsAfterDelivery = solver.makeIsGreaterOrEqualVar(breakInterval.startExpr(), deliveryTime);

                        // disjunctionIsMet = (breakEndsBeforePickup OR breakStartsAfterDelivery)
                        // This is true if (sum of boolean vars for conditions) >= 1
                        IntVar disjunctionIsMet = solver.makeIsGreaterOrEqualCstVar(
                            solver.makeSum(breakEndsBeforePickup, breakStartsAfterDelivery), 1L
                        );
                        
                        // Implication: if isRideOnThisVehicle is true, then disjunctionIsMet must be true.
                        // This can be expressed as: isRideOnThisVehicle <= disjunctionIsMet
                        solver.addConstraint(solver.makeLessOrEqual(isRideOnThisVehicle, disjunctionIsMet));
                    }

                    List<IntervalVar> vehicleBreaksList = new ArrayList<>();
                    vehicleBreaksList.add(breakInterval);
                    IntervalVar[] vehicleBreaksArray = vehicleBreaksList.toArray(new IntervalVar[0]);

                    logger.info("Setting break for vehicle ID: {} (model idx: {}): TW-based start range [{}, {}], Duration {}. Dynamic constraints linked to actual route times apply.",
                        vehicle.getId(), i,
                        Duration.ofSeconds(domainMinBreakStart),
                        Duration.ofSeconds(domainMaxBreakStart),
                        Duration.ofSeconds(REST_TIME_SECONDS));
                    
                    timeDimension.setBreakIntervalsOfVehicle(
                        vehicleBreaksArray, 
                        i, // vehicle index
                        zeroTransitCallbackIndex, // pre_travel_evaluator index
                        zeroTransitCallbackIndex  // post_travel_evaluator index
                    );
                } else {
                    // This warning means the vehicle's own TimeWindow is too short for the break duration.
                    logger.warn("Cannot schedule mandatory rest for vehicle ID: {} (model idx: {}). Vehicle TimeWindow [{}, {}] (duration {}s) is too short for rest duration ({}s). Break needs to start by {} at the latest within TW to fit.",
                            vehicle.getId(), i,
                            vehicle.getTimeWindow().getStart(), vehicle.getTimeWindow().getEnd(),
                            (vehicleEndTwSeconds - vehicleStartTwSeconds),
                            REST_TIME_SECONDS,
                            Duration.ofSeconds(domainMaxBreakStart));
                }
            }
        }

        var searchParams = main.defaultRoutingSearchParameters()
            .toBuilder()
            .setFirstSolutionStrategy(FirstSolutionStrategy.Value.AUTOMATIC)
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
            Solution emptySolution = new Solution();
            emptySolution.setDroppedRides(
                problem.getRideRequests().stream().map(RideRequest::getId).toList()
            );
            return emptySolution;
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
            int droppedNodeOriginalIndex = manager.indexToNode(node);
            var task = problem.getTasksByIndex().get(droppedNodeOriginalIndex);
            if (droppedNodeOriginalIndex >= problem.getVehicles().size() * 2) {
                 var ride = task.getRide();
                 var rideId = ride != null ? ride.getId() : null;

                 if (rideId != null && !solution.getDroppedRides().contains(rideId)) {
                     solution.getDroppedRides().add(rideId);
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
            int nodeIndexInProblem;
            int position = 0;
            long currentSolverIndex = routing.start(vehicle);

            if (assignment.value(routing.nextVar(currentSolverIndex)) == routing.end(vehicle)) {
                continue;
            }
            
            while (!routing.isEnd(currentSolverIndex)) {
                nodeIndexInProblem = manager.indexToNode(currentSolverIndex);
                var taskNode = problem.getTasksByIndex().get(nodeIndexInProblem);
                long nextSolverIndexValue = assignment.value(routing.nextVar(currentSolverIndex));
                int nextNodeIndexInProblem = manager.indexToNode(nextSolverIndexValue);

                var ride = taskNode.getRide();
                Visit visit = new Visit();
                visit.setPosition(position++);
                visit.setRideId(ride != null ? ride.getId() : null);
                visit.setUserId(ride != null ? ride.getUserId() : null);
                visit.setRideDirection(ride != null ? ride.getDirection() : null);
                visit.setAddress(taskNode.getAddress());
                visit.setCoordinates(taskNode.getCoordinates());
                visit.setArrivalTime(Duration.ofSeconds(assignment.min(timeDimension.cumulVar(currentSolverIndex))));
                visit.setTravelTimeToNextVisit(Duration.ofSeconds(timeMatrix[nodeIndexInProblem][nextNodeIndexInProblem]));
                visit.setSolutionWindow(new TimeWindow(
                    Duration.ofSeconds(assignment.min(timeDimension.cumulVar(currentSolverIndex))),
                    Duration.ofSeconds(assignment.max(timeDimension.cumulVar(currentSolverIndex)))
                ));
                visit.setType(taskNode.getType());
                visit.setStopId(taskNode.getStopId());

                route.getVisits().add(visit);
                currentSolverIndex = nextSolverIndexValue;
            }

            nodeIndexInProblem = manager.indexToNode(currentSolverIndex); 
            var endDepotTaskNode = problem.getTasksByIndex().get(nodeIndexInProblem);
            var endDepotRide = endDepotTaskNode.getRide(); 

            Visit lastVisit = new Visit();
            lastVisit.setPosition(position);
            lastVisit.setRideId(endDepotRide != null ? endDepotRide.getId() : null);
            lastVisit.setUserId(endDepotRide != null ? endDepotRide.getUserId() : null);
            lastVisit.setRideDirection(endDepotRide != null ? endDepotRide.getDirection() : null);
            lastVisit.setAddress(endDepotTaskNode.getAddress());
            lastVisit.setCoordinates(endDepotTaskNode.getCoordinates());
            lastVisit.setArrivalTime(Duration.ofSeconds(assignment.min(timeDimension.cumulVar(currentSolverIndex))));
            lastVisit.setTravelTimeToNextVisit(Duration.ZERO); 
            lastVisit.setSolutionWindow(new TimeWindow(
                Duration.ofSeconds(assignment.min(timeDimension.cumulVar(currentSolverIndex))),
                Duration.ofSeconds(assignment.max(timeDimension.cumulVar(currentSolverIndex)))
            ));
            lastVisit.setType(endDepotTaskNode.getType());
            lastVisit.setStopId(endDepotTaskNode.getStopId());

            route.getVisits().add(lastVisit);

            if (problemVehicle.isWithRest() && vehicleBreakIntervals.containsKey(vehicle)) {
                IntervalVar breakVar = vehicleBreakIntervals.get(vehicle);
                if (breakVar != null && assignment.performedValue(breakVar) == 1) {
                    long breakStart = assignment.startValue(breakVar);
                    long breakEnd = assignment.endValue(breakVar);
                    if (breakStart >=0 && breakEnd >=0 && breakStart < breakEnd) {
                        route.setRestTimeWindow(new TimeWindow(Duration.ofSeconds(breakStart), Duration.ofSeconds(breakEnd)));
                         logger.info("Vehicle ID: {} (model idx: {}) assigned rest: Start: {}, End: {}",
                            problemVehicle.getId(), vehicle,
                            Duration.ofSeconds(breakStart), Duration.ofSeconds(breakEnd));
                    } else {
                        logger.warn("Vehicle ID: {} (model idx: {}) had a mandatory rest defined, but couldn't retrieve valid start/end times from assignment. Start: {}, End: {}. Break Performed: {}",
                                problemVehicle.getId(), vehicle, breakStart, breakEnd, assignment.performedValue(breakVar));
                    }
                } else if (breakVar != null) {
                     logger.warn("Vehicle ID: {} (model idx: {}) had a mandatory rest defined, but it was not performed in the solution. Break Performed: {}",
                                problemVehicle.getId(), vehicle, assignment.performedValue(breakVar));
                }
            }

            if (!route.getVisits().isEmpty()) {
                var endTime = route.getVisits().get(route.getVisits().size() - 1).getArrivalTime();
                var startTime = route.getVisits().get(0).getArrivalTime();
                route.setDuration(endTime.minus(startTime));
                route.setDistance(Utils.convertDistanceBack(assignment.value(distanceDimension.cumulVar(currentSolverIndex))));
                solution.getRoutes().add(route);
            }
        }
        return solution;
    }
}
