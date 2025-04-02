package org.mides.optimization.service;

import com.google.ortools.Loader; // Make sure Loader is loaded (usually via BootstrapConfiguration)
import com.google.ortools.constraintsolver.*;
import com.google.protobuf.Duration; // Use protobuf Duration for OR-Tools config
import org.mides.optimization.model.*;
// Import Task Type constants if defined in Problem or elsewhere
import static org.mides.optimization.model.Problem.*; // Import static constants

import org.mides.optimization.util.Utils;
import org.slf4j.Logger; // Add logging
import org.slf4j.LoggerFactory; // Add logging
import org.springframework.stereotype.Service;

// import java.time.Duration; // Keep this for model objects like TimeWindow, Visit

@Service
public class ORToolsService implements IORToolsService {

    private static final Logger logger = LoggerFactory.getLogger(ORToolsService.class);

    /* Max approx. distance between nodes * SpanCostCoefficient */
    // Consider making DROP_PENALTY configurable or based on max possible cost
    private static final long DROP_PENALTY = 10000000L * 100L; // Use L for long literals
    private static final long MAX_RIDE_TIME = 3600L; // 1 hour in seconds
    private static final String TOTAL_CAPACITY_DIMENSION = "Capacity"; // Keep original name for minimal change
    private static final String WHEELCHAIR_CAPACITY_DIMENSION = "WheelchairCapacity";


    // Static block to ensure native libraries are loaded (alternative to BootstrapConfiguration)
    // static {
    //     try {
    //          System.loadLibrary("jniortools"); // Or Loader.loadNativeLibraries();
    //     } catch (UnsatisfiedLinkError e) {
    //          logger.error("Native code library failed to load.\n" + e);
    //          System.exit(1);
    //     }
    // }


    @Override
    public Solution solve(Problem problem, long[][] distanceMatrix, long[][] timeMatrix) {
        logger.info("Starting OR-Tools solver for {} vehicles and {} ride requests.",
            problem.getVehicles().size(), problem.getRideRequests().size());

        // --- Initialization ---
        int numVehicles = problem.getVehicles().size();
        int numNodes = problem.getNumberOfNodes();
        var starts = new int[numVehicles];
        var ends = new int[numVehicles];

        for (int i = 0; i < numVehicles; i++) {
            var vehicle = problem.getVehicles().get(i);
            starts[i] = vehicle.getDepotStart().getIndex();
            ends[i] = vehicle.getDepotEnd().getIndex();
            logger.debug("Vehicle {} [ID: {}]: Start Node {}, End Node {}", i, vehicle.getId(), starts[i], ends[i]);
        }

        var manager = new RoutingIndexManager(numNodes, numVehicles, starts, ends);
        var routing = new RoutingModel(manager);
        var solver = routing.solver(); // Get solver instance

        // --- Disjunctions (Allow Dropping Nodes) ---
        int firstTaskNodeIndex = problem.getFirstTaskNodeIndex(); // Use helper method
        logger.debug("First task node index (after depots): {}", firstTaskNodeIndex);
        for (int i = firstTaskNodeIndex; i < numNodes; i++) {
            PickupDeliveryTask task = problem.getTasksByIndex().get(i);
            // Only add disjunction for PICKUP nodes to represent dropping a whole ride
            if (task != null && TASK_TYPE_PICKUP.equals(task.getType())) {
                long nodeRoutingIndex = manager.nodeToIndex(i);
                logger.trace("Adding disjunction for node {} (Task Index {}) with penalty {}", nodeRoutingIndex, i, DROP_PENALTY);
                routing.addDisjunction(new long[]{nodeRoutingIndex}, DROP_PENALTY);
            }
        }

        // --- Distance Dimension ---
        final int distanceCallbackIndex = routing.registerTransitCallback((fromIndex, toIndex) -> {
            int fromNode = manager.indexToNode(fromIndex);
            int toNode = manager.indexToNode(toIndex);
            // Add basic bounds check for safety
            if (fromNode < 0 || fromNode >= distanceMatrix.length || toNode < 0 || toNode >= distanceMatrix[0].length) {
                 logger.error("Distance Matrix access out of bounds: fromNode={}, toNode={}", fromNode, toNode);
                 return Long.MAX_VALUE; // Penalize invalid access
            }
            return distanceMatrix[fromNode][toNode];
        });
        routing.addDimension(
            distanceCallbackIndex,
            0, // No slack allowed for distance
            Long.MAX_VALUE, // Max total distance per vehicle (can be constrained further if needed)
            true, // Force start cumul to zero
            "distance"
        );
        routing.setArcCostEvaluatorOfAllVehicles(distanceCallbackIndex); // Minimize total distance
        var distanceDimension = routing.getMutableDimension("distance");
        distanceDimension.setGlobalSpanCostCoefficient(100); // Penalize total span (longest route distance)

        // --- Time Dimension ---
        final int timeCallbackIndex = routing.registerTransitCallback((fromIndex, toIndex) -> {
            int fromNode = manager.indexToNode(fromIndex);
            int toNode = manager.indexToNode(toIndex);
             // Add basic bounds check for safety
            if (fromNode < 0 || fromNode >= timeMatrix.length || toNode < 0 || toNode >= timeMatrix[0].length) {
                 logger.error("Time Matrix access out of bounds: fromNode={}, toNode={}", fromNode, toNode);
                 return Long.MAX_VALUE; // Penalize invalid access
            }
            // Optional: Add service time here if applicable
            // long serviceTime = problem.getTasksByIndex().get(fromNode).getServiceTime(); // Example
            return timeMatrix[fromNode][toNode]; // + serviceTime;
        });
        routing.addDimension(
            timeCallbackIndex,
            Long.MAX_VALUE, // Slack (allowed waiting time) - adjust if needed (e.g., 3600 for 1hr max wait)
            Long.MAX_VALUE, // Max total time per vehicle (e.g., 8 * 3600 for 8 hours)
            false, // Don't force start cumulative to zero (depots have own time windows)
            "time"
        );
        var timeDimension = routing.getMutableDimension("time");

        // Apply Time Windows to Tasks
        for (int i = firstTaskNodeIndex; i < numNodes; i++) {
            PickupDeliveryTask task = problem.getTasksByIndex().get(i);
            if (task != null && task.getTimeWindow() != null) {
                long index = manager.nodeToIndex(i);
                long startSeconds = task.getTimeWindow().startSeconds();
                long endSeconds = task.getTimeWindow().endSeconds();
                 // Ensure start <= end, otherwise OR-Tools might error
                 if (startSeconds > endSeconds) {
                      logger.warn("Task index {} (Ride {}) has invalid time window [{}, {}]. Using [{}, {}].",
                           i, task.getRide() != null ? task.getRide().getId() : "N/A", startSeconds, endSeconds, startSeconds, startSeconds);
                      endSeconds = startSeconds; // Or handle as error
                 }
                 logger.trace("Setting time window for node {} (Task Index {}): [{}, {}]", index, i, startSeconds, endSeconds);
                timeDimension.cumulVar(index).setRange(startSeconds, endSeconds);

                if (TASK_TYPE_PICKUP.equals(task.getType())) {
                    timeDimension.slackVar(index).setValue(0);
                }
                    routing.addToAssignment(timeDimension.slackVar(index));
            } else if (task == null) {
                 logger.warn("Task not found for index {} while setting time windows.", i);
            }
        }

        // Apply Time Windows to Depots
        for (int i = 0; i < numVehicles; i++) {
            Vehicle vehicle = problem.getVehicles().get(i);
            long startIndex = routing.start(i);
            long endIndex = routing.end(i);

            long startDepotStart = vehicle.getDepotStart().getTimeWindow().startSeconds();
            long startDepotEnd = vehicle.getDepotStart().getTimeWindow().endSeconds(); // Usually start == end for start depot
            long endDepotStart = vehicle.getDepotEnd().getTimeWindow().startSeconds();
            long endDepotEnd = vehicle.getDepotEnd().getTimeWindow().endSeconds();

            logger.trace("Vehicle {} Start Depot Node {}: Time Window [{}, {}]", i, startIndex, startDepotStart, startDepotEnd);
            logger.trace("Vehicle {} End Depot Node {}: Time Window [{}, {}]", i, endIndex, endDepotStart, endDepotEnd);

            timeDimension.cumulVar(startIndex).setRange(startDepotStart, startDepotEnd);
            timeDimension.cumulVar(endIndex).setRange(endDepotStart, endDepotEnd);
        }

        // --- Time-related Objectives (Optional) ---
        // Minimize latest arrival at end depot (makespan)
        // for (int i = 0; i < numVehicles; i++) {
        //     routing.addVariableMinimizedByFinalizer(timeDimension.cumulVar(routing.end(i)));
        // }


        // --- Capacity Constraints ---

        // 1. Total Capacity Dimension
        final int totalCapacityCallbackIndex = routing.registerUnaryTransitCallback((long fromIndex) -> {
            int fromNode = manager.indexToNode(fromIndex);
            PickupDeliveryTask task = problem.getTasksByIndex().get(fromNode);

            if (task == null) {
                 logger.warn("Total Capacity Callback: Task not found for node index {}", fromNode);
                 return 0;
            }

            String taskType = task.getType();
            RideRequest ride = task.getRide(); // Ride is null for depots

            if (TASK_TYPE_PICKUP.equals(taskType) && ride != null) {
                return ride.isHasCompanion() ? 2 : 1; // +1 or +2 for pickup
            } else if (TASK_TYPE_DELIVERY.equals(taskType) && ride != null) {
                return ride.isHasCompanion() ? -2 : -1; // -1 or -2 for delivery
            } else {
                return 0; // Depots have 0 demand change
            }
        });

        long[] vehicleTotalCapacities = new long[numVehicles];
        for (int i = 0; i < numVehicles; i++) {
            // Cast double capacity to long for OR-Tools
            vehicleTotalCapacities[i] = (long) problem.getVehicles().get(i).getCapacity();
             if (vehicleTotalCapacities[i] < 0) {
                 logger.warn("Vehicle {} has negative total capacity ({}), setting to 0.", i, vehicleTotalCapacities[i]);
                 vehicleTotalCapacities[i] = 0;
             }
             logger.trace("Vehicle {} Total Capacity: {}", i, vehicleTotalCapacities[i]);
        }

        routing.addDimensionWithVehicleCapacity(
            totalCapacityCallbackIndex,
            0L, // No slack allowed for capacity
            vehicleTotalCapacities,
            true, // Force start cumul to zero
            TOTAL_CAPACITY_DIMENSION // Use constant name
        );
        logger.info("Added Total Capacity dimension.");


        // 2. Wheelchair Capacity Dimension
        final int wheelchairCapacityCallbackIndex = routing.registerUnaryTransitCallback((long fromIndex) -> {
            int fromNode = manager.indexToNode(fromIndex);
            PickupDeliveryTask task = problem.getTasksByIndex().get(fromNode);

             if (task == null) {
                 logger.warn("Wheelchair Capacity Callback: Task not found for node index {}", fromNode);
                 return 0;
            }

            String taskType = task.getType();
            RideRequest ride = task.getRide();

            // Only consider wheelchair rides for this dimension
            if (ride != null && ride.isWheelchairRequired()) {
                if (TASK_TYPE_PICKUP.equals(taskType)) {
                    return 1; // +1 wheelchair demand for pickup
                } else if (TASK_TYPE_DELIVERY.equals(taskType)) {
                    return -1; // -1 wheelchair demand for delivery
                }
            }
            // Non-wheelchair rides or depots have 0 wheelchair demand change
            return 0;
        });

        long[] vehicleWheelchairCapacities = new long[numVehicles];
        for (int i = 0; i < numVehicles; i++) {
             // Cast double capacity to long
            vehicleWheelchairCapacities[i] = (long) problem.getVehicles().get(i).getWheelchairCapacity();
             if (vehicleWheelchairCapacities[i] < 0) {
                 logger.warn("Vehicle {} has negative wheelchair capacity ({}), setting to 0.", i, vehicleWheelchairCapacities[i]);
                 vehicleWheelchairCapacities[i] = 0;
             }
             logger.trace("Vehicle {} Wheelchair Capacity: {}", i, vehicleWheelchairCapacities[i]);
        }

        routing.addDimensionWithVehicleCapacity(
            wheelchairCapacityCallbackIndex,
            0L, // No slack allowed for capacity
            vehicleWheelchairCapacities,
            true, // Force start cumul to zero
            WHEELCHAIR_CAPACITY_DIMENSION // Use constant name
        );
        logger.info("Added Wheelchair Capacity dimension.");

        // --- Pickup and Delivery Constraints ---
        for (RideRequest ride : problem.getRideRequests()) {
            long pickupIndex = manager.nodeToIndex(ride.getPickup().getIndex());
            long deliveryIndex = manager.nodeToIndex(ride.getDelivery().getIndex());


            // Ensures pickup before delivery, same vehicle
            routing.addPickupAndDelivery(pickupIndex, deliveryIndex);

            // Explicitly ensure time ordering (redundant with P&D but safe)
            solver.addConstraint(solver.makeLessOrEqual(
                timeDimension.cumulVar(pickupIndex),
                timeDimension.cumulVar(deliveryIndex)
            ));

            // Optional: Explicit distance ordering (usually implied by time)
            // solver.addConstraint(solver.makeLessOrEqual(
            //     distanceDimension.cumulVar(pickupIndex),
            //     distanceDimension.cumulVar(deliveryIndex)
            // ));
        }

        // --- Maximum Ride Time Constraint ---
        for (RideRequest ride : problem.getRideRequests()) {
            long pickupIndex = manager.nodeToIndex(ride.getPickup().getIndex());
            long deliveryIndex = manager.nodeToIndex(ride.getDelivery().getIndex());

            // Constraint: time(delivery) <= time(pickup) + MAX_RIDE_TIME
            // Need to use solver variables for constraints
            IntVar deliveryTimeVar = timeDimension.cumulVar(deliveryIndex);
            IntVar pickupTimeVar = timeDimension.cumulVar(pickupIndex);

            logger.trace("Adding Max Ride Time constraint for Ride {}: time({}) <= time({}) + {}",
                         ride.getId(), deliveryIndex, pickupIndex, MAX_RIDE_TIME);

            solver.addConstraint(solver.makeLessOrEqual(
                deliveryTimeVar,
                solver.makeSum(pickupTimeVar, MAX_RIDE_TIME).var() // Use .var() to get the expression variable
            ));
        }

        // --- Remove Old Depot Time Window Constraint Logic ---
        // The complex boolean logic using makeIsEqualCstVar etc. is removed.
        // It's handled implicitly by setting time windows on tasks and depots.

        // --- Search Parameters ---
        RoutingSearchParameters searchParams = main.defaultRoutingSearchParameters()
            .toBuilder()
            .setFirstSolutionStrategy(FirstSolutionStrategy.Value.PATH_CHEAPEST_ARC)
            // Consider others: PARALLEL_CHEAPEST_INSERTION, SAVINGS, BEST_INSERTION
            .setLocalSearchMetaheuristic(LocalSearchMetaheuristic.Value.GUIDED_LOCAL_SEARCH)
            // Consider others: TABU_SEARCH, SIMULATED_ANNEALING
            .setTimeLimit(Duration.newBuilder().setSeconds(300).build()) // Use protobuf Duration, e.g., 30 seconds
            // .setLogSearch(true) // Enable for detailed solver logs
            .build();
        logger.info("Solver Parameters: Strategy={}, Metaheuristic={}, TimeLimit={}s",
            searchParams.getFirstSolutionStrategy(), searchParams.getLocalSearchMetaheuristic(), searchParams.getTimeLimit().getSeconds());

        // --- Solve ---
        logger.info("Starting solver...");
        Assignment assignment = routing.solveWithParameters(searchParams);
        int status = routing.status();
        logger.info("Solver finished with status: {} ({})", status, routingStatusToString(status));

        // --- Build Solution ---
        return buildSolution(problem, routing, manager, assignment, timeMatrix, status); // Pass status
    }

    // --- Build Solution Helper ---
    private static Solution buildSolution(
        Problem problem,
        RoutingModel routing,
        RoutingIndexManager manager,
        Assignment assignment,
        long[][] timeMatrix,
        int solverStatus // Receive solver status
    ) {
        Solution solution = new Solution();

        // Handle cases where no solution is found
        if (assignment == null || solverStatus == RoutingModel.ROUTING_FAIL || solverStatus == RoutingModel.ROUTING_FAIL_TIMEOUT) {
            logger.warn("No feasible solution found or solver timed out (Status: {}). Returning empty solution with dropped rides.", solverStatus);
            solution.setErrorMessage("Solver failed to find a feasible solution. Status: " + routingStatusToString(solverStatus));
            // Still try to populate dropped rides
            populateDroppedRides(problem, routing, manager, assignment, solution); // assignment might be null here
            return solution;
        }
         if (solverStatus == RoutingModel.ROUTING_INVALID) {
             logger.error("Invalid routing model setup (Status: {}).", solverStatus);
             throw new IllegalArgumentException("Invalid model setup. Status: " + routingStatusToString(solverStatus));
         }

        // Populate Dropped Rides
        populateDroppedRides(problem, routing, manager, assignment, solution);
        logger.info("Number of dropped rides: {}", solution.getDroppedRides().size());
        if (!solution.getDroppedRides().isEmpty()) {
            logger.warn("Dropped Ride IDs: {}", solution.getDroppedRides());
        }


        // Build Routes
        var timeDimension = routing.getMutableDimension("time");
        var distanceDimension = routing.getMutableDimension("distance");
        // var totalCapacityDimension = routing.getMutableDimension(TOTAL_CAPACITY_DIMENSION); // For debug
        // var wheelchairCapacityDimension = routing.getMutableDimension(WHEELCHAIR_CAPACITY_DIMENSION); // For debug

        logger.info("Building routes for {} vehicles...", problem.getVehicles().size());
        for (int vehicleIndex = 0; vehicleIndex < problem.getVehicles().size(); vehicleIndex++) {
            Route route = new Route();
            Vehicle problemVehicle = problem.getVehicles().get(vehicleIndex);
            route.setVehicleId(problemVehicle.getId());

            long routeDistance = 0;
            long routeStartTimeSeconds = -1;
            long routeEndTimeSeconds = -1;
            int position = 0;
            long currentIndex = routing.start(vehicleIndex);
            long endDepotIndex = routing.end(vehicleIndex);

            // Check if route is used at all (next node is not the end depot immediately)
            if (assignment.value(routing.nextVar(currentIndex)) == endDepotIndex) {
                logger.debug("Vehicle {} [ID: {}] route is empty, skipping.", vehicleIndex, problemVehicle.getId());
                continue; // Skip empty routes
            }

            logger.debug("Building route for Vehicle {} [ID: {}]", vehicleIndex, problemVehicle.getId());

            while (currentIndex != endDepotIndex) { // Loop until we are AT the end depot index
                long nextIndex = assignment.value(routing.nextVar(currentIndex));
                int currentNodeIndex = manager.indexToNode(currentIndex);
                int nextNodeIndex = manager.indexToNode(nextIndex); // Needed for travel time calculation

                PickupDeliveryTask taskNode = problem.getTasksByIndex().get(currentNodeIndex);
                if (taskNode == null) {
                    logger.error("FATAL: TaskNode not found for node index {} in route for vehicle {}", currentNodeIndex, vehicleIndex);
                    // This indicates a serious issue in model setup, maybe break or throw
                    break; // Avoid further errors on this route
                }

                RideRequest ride = taskNode.getRide(); // Null for depots
                Visit visit = new Visit();
                visit.setPosition(position);
                visit.setRideId(ride != null ? ride.getId() : null);
                visit.setUserId(ride != null ? ride.getUserId() : null);
                visit.setRideDirection(ride != null ? ride.getDirection() : null); // Set direction from RideRequest
                visit.setAddress(taskNode.getAddress());
                visit.setCoordinates(taskNode.getCoordinates());
                visit.setType(taskNode.getType()); // Set type from TaskNode
                visit.setStopId(taskNode.getStopId());

                // Arrival Time (use min for safety)
                long arrivalTimeSeconds = assignment.min(timeDimension.cumulVar(currentIndex));
                visit.setArrivalTime(java.time.Duration.ofSeconds(arrivalTimeSeconds)); // Use java.time.Duration

                // Travel Time to Next Visit
                long travelSeconds = 0;
                if (nextIndex != endDepotIndex) { // Don't calculate travel time *from* the end depot
                     // Bounds check timeMatrix
                     if (currentNodeIndex >= 0 && currentNodeIndex < timeMatrix.length && nextNodeIndex >= 0 && nextNodeIndex < timeMatrix[0].length) {
                         travelSeconds = timeMatrix[currentNodeIndex][nextNodeIndex];
                     } else {
                         logger.error("Time Matrix access out of bounds when calculating travel time: fromNode={}, toNode={}", currentNodeIndex, nextNodeIndex);
                         travelSeconds = -1; // Indicate error
                     }
                }
                visit.setTravelTimeToNextVisit(java.time.Duration.ofSeconds(travelSeconds)); // Use java.time.Duration

                // Solution Window
                visit.setSolutionWindow(new TimeWindow(
                    java.time.Duration.ofSeconds(assignment.min(timeDimension.cumulVar(currentIndex))),
                    java.time.Duration.ofSeconds(assignment.max(timeDimension.cumulVar(currentIndex)))
                ));

                // Debug log for visit details
                logger.trace("  Visit {}: Node {}, TaskType {}, RideID {}, Arrival: {}, TravelToNext: {}",
                             position, currentNodeIndex, visit.getType(), visit.getRideId(),
                             visit.getArrivalTime(), visit.getTravelTimeToNextVisit());

                route.getVisits().add(visit);

                // Track route start/end times
                if (position == 0) { // First node is Start Depot
                    routeStartTimeSeconds = arrivalTimeSeconds;
                }
                // End time will be set when we reach the end depot node

                position++;
                currentIndex = nextIndex; // Move to the next node
            }

            // --- Process the End Depot Node ---
            // At this point, currentIndex == endDepotIndex
             int endNodeMapIndex = manager.indexToNode(currentIndex);
             PickupDeliveryTask endTaskNode = problem.getTasksByIndex().get(endNodeMapIndex);
             if (endTaskNode != null) {
                 Visit lastVisit = new Visit();
                 lastVisit.setPosition(position);
                 lastVisit.setRideId(null);
                 lastVisit.setUserId(null);
                 lastVisit.setRideDirection(null);
                 lastVisit.setAddress(endTaskNode.getAddress());
                 lastVisit.setCoordinates(endTaskNode.getCoordinates());
                 lastVisit.setType(endTaskNode.getType());
                 lastVisit.setStopId(endTaskNode.getStopId());

                 // Arrival time at the end depot
                 routeEndTimeSeconds = assignment.min(timeDimension.cumulVar(currentIndex));
                 lastVisit.setArrivalTime(java.time.Duration.ofSeconds(routeEndTimeSeconds));
                 lastVisit.setTravelTimeToNextVisit(java.time.Duration.ZERO); // No travel from last node
                 lastVisit.setSolutionWindow(new TimeWindow(
                     java.time.Duration.ofSeconds(assignment.min(timeDimension.cumulVar(currentIndex))),
                     java.time.Duration.ofSeconds(assignment.max(timeDimension.cumulVar(currentIndex)))
                 ));

                 logger.trace("  Visit {}: Node {}, TaskType {}, Arrival: {}",
                              position, endNodeMapIndex, lastVisit.getType(), lastVisit.getArrivalTime());

                 route.getVisits().add(lastVisit);
             } else {
                 logger.error("FATAL: End Depot TaskNode not found for node index {}", endNodeMapIndex);
                 // Try to get end time anyway if possible
                 routeEndTimeSeconds = assignment.min(timeDimension.cumulVar(currentIndex));
             }


            // Calculate Route Duration and Distance
            if (routeStartTimeSeconds != -1 && routeEndTimeSeconds != -1) {
                 route.setDuration(java.time.Duration.ofSeconds(routeEndTimeSeconds - routeStartTimeSeconds));
            } else {
                 logger.warn("Could not determine route duration accurately for vehicle {}.", vehicleIndex);
                 route.setDuration(java.time.Duration.ZERO);
            }

            // Get total distance from the distance dimension cumulVar at the *end* node
            routeDistance = assignment.value(distanceDimension.cumulVar(currentIndex)); // Use end depot index
            route.setDistance(Utils.convertDistanceBack(routeDistance)); // Use utility

             // Set the route's overall time window based on vehicle's depot times
             route.setTimeWindow(new TimeWindow(
                 java.time.Duration.ofSeconds(problemVehicle.getDepotStart().getTimeWindow().startSeconds()),
                 java.time.Duration.ofSeconds(problemVehicle.getDepotEnd().getTimeWindow().endSeconds())
             ));


            logger.debug("Route for Vehicle {} finished. Visits: {}, Duration: {}, Distance: {} km",
                         vehicleIndex, route.getVisits().size(), route.getDuration(), route.getDistance());

            // Only add routes with actual tasks (more than just start/end depots)
            if (route.getVisits().size() > 2) {
                solution.getRoutes().add(route);
            } else {
                logger.debug("Route for Vehicle {} only had depot visits, not adding to solution.", vehicleIndex);
            }
        }
        logger.info("Finished building {} non-empty routes.", solution.getRoutes().size());

        // Optional: Calculate total solution metrics if needed
        // solution.setTotalCost(assignment.objectiveValue());

        return solution;
    }


    // --- Helper to Populate Dropped Rides ---
    private static void populateDroppedRides(
        Problem problem,
        RoutingModel routing,
        RoutingIndexManager manager,
        Assignment assignment, // Can be null if solver fails early
        Solution solution)
    {
        solution.getDroppedRides().clear(); // Start fresh
        int firstTaskNodeIndex = problem.getFirstTaskNodeIndex();

        for (int nodeIdx = firstTaskNodeIndex; nodeIdx < problem.getNumberOfNodes(); ++nodeIdx) {
            PickupDeliveryTask task = problem.getTasksByIndex().get(nodeIdx);
            // Only consider PICKUP tasks for identifying dropped rides
            if (task == null || !TASK_TYPE_PICKUP.equals(task.getType())) {
                continue;
            }

            RideRequest ride = task.getRide();
            if (ride == null) continue; // Should have a ride if it's a pickup task

            long index = manager.nodeToIndex(nodeIdx);
            boolean dropped = true; // Assume dropped unless proven otherwise

            if (assignment != null) {
                 // Check if the node is served by any vehicle
                 // vehicleVar(index) returns -1 if the node is unperformed (dropped)
                 if (assignment.value(routing.vehicleVar(index)) != -1) {
                     dropped = false;
                 }
                 // Alternative/Sanity check: If nextVar points to itself, it's dropped (for non-depots)
                 // if (assignment.value(routing.nextVar(index)) == index && !routing.isStart(index) && !routing.isEnd(index)) {
                 //     dropped = true; // Overrides vehicleVar check if needed
                 // }
            } else {
                // If assignment is null (solver failed), all tasks are considered dropped
                dropped = true;
            }


            if (dropped) {
                // Add ride ID only once
                if (!solution.getDroppedRides().contains(ride.getId())) {
                    solution.getDroppedRides().add(ride.getId());
                    logger.trace("Identified dropped ride: {}", ride.getId());
                }
            }
        }
    }

        // Helper method to convert status code to string
    private static String routingStatusToString(int status) {
        if (status == RoutingModel.ROUTING_NOT_SOLVED) {
            return "ROUTING_NOT_SOLVED";
        } else if (status == RoutingModel.ROUTING_SUCCESS) {
            return "ROUTING_SUCCESS";
        } else if (status == RoutingModel.ROUTING_FAIL) {
            return "ROUTING_FAIL";
        } else if (status == RoutingModel.ROUTING_FAIL_TIMEOUT) {
            return "ROUTING_FAIL_TIMEOUT";
        } else if (status == RoutingModel.ROUTING_INVALID) {
            return "ROUTING_INVALID";
        } else {
            return "UNKNOWN_STATUS_" + status;
        }
    }
}