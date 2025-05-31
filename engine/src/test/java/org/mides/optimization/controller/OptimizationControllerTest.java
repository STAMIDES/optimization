package org.mides.optimization.controller;

import com.fasterxml.jackson.databind.ObjectMapper;
import org.junit.jupiter.api.Test;
import org.mides.optimization.model.Problem;
import org.mides.optimization.model.Vehicle;
import org.mides.optimization.model.RideRequest;
import org.mides.optimization.model.Depot;
import org.mides.optimization.model.TimeWindow;
import org.mides.optimization.model.*;
import org.mides.optimization.model.osrm.OSRMMatrixResult;
import org.mides.optimization.model.osrm.OSRMRoute;
import org.mides.optimization.model.osrm.OSRMRouteResult;
import org.mides.optimization.service.IOSRMService;
import org.mides.optimization.service.IORToolsService;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.boot.test.autoconfigure.web.servlet.AutoConfigureMockMvc;
import org.springframework.boot.test.context.SpringBootTest;
import org.springframework.boot.test.mock.mockito.MockBean;
import org.springframework.http.MediaType;
import org.springframework.test.web.servlet.MockMvc;
import org.springframework.test.web.servlet.request.MockMvcRequestBuilders;

import java.time.Duration;
import static org.springframework.test.web.servlet.result.MockMvcResultMatchers.*;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.Mockito.when;
import java.time.LocalTime;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

@SpringBootTest
@AutoConfigureMockMvc
public class OptimizationControllerTest {

    @Autowired
    private MockMvc mockMvc;

    @Autowired
    private ObjectMapper objectMapper;

    @MockBean
    private IOSRMService osrmService;

    @MockBean
    private IORToolsService orToolsService;

    @Test
    void contextLoads() {
    }

    @Test
    void solve_normalCase_shouldReturnSolution() throws Exception {
        // Arrange
        int numVehicles = 3;
        int numRideRequests = 5; // Reduced for simplicity in manual mock solution
        Problem problem = createProblem(numVehicles, numRideRequests);

        // Mock OSRMMatrixResult
        OSRMMatrixResult mockMatrixResult = new OSRMMatrixResult();
        int numNodes = 1 + (numRideRequests * 2); // 1 depot + 2 nodes per request (pickup/delivery)
        List<List<Double>> distances = new ArrayList<>();
        List<List<Double>> durations = new ArrayList<>();
        for (int i = 0; i < numNodes; i++) {
            List<Double> distRow = new ArrayList<>();
            List<Double> durRow = new ArrayList<>();
            for (int j = 0; j < numNodes; j++) {
                distRow.add(i == j ? 0.0 : 100.0 + (i*10) + j); // Placeholder distances
                durRow.add(i == j ? 0.0 : 60.0 + (i*5) + j);  // Placeholder durations
            }
            distances.add(distRow);
            durations.add(durRow);
        }
        mockMatrixResult.setDistances(distances);
        mockMatrixResult.setDurations(durations);
        when(osrmService.queryMatrix(any(List.class))).thenReturn(mockMatrixResult);

        // Mock Solution
        Solution mockSolution = new Solution();
        List<Route> routes = new ArrayList<>();
        if (numVehicles > 0 && numRideRequests > 0) {
            Route route1 = new Route();
            route1.setVehicleId(problem.getVehicles().get(0).getId());
            List<Visit> visits = new ArrayList<>();
            // Simplified: assign first request to first vehicle
            RideRequest firstRequest = problem.getRideRequests().get(0);

            Visit pickupVisit = new Visit();
            pickupVisit.setPosition(0);
            pickupVisit.setRideId(firstRequest.getId());
            pickupVisit.setUserId(firstRequest.getUserId());
            pickupVisit.setAddress(firstRequest.getPickup().getAddress());
            pickupVisit.setCoordinates(firstRequest.getPickup().getCoordinates());
            pickupVisit.setArrivalTime(firstRequest.getPickup().getTimeWindow().getStartTime());
            pickupVisit.setTravelTimeToNextVisit(Duration.ofMinutes(10));
            pickupVisit.setSolutionWindow(firstRequest.getPickup().getTimeWindow());
            pickupVisit.setType("Pickup"); // Assuming type string
            pickupVisit.setStopId(firstRequest.getPickup().getId()); // Using task ID as stopID
            visits.add(pickupVisit);

            Visit deliveryVisit = new Visit();
            deliveryVisit.setPosition(1);
            deliveryVisit.setRideId(firstRequest.getId());
            deliveryVisit.setUserId(firstRequest.getUserId());
            deliveryVisit.setAddress(firstRequest.getDelivery().getAddress());
            deliveryVisit.setCoordinates(firstRequest.getDelivery().getCoordinates());
            deliveryVisit.setArrivalTime(firstRequest.getPickup().getTimeWindow().getStartTime().plusMinutes(20)); // Simplified arrival
            deliveryVisit.setTravelTimeToNextVisit(Duration.ZERO);
            deliveryVisit.setSolutionWindow(firstRequest.getDelivery().getTimeWindow());
            deliveryVisit.setType("Delivery"); // Assuming type string
            deliveryVisit.setStopId(firstRequest.getDelivery().getId()); // Using task ID as stopID
            visits.add(deliveryVisit);

            route1.setVisits(visits);
            route1.setDuration(Duration.ofMinutes(30));
            route1.setDistance(2000.0); // meters
            routes.add(route1);
        }
        mockSolution.setRoutes(routes);
        List<String> droppedRides = new ArrayList<>();
        if (numRideRequests > 1) {
            droppedRides.add(problem.getRideRequests().get(1).getId()); // Drop the second ride
        }
        mockSolution.setDroppedRides(droppedRides);

        // Mock OR-Tools solve call
        // Note: The controller converts double[][] from OSRM to long[][] for OR-Tools
        when(orToolsService.solve(any(Problem.class), any(long[][].class), any(long[][].class)))
            .thenReturn(mockSolution);

        // Mock OSRMRouteResult for geometry
        OSRMRoute osrmRoute = new OSRMRoute();
        osrmRoute.setGeometry("test_geometry_polyline"); // This will be decoded by OSRMRoute.decodeGeometry()
        OSRMRouteResult routeResult = new OSRMRouteResult();
        routeResult.setRoutes(List.of(osrmRoute));
        when(osrmService.queryRoute(any(List.class))).thenReturn(routeResult);

        // Act & Assert
        org.springframework.test.web.servlet.ResultActions resultActions = mockMvc.perform(MockMvcRequestBuilders.post("/optimization/v1/solve")
                .contentType(MediaType.APPLICATION_JSON)
                .content(objectMapper.writeValueAsString(problem)))
                .andExpect(status().isOk())
                .andExpect(content().contentType(MediaType.APPLICATION_JSON))
                .andExpect(jsonPath("$.routes").isArray())
                .andExpect(jsonPath("$.droppedRides").isArray());

        if (!mockSolution.getRoutes().isEmpty()) {
            resultActions
                .andExpect(jsonPath("$.routes[0].vehicleId").value(mockSolution.getRoutes().get(0).getVehicleId()))
                .andExpect(jsonPath("$.routes[0].geometry").exists())
                .andExpect(jsonPath("$.routes[0].geometry").isArray()); // Decoded geometry is List<List<Double>>
        } else {
            resultActions.andExpect(jsonPath("$.routes").isEmpty());
        }

        if (!mockSolution.getDroppedRides().isEmpty()) {
            resultActions.andExpect(jsonPath("$.droppedRides[0]").value(mockSolution.getDroppedRides().get(0)));
        } else {
            resultActions.andExpect(jsonPath("$.droppedRides").isEmpty());
        }
    }

    @Test
    void solve_moreRideRequests_shouldReturnSolution() throws Exception {
        // Arrange
        int numVehicles = 3;
        int numRideRequests = 50;
        Problem problem = createProblem(numVehicles, numRideRequests);

        // Mock OSRMMatrixResult
        OSRMMatrixResult mockMatrixResult = new OSRMMatrixResult();
        int numMatrixNodes = 1 + (numRideRequests * 2); // 1 depot + 2 nodes per request
        List<List<Double>> distances = new ArrayList<>();
        List<List<Double>> durations = new ArrayList<>();
        for (int i = 0; i < numMatrixNodes; i++) {
            List<Double> distRow = new ArrayList<>();
            List<Double> durRow = new ArrayList<>();
            for (int j = 0; j < numMatrixNodes; j++) {
                distRow.add(i == j ? 0.0 : 120.0 + (i*10) + j*2); // Different placeholder distances
                durRow.add(i == j ? 0.0 : 80.0 + (i*6) + j*1.5);  // Different placeholder durations
            }
            distances.add(distRow);
            durations.add(durRow);
        }
        mockMatrixResult.setDistances(distances);
        mockMatrixResult.setDurations(durations);
        when(osrmService.queryMatrix(any(List.class))).thenReturn(mockMatrixResult);

        // Mock Solution
        Solution mockSolution = new Solution();
        List<Route> routes = new ArrayList<>();
        // Assign some requests to the available vehicles
        for(int v = 0; v < numVehicles; v++) {
            if (problem.getVehicles().size() <= v || problem.getRideRequests().size() <= v) break;

            Route route = new Route();
            route.setVehicleId(problem.getVehicles().get(v).getId());
            RideRequest request = problem.getRideRequests().get(v); // Assign v-th request to v-th vehicle

            List<Visit> visits = new ArrayList<>();
            visits.add(new Visit(0, request.getId(), request.getUserId(), request.getPickup().getAddress(), request.getPickup().getCoordinates(), request.getPickup().getTimeWindow().getStartTime(), Duration.ofMinutes(10), request.getPickup().getTimeWindow(), "Pickup", request.getPickup().getId(),0,0,0,0,0,0,0,0,0,0,0,false, null, null,0,0));
            visits.add(new Visit(1, request.getId(), request.getUserId(), request.getDelivery().getAddress(), request.getDelivery().getCoordinates(), request.getPickup().getTimeWindow().getStartTime().plusMinutes(25), Duration.ZERO, request.getDelivery().getTimeWindow(), "Delivery", request.getDelivery().getId(),0,0,0,0,0,0,0,0,0,0,0,false, null, null,0,0));
            route.setVisits(visits);
            route.setDuration(Duration.ofMinutes(35 + v*5)); // Vary duration slightly
            route.setDistance(2500.0 + v*100); // Vary distance slightly
            routes.add(route);
        }
        mockSolution.setRoutes(routes);

        List<String> droppedRides = new ArrayList<>();
        // Expect many dropped rides
        int expectedDroppedCount = 0;
        for (int i = numVehicles; i < numRideRequests; i++) {
            if (problem.getRideRequests().size() > i) {
                droppedRides.add(problem.getRideRequests().get(i).getId());
                expectedDroppedCount++;
            }
        }
        mockSolution.setDroppedRides(droppedRides);

        when(orToolsService.solve(any(Problem.class), any(long[][].class), any(long[][].class)))
            .thenReturn(mockSolution);

        OSRMRoute osrmRoute = new OSRMRoute();
        osrmRoute.setGeometry("yet_another_geometry_polyline");
        OSRMRouteResult routeResult = new OSRMRouteResult();
        routeResult.setRoutes(List.of(osrmRoute));
        when(osrmService.queryRoute(any(List.class))).thenReturn(routeResult);

        // Act & Assert
        org.springframework.test.web.servlet.ResultActions resultActions = mockMvc.perform(MockMvcRequestBuilders.post("/optimization/v1/solve")
                .contentType(MediaType.APPLICATION_JSON)
                .content(objectMapper.writeValueAsString(problem)))
                .andExpect(status().isOk())
                .andExpect(content().contentType(MediaType.APPLICATION_JSON))
                .andExpect(jsonPath("$.routes").isArray())
                .andExpect(jsonPath("$.droppedRides").isArray());

        if (!mockSolution.getRoutes().isEmpty()) {
            resultActions
                .andExpect(jsonPath("$.routes[0].vehicleId").value(mockSolution.getRoutes().get(0).getVehicleId()))
                .andExpect(jsonPath("$.routes[0].geometry").exists())
                .andExpect(jsonPath("$.routes[0].geometry").isArray());
        } else {
            resultActions.andExpect(jsonPath("$.routes").isEmpty());
        }

        resultActions.andExpect(jsonPath("$.droppedRides.length()").value(expectedDroppedCount));
        if (expectedDroppedCount > 0) {
            resultActions.andExpect(jsonPath("$.droppedRides[0]").value(mockSolution.getDroppedRides().get(0)));
        }
    }

    @Test
    void solve_moreVehicles_shouldReturnSolution() throws Exception {
        // Arrange
        int numVehicles = 10;
        int numRideRequests = 20;
        Problem problem = createProblem(numVehicles, numRideRequests);

        // Mock OSRMMatrixResult
        OSRMMatrixResult mockMatrixResult = new OSRMMatrixResult();
        // Number of nodes = 1 depot + 2 nodes per ride request (pickup/delivery)
        // The actual list of coordinates passed to OSRM also includes vehicle start/end depots if different,
        // but for simplicity, our current problem generator uses one main depot for everything.
        // The controller builds the matrix based on all unique locations.
        // For this mock, we'll assume locations are: 1 main_depot + 2*numRideRequests unique task locations.
        // If vehicles had unique start/end depots, those would also be part of the matrix.
        int numMatrixNodes = 1 + (numRideRequests * 2) ;
        List<List<Double>> distances = new ArrayList<>();
        List<List<Double>> durations = new ArrayList<>();
        for (int i = 0; i < numMatrixNodes; i++) {
            List<Double> distRow = new ArrayList<>();
            List<Double> durRow = new ArrayList<>();
            for (int j = 0; j < numMatrixNodes; j++) {
                distRow.add(i == j ? 0.0 : 150.0 + (i*12) + j); // Slightly different placeholder distances
                durRow.add(i == j ? 0.0 : 70.0 + (i*7) + j);  // Slightly different placeholder durations
            }
            distances.add(distRow);
            durations.add(durRow);
        }
        mockMatrixResult.setDistances(distances);
        mockMatrixResult.setDurations(durations);
        when(osrmService.queryMatrix(any(List.class))).thenReturn(mockMatrixResult);

        // Mock Solution - more extensive if needed, but for this test, a simple one is fine
        Solution mockSolution = new Solution();
        List<Route> routes = new ArrayList<>();
        // Create a couple of routes for the first two vehicles if they exist
        if (numVehicles > 0 && numRideRequests > 0) {
            Route route1 = new Route();
            route1.setVehicleId(problem.getVehicles().get(0).getId()); // Vehicle 0
            // Add some visits if ride requests exist
            if (!problem.getRideRequests().isEmpty()) {
                RideRequest request1 = problem.getRideRequests().get(0);
                List<Visit> visits1 = new ArrayList<>();
                visits1.add(new Visit(0, request1.getId(), request1.getUserId(), request1.getPickup().getAddress(), request1.getPickup().getCoordinates(), request1.getPickup().getTimeWindow().getStartTime(), Duration.ofMinutes(10), request1.getPickup().getTimeWindow(), "Pickup", request1.getPickup().getId(),0,0,0,0,0,0,0,0,0,0,0,false, null, null,0,0));
                visits1.add(new Visit(1, request1.getId(), request1.getUserId(), request1.getDelivery().getAddress(), request1.getDelivery().getCoordinates(), request1.getPickup().getTimeWindow().getStartTime().plusMinutes(25), Duration.ZERO, request1.getDelivery().getTimeWindow(), "Delivery", request1.getDelivery().getId(),0,0,0,0,0,0,0,0,0,0,0,false, null, null,0,0));
                route1.setVisits(visits1);
                route1.setDuration(Duration.ofMinutes(40));
                route1.setDistance(3000.0);
                routes.add(route1);
            }
        }
        if (numVehicles > 1 && numRideRequests > 1) {
             Route route2 = new Route();
            route2.setVehicleId(problem.getVehicles().get(1).getId()); // Vehicle 1
            if (problem.getRideRequests().size() > 1) {
                RideRequest request2 = problem.getRideRequests().get(1);
                 List<Visit> visits2 = new ArrayList<>();
                visits2.add(new Visit(0, request2.getId(), request2.getUserId(), request2.getPickup().getAddress(), request2.getPickup().getCoordinates(), request2.getPickup().getTimeWindow().getStartTime(), Duration.ofMinutes(12), request2.getPickup().getTimeWindow(), "Pickup", request2.getPickup().getId(),0,0,0,0,0,0,0,0,0,0,0,false, null, null,0,0));
                visits2.add(new Visit(1, request2.getId(), request2.getUserId(), request2.getDelivery().getAddress(), request2.getDelivery().getCoordinates(), request2.getPickup().getTimeWindow().getStartTime().plusMinutes(30), Duration.ZERO, request2.getDelivery().getTimeWindow(), "Delivery", request2.getDelivery().getId(),0,0,0,0,0,0,0,0,0,0,0,false, null, null,0,0));
                route2.setVisits(visits2);
                route2.setDuration(Duration.ofMinutes(45));
                route2.setDistance(3500.0);
                routes.add(route2);
            }
        }
        mockSolution.setRoutes(routes);

        List<String> droppedRides = new ArrayList<>();
        // Drop a few ride requests if they exist
        if (numRideRequests > 2) droppedRides.add(problem.getRideRequests().get(2).getId());
        if (numRideRequests > 3) droppedRides.add(problem.getRideRequests().get(3).getId());
        mockSolution.setDroppedRides(droppedRides);

        when(orToolsService.solve(any(Problem.class), any(long[][].class), any(long[][].class)))
            .thenReturn(mockSolution);

        OSRMRoute osrmRoute = new OSRMRoute();
        osrmRoute.setGeometry("another_test_geometry_polyline");
        OSRMRouteResult routeResult = new OSRMRouteResult();
        routeResult.setRoutes(List.of(osrmRoute));
        when(osrmService.queryRoute(any(List.class))).thenReturn(routeResult);

        // Act & Assert
        org.springframework.test.web.servlet.ResultActions resultActions = mockMvc.perform(MockMvcRequestBuilders.post("/optimization/v1/solve")
                .contentType(MediaType.APPLICATION_JSON)
                .content(objectMapper.writeValueAsString(problem)))
                .andExpect(status().isOk())
                .andExpect(content().contentType(MediaType.APPLICATION_JSON))
                .andExpect(jsonPath("$.routes").isArray())
                .andExpect(jsonPath("$.droppedRides").isArray());

        if (!mockSolution.getRoutes().isEmpty()) {
            resultActions
                .andExpect(jsonPath("$.routes[0].vehicleId").value(mockSolution.getRoutes().get(0).getVehicleId()))
                .andExpect(jsonPath("$.routes[0].geometry").exists())
                .andExpect(jsonPath("$.routes[0].geometry").isArray());
            if (mockSolution.getRoutes().size() > 1) {
                 resultActions.andExpect(jsonPath("$.routes[1].vehicleId").value(mockSolution.getRoutes().get(1).getVehicleId()));
            }
        } else {
            resultActions.andExpect(jsonPath("$.routes").isEmpty());
        }

        if (!mockSolution.getDroppedRides().isEmpty()) {
            resultActions.andExpect(jsonPath("$.droppedRides[0]").value(mockSolution.getDroppedRides().get(0)));
        } else {
            resultActions.andExpect(jsonPath("$.droppedRides").isEmpty());
        }
    }

    @Test
    void solve_noSolution_shouldReturnEmptyRoutesOrAllDropped() throws Exception {
        // Arrange
        int numVehicles = 2;
        int numRideRequests = 5;
        Problem problem = createProblem(numVehicles, numRideRequests);

        // Mock OSRMMatrixResult - still needed as matrix query happens before solve
        OSRMMatrixResult mockMatrixResult = new OSRMMatrixResult();
        int numMatrixNodes = 1 + (numRideRequests * 2);
        List<List<Double>> distances = new ArrayList<>();
        List<List<Double>> durations = new ArrayList<>();
        for (int i = 0; i < numMatrixNodes; i++) {
            List<Double> distRow = new ArrayList<>();
            List<Double> durRow = new ArrayList<>();
            for (int j = 0; j < numMatrixNodes; j++) {
                distRow.add(i == j ? 0.0 : 200.0);
                durRow.add(i == j ? 0.0 : 100.0);
            }
            distances.add(distRow);
            durations.add(durRow);
        }
        mockMatrixResult.setDistances(distances);
        mockMatrixResult.setDurations(durations);
        when(osrmService.queryMatrix(any(List.class))).thenReturn(mockMatrixResult);

        // Mock Solution - No routes, all rides dropped
        Solution mockNoSolution = new Solution();
        mockNoSolution.setRoutes(new ArrayList<>()); // Empty list of routes
        List<String> droppedRides = new ArrayList<>();
        for (RideRequest request : problem.getRideRequests()) {
            droppedRides.add(request.getId());
        }
        mockNoSolution.setDroppedRides(droppedRides);

        when(orToolsService.solve(any(Problem.class), any(long[][].class), any(long[][].class)))
            .thenReturn(mockNoSolution);

        // Mock OSRMRouteResult for geometry - controller might not even call this if no routes
        // Providing a default empty result is safest.
        OSRMRouteResult emptyRouteResult = new OSRMRouteResult();
        emptyRouteResult.setRoutes(new ArrayList<>()); // Ensure getRoutes() is not null
        when(osrmService.queryRoute(any(List.class))).thenReturn(emptyRouteResult);


        // Act & Assert
        mockMvc.perform(MockMvcRequestBuilders.post("/optimization/v1/solve")
                .contentType(MediaType.APPLICATION_JSON)
                .content(objectMapper.writeValueAsString(problem)))
                .andExpect(status().isOk())
                .andExpect(content().contentType(MediaType.APPLICATION_JSON))
                .andExpect(jsonPath("$.routes").isArray())
                .andExpect(jsonPath("$.routes").isEmpty()) // or .andExpect(jsonPath("$.routes.length()").value(0))
                .andExpect(jsonPath("$.droppedRides").isArray())
                .andExpect(jsonPath("$.droppedRides.length()").value(numRideRequests));
    }

    @Test
    void solve_invalidInput_shouldReturnBadRequest() throws Exception {
        // Arrange
        String invalidJsonInput = "{}"; // Assuming this will trigger validation constraints (e.g. @NotNull fields in Problem)

        // Act & Assert
        mockMvc.perform(MockMvcRequestBuilders.post("/optimization/v1/solve")
                .contentType(MediaType.APPLICATION_JSON)
                .content(invalidJsonInput))
                .andExpect(status().isBadRequest());
        // Optionally, further assert the error response structure if known
        // e.g., .andExpect(jsonPath("$.errors").isArray())
        //       .andExpect(jsonPath("$.errors[0].field").exists())
        //       .andExpect(jsonPath("$.errors[0].message").exists());
    }


    private static Problem createProblem(int numVehicles, int numRideRequests) {
        // TODO: More sophisticated depot generation if needed (e.g. multiple depots)
        Depot mainDepot = generateDepot("main");
        List<Vehicle> vehicles = generateVehicles(numVehicles, mainDepot); // Vehicles will use this as their start/end depot
        List<RideRequest> rideRequests = generateRideRequests(numRideRequests, mainDepot); // Pass mainDepot, though currently unused directly by tasks
        return new Problem(vehicles, rideRequests, mainDepot);
    }

    private static List<Vehicle> generateVehicles(int numVehicles, Depot commonDepot) {
        List<Vehicle> vehicles = new ArrayList<>();
        Random random = new Random();
        for (int i = 0; i < numVehicles; i++) {
            String id = "vehicle_" + i;
            int seatCapacity = 4 + random.nextInt(5); // 4-8 seats
            int wheelchairCapacity = random.nextInt(3); // 0-2 wheelchair spots
            // TODO: Generate meaningful characteristics if needed
            List<String> supportedCharacteristics = new ArrayList<>();
            TimeWindow timeWindow = generateRandomTimeWindow(
                    LocalTime.of(7, 0), LocalTime.of(9,0), // Start between 7-9 AM
                    Duration.ofHours(8), Duration.ofHours(12) // Work duration 8-12 hours
            );
            boolean withRest = random.nextBoolean();

            vehicles.add(new Vehicle(id, seatCapacity, wheelchairCapacity, supportedCharacteristics,
                                   commonDepot, commonDepot, timeWindow, withRest, null, null, null, null, 0,0,0,0, false));
        }
        return vehicles;
    }

    private static List<RideRequest> generateRideRequests(int numRideRequests, Depot mainDepot) {
        List<RideRequest> rideRequests = new ArrayList<>();
        Random random = new Random();
        String[] directions = {"going", "return"};

        for (int i = 0; i < numRideRequests; i++) {
            String rideId = "ride_" + i;
            // TODO: Use Faker for userIds if added
            String userId = "user_" + random.nextInt(10000);
            boolean wheelchairRequired = random.nextBoolean();
            // TODO: Generate meaningful characteristics if needed
            List<String> characteristics = new ArrayList<>();

            PickupDeliveryTask pickupTask = generatePickupTask(rideId, "task" + i);
            PickupDeliveryTask deliveryTask = generateDeliveryTask(rideId, "task" + i, pickupTask);

            String direction = directions[random.nextInt(directions.length)];

            // TODO: Consider how maxWaitTime, maxDelay, and timeMultiplier are best generated or if they should be fixed for tests.
            rideRequests.add(new RideRequest(rideId, userId, wheelchairRequired, characteristics,
                                           pickupTask, deliveryTask, direction,
                                           Duration.ofMinutes(10), Duration.ofMinutes(15), 1.5, null, 0,0,0,0, false));
        }
        return rideRequests;
    }

    private static Depot generateDepot(String idPrefix) {
        Random random = new Random();
        String id = idPrefix + "_depot_" + random.nextInt(1000);
        // TODO: Use Faker for addresses if added
        String address = "Depot Address " + random.nextInt(100);
        Coordinate coordinates = generateMontevideoCoordinate();
        return new Depot(id, address, coordinates);
    }

    private static PickupDeliveryTask generatePickupTask(String rideId, String stopIdPrefix) {
        Random random = new Random();
        Coordinate coordinates = generateMontevideoCoordinate();
        // TODO: Use Faker for addresses
        String address = "Pickup Address " + random.nextInt(1000);
        String[] types = {"Particular", "Mides", "Hospital"};
        String type = types[random.nextInt(types.length)];
        // Ensure pickup time window is reasonable, e.g., between 8 AM and 6 PM, duration 15-30 mins
        TimeWindow timeWindow = generateRandomTimeWindow(
                LocalTime.of(8, 0), LocalTime.of(18, 0),
                Duration.ofMinutes(15), Duration.ofMinutes(30)
        );
        // TODO: More sophisticated ID generation if needed
        return new PickupDeliveryTask(rideId + "_pickup_" + stopIdPrefix , coordinates, timeWindow, 0, address, type, null, 0, 0,0,0,false, false);
    }

    private static PickupDeliveryTask generateDeliveryTask(String rideId, String stopIdPrefix, PickupDeliveryTask pickupTask) {
        Random random = new Random();
        Coordinate coordinates = generateMontevideoCoordinate();
        // TODO: Use Faker for addresses
        String address = "Delivery Address " + random.nextInt(1000);
        String[] types = {"Particular", "Mides", "Hospital"}; // Can be different from pickup
        String type = types[random.nextInt(types.length)];

        LocalTime pickupStartTime = (pickupTask != null && pickupTask.getTimeWindow() != null) ? pickupTask.getTimeWindow().getStartTime() : LocalTime.of(8, 0);
        LocalTime minDeliveryStartTime = pickupStartTime.plusMinutes(30); // Ensure delivery is after pickup
        LocalTime maxDeliveryStartTime = LocalTime.of(19, 0);
        if(minDeliveryStartTime.isAfter(maxDeliveryStartTime)) {
            maxDeliveryStartTime = minDeliveryStartTime.plusHours(1); // Adjust if pickup is very late
        }

        TimeWindow timeWindow = generateRandomTimeWindow(
                minDeliveryStartTime, maxDeliveryStartTime,
                Duration.ofMinutes(15), Duration.ofMinutes(30)
        );
        // TODO: More sophisticated ID generation if needed
        return new PickupDeliveryTask(rideId + "_delivery_" + stopIdPrefix, coordinates, timeWindow, 0, address, type, null, 0, 0,0,0,false, false);
    }

    private static TimeWindow generateTimeWindow(LocalTime startTime, Duration duration) {
        if (startTime == null || duration == null) {
            return null; // Or throw an exception, or return a default
        }
        LocalTime endTime = startTime.plus(duration);
        return new TimeWindow(startTime, endTime);
    }

    private static TimeWindow generateRandomTimeWindow(LocalTime minStart, LocalTime maxStart, Duration minDuration, Duration maxDuration) {
        Random random = new Random();
        if (minStart == null || maxStart == null || minDuration == null || maxDuration == null) {
            return new TimeWindow(LocalTime.of(8,0), LocalTime.of(18,0)); // Default if params are null
        }

        long minStartSeconds = minStart.toSecondOfDay();
        long maxStartSeconds = maxStart.toSecondOfDay();
        long randomStartSeconds = minStartSeconds + (long) (random.nextDouble() * (maxStartSeconds - minStartSeconds));
        LocalTime startTime = LocalTime.ofSecondOfDay(randomStartSeconds);

        long minDurationSeconds = minDuration.getSeconds();
        long maxDurationSeconds = maxDuration.getSeconds();
        long randomDurationSeconds = minDurationSeconds + (long) (random.nextDouble() * (maxDurationSeconds - minDurationSeconds));
        Duration duration = Duration.ofSeconds(randomDurationSeconds);

        return generateTimeWindow(startTime, duration);
    }

    private static Coordinate generateMontevideoCoordinate() {
        Random random = new Random();
        // Montevideo bounding box (approximate)
        // Latitude: -34.797 to -34.927
        // Longitude: -56.053 to -56.256
        double lat = -34.797 + (-34.927 - (-34.797)) * random.nextDouble();
        double lon = -56.053 + (-56.256 - (-56.053)) * random.nextDouble();
        return new Coordinate(lat, lon);
    }

    private static Depot generateDepot(String idPrefix) {
        Random random = new Random();
        String id = idPrefix + "_depot_" + random.nextInt(1000);
        // TODO: Use Faker for addresses if added
        String address = "Depot Address " + random.nextInt(100);
        Coordinate coordinates = generateMontevideoCoordinate();
        return new Depot(id, address, coordinates);
    }

}
