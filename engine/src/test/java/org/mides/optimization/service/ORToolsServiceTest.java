package org.mides.optimization.service;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.junit.jupiter.MockitoExtension; // If Mockito is used, though maybe not for first test
import org.mides.optimization.model.*;
import org.mides.optimization.util.Constants; // To potentially modify ALLOW_DEPOT_DROP if possible, or acknowledge its default

import java.time.Duration;
import java.util.ArrayList;
import java.util.List;
import java.util.Collections;

import static org.junit.jupiter.api.Assertions.*;

// @ExtendWith(MockitoExtension.class) // Uncomment if Mockito is used
class ORToolsServiceTest {

    private ORToolsService orToolsService;

    @BeforeEach
    void setUp() {
        orToolsService = new ORToolsService();
        // We are testing with ALLOW_DEPOT_DROP = true (its default in Constants.java)
        // For tests where it needs to be false, we'd need a way to set it, which is not straightforward for a static final.
        // So, tests will assume Constants.ALLOW_DEPOT_DROP is true.
    }

    // Helper method to create a basic Problem
    private Problem createSimpleProblem(Vehicle vehicle, RideRequest rideRequest) {
        Problem problem = new Problem();
        problem.getVehicles().add(vehicle);
        problem.getRideRequests().add(rideRequest);
        problem.initialize(); // This sets up indices and tasksByIndex
        return problem;
    }

    @Test
    void testDepotDropOccurs_WhenShiftIsEnding() {
        // 1. Setup: Vehicle with a short shift, Ride Request
        Depot depot1Start = new Depot();
        depot1Start.setId("D1S");
        depot1Start.setCoordinates(new Coordinate(0,0));

        Depot depot1End = new Depot();
        depot1End.setId("D1E");
        depot1End.setCoordinates(new Coordinate(0,1)); // Vehicle ends slightly away

        Vehicle vehicle1 = new Vehicle();
        vehicle1.setId("V1");
        vehicle1.setDepotStart(depot1Start);
        vehicle1.setDepotEnd(depot1End);
        vehicle1.setSeatCapacity(4);
        // Vehicle shift: 0s to 100s. Pickup is at 50s. Delivery would be at 150s (too late). Depot drop at 70s.
        vehicle1.setTimeWindow(new TimeWindow(Duration.ofSeconds(0), Duration.ofSeconds(100)));

        PickupDeliveryTask pickupTask = new PickupDeliveryTask();
        pickupTask.setCoordinates(new Coordinate(1,0)); // Pickup location
        pickupTask.setTimeWindow(new TimeWindow(Duration.ofSeconds(40), Duration.ofSeconds(60))); // Pickup window
        pickupTask.setType(TaskType.PICKUP);

        PickupDeliveryTask deliveryTask = new PickupDeliveryTask();
        deliveryTask.setCoordinates(new Coordinate(1,10)); // Original delivery (far)
        deliveryTask.setTimeWindow(new TimeWindow(Duration.ofSeconds(140), Duration.ofSeconds(160))); // Delivery window
        deliveryTask.setType(TaskType.DELIVERY);

        RideRequest ride1 = new RideRequest();
        ride1.setId("R1");
        ride1.setUserId("U1");
        ride1.setPickup(pickupTask);
        ride1.setDelivery(deliveryTask);
        pickupTask.setRide(ride1); // Set back-reference
        deliveryTask.setRide(ride1); // Set back-reference


        Problem problem = createSimpleProblem(vehicle1, ride1);

        // 2. Setup distance and time matrices
        // Nodes: V1_DepotStart(0), V1_DepotEnd(1), R1_Pickup(2), R1_Delivery(3)
        // Distances/Times:
        // D1S -> P: 50 units
        // P -> D1E (Depot End for V1): 20 units
        // P -> D_orig (Original Delivery): 100 units
        // D1S -> D_orig: 150 units
        long[][] distanceMatrix = new long[][]{
            {0, 10, 500, 1500}, // D1S to D1E, P, D_orig
            {10, 0, 200, 1000}, // D1E to D1S, P, D_orig (not typical)
            {500, 200, 0, 1000}, // P to D1S, D1E, D_orig
            {1500, 1000, 1000, 0}  // D_orig to D1S, D1E, P
        };
        // Time matrix mirrors distance for simplicity
        long[][] timeMatrix = new long[][]{
            {0, 10, 50, 150},
            {10, 0, 20, 100},
            {50, 20, 0, 100},
            {150, 100, 100, 0}
        };

        // Adjust for DISTANCE_MULTIPLIER if service uses it internally before callbacks
        // The ORToolsService uses the matrices directly in callbacks, so no multiplier needed here.

        // 3. Solve
        Solution solution = orToolsService.solve(problem, distanceMatrix, timeMatrix);

        // 4. Assertions
        assertNotNull(solution);
        assertTrue(solution.getDroppedRides().isEmpty(), "Ride should not be fully dropped.");
        assertEquals(1, solution.getDepotDroppedRides().size(), "One ride should be depot-dropped.");

        DepotDroppedRideInfo depotDropInfo = solution.getDepotDroppedRides().get(0);
        assertEquals("R1", depotDropInfo.getRideId());
        assertEquals("V1", depotDropInfo.getVehicleIdDroppedBy());
        assertEquals(depot1End.getId(), depotDropInfo.getDroppedAtDepotId()); // Check dropped at V1's end depot
        //assertEquals(Duration.ofSeconds(50), depotDropInfo.getTimeOfDropAtDepot()); // Placeholder: current time is pickup time
        // TODO: Refine assertion for timeOfDropAtDepot when it's more accurate (e.g. P_time + P_to_Depot_time)

        assertEquals(1, solution.getRoutes().size(), "Should have one route for V1.");
        Route vehicle1Route = solution.getRoutes().get(0);
        assertEquals("V1", vehicle1Route.getVehicleId());

        boolean pickupVisited = false;
        boolean originalDeliveryVisited = false;
        for (Visit visit : vehicle1Route.getVisits()) {
            if (visit.getRideId() != null && visit.getRideId().equals("R1")) {
                if (visit.getType() == TaskType.PICKUP) {
                    pickupVisited = true;
                    assertTrue(visit.isDepotDropPickup(), "Pickup visit should be marked as part of depot drop.");
                    // Check arrival time at pickup
                     assertEquals(Duration.ofSeconds(50), visit.getArrivalTime(), "Arrival at pickup should be 50s");
                }
                if (visit.getType() == TaskType.DELIVERY) {
                    originalDeliveryVisited = true; // This should not happen for R1 on V1
                }
            }
        }
        assertTrue(pickupVisited, "Pickup for R1 should be on V1's route.");
        assertFalse(originalDeliveryVisited, "Original delivery for R1 should NOT be on V1's route.");

        // Check that the vehicle's route ends at its designated depot (D1E)
        // The last visit in the route list is the vehicle's end depot.
        Visit lastVisit = vehicle1Route.getVisits().get(vehicle1Route.getVisits().size() - 1);
        assertEquals(depot1End.getId(), lastVisit.getStopId(), "Vehicle should end at its designated depot D1E.");
        // Expected arrival at D1E = PickupTime (50) + TravelTime(P -> D1E) (20) = 70
        assertEquals(Duration.ofSeconds(70), lastVisit.getArrivalTime(), "Vehicle should arrive at D1E at 70s.");

    }

    // TODO: Add testDepotDropAllowed_ButNotOptimal
    // TODO: Add testDepotDropNotAllowed_ResultsInFullDropOrCompletion (assuming ALLOW_DEPOT_DROP is true by default)
    // TODO: Add testComplexScenario_MultipleVehiclesAndRides

            @Test
            void testDepotDropAllowed_ButNotOptimal() {
                // 1. Setup: Vehicle with ample time, Ride Request
                Depot depot1Start = new Depot();
                depot1Start.setId("D1S");
                depot1Start.setCoordinates(new Coordinate(0,0));

                Depot depot1End = new Depot();
                depot1End.setId("D1E");
                depot1End.setCoordinates(new Coordinate(0,1));

                Vehicle vehicle1 = new Vehicle();
                vehicle1.setId("V1");
                vehicle1.setDepotStart(depot1Start);
                vehicle1.setDepotEnd(depot1End);
                vehicle1.setSeatCapacity(4);
                // Vehicle shift: 0s to 500s (ample time)
                vehicle1.setTimeWindow(new TimeWindow(Duration.ofSeconds(0), Duration.ofSeconds(500)));

                PickupDeliveryTask pickupTask = new PickupDeliveryTask();
                pickupTask.setCoordinates(new Coordinate(1,0)); // Pickup location
                pickupTask.setTimeWindow(new TimeWindow(Duration.ofSeconds(40), Duration.ofSeconds(60)));
                pickupTask.setType(TaskType.PICKUP);

                PickupDeliveryTask deliveryTask = new PickupDeliveryTask();
                deliveryTask.setCoordinates(new Coordinate(1,5)); // Original delivery (not too far)
                deliveryTask.setTimeWindow(new TimeWindow(Duration.ofSeconds(100), Duration.ofSeconds(120))); // Feasible delivery window
                deliveryTask.setType(TaskType.DELIVERY);

                RideRequest ride1 = new RideRequest();
                ride1.setId("R1");
                ride1.setUserId("U1");
                ride1.setPickup(pickupTask);
                ride1.setDelivery(deliveryTask);
                pickupTask.setRide(ride1);
                deliveryTask.setRide(ride1);

                Problem problem = createSimpleProblem(vehicle1, ride1);

                // 2. Setup distance and time matrices
                // Nodes: V1_DepotStart(0), V1_DepotEnd(1), R1_Pickup(2), R1_Delivery(3)
                // D1S -> P: 50
                // P -> D_orig: 50
                // P -> D1E: 100 (making depot drop less attractive than direct)
                long[][] distanceMatrix = new long[][]{
                    {0,  200, 500, 1000}, // D1S to D1E, P, D_orig
                    {200, 0,  1000, 500}, // D1E
                    {500, 1000, 0,  500}, // P to D1S, D1E, D_orig
                    {1000, 500, 500, 0}   // D_orig
                };
                long[][] timeMatrix = new long[][]{ // Times match distances
                    {0,  20, 50, 100},
                    {20, 0,  100, 50},
                    {50, 100, 0,  50}, // P -> D_orig = 50s. P -> D1E = 100s
                    {100, 50, 50, 0}
                };
                // Cost of P -> D_orig = 50.
                // Cost of P -> D1E (depot) = 100. DEPOT_DROP_PENALTY is also added.
                // Direct delivery should be chosen.

                // 3. Solve
                Solution solution = orToolsService.solve(problem, distanceMatrix, timeMatrix);

                // 4. Assertions
                assertNotNull(solution);
                assertTrue(solution.getDroppedRides().isEmpty(), "Ride should not be fully dropped.");
                assertTrue(solution.getDepotDroppedRides().isEmpty(), "Ride should not be depot-dropped if direct is optimal.");

                assertEquals(1, solution.getRoutes().size(), "Should have one route for V1.");
                Route vehicle1Route = solution.getRoutes().get(0);
                assertEquals("V1", vehicle1Route.getVehicleId());

                boolean pickupVisited = false;
                boolean originalDeliveryVisited = false;
                for (Visit visit : vehicle1Route.getVisits()) {
                    if (visit.getRideId() != null && visit.getRideId().equals("R1")) {
                        if (visit.getType() == TaskType.PICKUP) {
                            pickupVisited = true;
                            assertFalse(visit.isDepotDropPickup(), "Pickup visit should NOT be marked as part of depot drop.");
                            assertEquals(Duration.ofSeconds(50), visit.getArrivalTime()); // D1S -> P
                        }
                        if (visit.getType() == TaskType.DELIVERY) {
                            originalDeliveryVisited = true;
                            assertEquals(Duration.ofSeconds(100), visit.getArrivalTime()); // P -> D_orig (50 + 50)
                        }
                    }
                }
                assertTrue(pickupVisited, "Pickup for R1 should be on V1's route.");
                assertTrue(originalDeliveryVisited, "Original delivery for R1 should be on V1's route.");
            }

            @Test
            void testRideFullyDropped_WhenPickupIsImpossible() {
                // 1. Setup: Vehicle, Ride Request with impossible pickup time
                Depot depot1Start = new Depot();
                depot1Start.setId("D1S");
                depot1Start.setCoordinates(new Coordinate(0,0));

                Depot depot1End = new Depot();
                depot1End.setId("D1E");
                depot1End.setCoordinates(new Coordinate(0,1));

                Vehicle vehicle1 = new Vehicle();
                vehicle1.setId("V1");
                vehicle1.setDepotStart(depot1Start);
                vehicle1.setDepotEnd(depot1End);
                vehicle1.setSeatCapacity(4);
                // Vehicle shift: 0s to 100s
                vehicle1.setTimeWindow(new TimeWindow(Duration.ofSeconds(0), Duration.ofSeconds(100)));

                PickupDeliveryTask pickupTask = new PickupDeliveryTask();
                pickupTask.setCoordinates(new Coordinate(1,0));
                // Pickup window (110s-120s) is entirely outside vehicle's shift (0-100s)
                pickupTask.setTimeWindow(new TimeWindow(Duration.ofSeconds(110), Duration.ofSeconds(120)));
                pickupTask.setType(TaskType.PICKUP);

                PickupDeliveryTask deliveryTask = new PickupDeliveryTask();
                deliveryTask.setCoordinates(new Coordinate(1,5));
                deliveryTask.setTimeWindow(new TimeWindow(Duration.ofSeconds(130), Duration.ofSeconds(140)));
                deliveryTask.setType(TaskType.DELIVERY);

                RideRequest ride1 = new RideRequest();
                ride1.setId("R1");
                ride1.setUserId("U1");
                ride1.setPickup(pickupTask);
                ride1.setDelivery(deliveryTask);
                pickupTask.setRide(ride1);
                deliveryTask.setRide(ride1);

                Problem problem = createSimpleProblem(vehicle1, ride1);

                // 2. Setup distance and time matrices (can be simple, won't be used if pickup fails)
                long[][] distanceMatrix = new long[][]{
                    {0, 10, 50, 100},
                    {10, 0, 50, 100},
                    {50, 50, 0, 50},
                    {100, 100, 50, 0}
                };
                long[][] timeMatrix = distanceMatrix; // Times match distances

                // 3. Solve
                Solution solution = orToolsService.solve(problem, distanceMatrix, timeMatrix);

                // 4. Assertions
                assertNotNull(solution);
                assertFalse(solution.getDroppedRides().isEmpty(), "Ride should be fully dropped.");
                assertEquals(1, solution.getDroppedRides().size(), "One ride should be fully dropped.");
                assertEquals("R1", solution.getDroppedRides().get(0), "Ride R1 should be the one dropped.");
                assertTrue(solution.getDepotDroppedRides().isEmpty(), "No rides should be depot-dropped.");
                assertTrue(solution.getRoutes().isEmpty(), "No routes should be formed if the only ride is dropped.");
            }
}
