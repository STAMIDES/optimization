# Use a Maven image with OpenJDK 20.0.1 to build the project
FROM maven:3.9.4-eclipse-temurin-20 AS build

# Set the working directory in the container
WORKDIR /app

# Copy the pom.xml and download dependencies
COPY pom.xml .
RUN mvn dependency:go-offline

# Copy the entire project to the container
COPY . .

# Build the Spring Boot application
RUN mvn clean package -DskipTests

# Use OpenJDK 20.0.1 as the base image for the runtime
FROM eclipse-temurin:20.0.1_9-jdk

# Set the working directory in the container
WORKDIR /app

# Copy the jar file from the build stage
COPY --from=build /app/target/*.jar app.jar

# Set the default environment variable for OSRM base URL (optional)
ENV OSRM_BASE_URL=""

# Run the application, only overriding osrm.base-url if OSRM_BASE_URL is set
ENTRYPOINT ["sh", "-c", "java -jar app.jar ${OSRM_BASE_URL:+--osrm.base-url=$OSRM_BASE_URL}"]
