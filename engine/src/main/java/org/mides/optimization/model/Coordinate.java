package org.mides.optimization.model;

import jakarta.validation.constraints.NotNull;
import lombok.AllArgsConstructor;
import lombok.Data;
import lombok.NoArgsConstructor;

@Data
@NoArgsConstructor
@AllArgsConstructor
public class Coordinate {
    @NotNull
    private Double latitude;

    @NotNull
    private Double longitude;

    @Override
    public String toString() {
        return String.format("%.8f,%.8f", longitude, latitude);
    }
}
