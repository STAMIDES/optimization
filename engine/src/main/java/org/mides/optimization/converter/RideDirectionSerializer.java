package org.mides.optimization.converter;

import com.fasterxml.jackson.core.JsonGenerator;
import com.fasterxml.jackson.databind.JsonSerializer;
import com.fasterxml.jackson.databind.SerializerProvider;
import org.mides.optimization.model.RideDirection;

import java.io.IOException;

public class RideDirectionSerializer extends JsonSerializer<RideDirection> {

    @Override
    public void serialize(RideDirection rideDir, JsonGenerator gen, SerializerProvider serializer) throws IOException {
        gen.writeString(rideDir.toString().toLowerCase());
    }
}
