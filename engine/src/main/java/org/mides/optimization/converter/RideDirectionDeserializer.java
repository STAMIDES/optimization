package org.mides.optimization.converter;

import com.fasterxml.jackson.core.JsonParser;
import com.fasterxml.jackson.databind.DeserializationContext;
import com.fasterxml.jackson.databind.JsonDeserializer;
import org.mides.optimization.model.RideDirection;

import java.io.IOException;

public class RideDirectionDeserializer extends JsonDeserializer<RideDirection> {

    @Override
    public RideDirection deserialize(JsonParser p, DeserializationContext context) throws IOException {
        String direction = p.getText();
        return RideDirection.valueOf(direction.toUpperCase());
    }
}
