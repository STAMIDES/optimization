package org.mides.optimization.converter;

import com.fasterxml.jackson.core.JsonParser;
import com.fasterxml.jackson.databind.DeserializationContext;
import com.fasterxml.jackson.databind.JsonDeserializer;

import java.io.IOException;
import java.time.Duration;
import java.time.LocalTime;
import java.time.format.DateTimeFormatter;
import java.time.format.DateTimeParseException;

public class DurationDeserializer extends JsonDeserializer<Duration> {

    private static final DateTimeFormatter TIME_FORMATTER = DateTimeFormatter.ofPattern("HH:mm:ss");

    @Override
    public Duration deserialize(JsonParser p, DeserializationContext context) throws IOException {
        String durationStr = p.getText();
        try {
            LocalTime time = LocalTime.parse(durationStr, TIME_FORMATTER);
            return Duration.between(LocalTime.MIDNIGHT, time);
        } catch (DateTimeParseException e) {
            throw new IOException("Failed to parse Duration", e);
        }
    }
}
