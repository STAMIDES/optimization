package org.mides.optimization.model;

import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.databind.annotation.JsonDeserialize;
import com.fasterxml.jackson.databind.annotation.JsonSerialize;
import jakarta.validation.constraints.NotNull;
import lombok.Data;
import lombok.NoArgsConstructor;
import org.mides.optimization.converter.DurationDeserializer;
import org.mides.optimization.converter.DurationSerializer;

import java.time.Duration;

@Data
@NoArgsConstructor
public class TimeWindow {
    public static final long MIN_SECONDS = 0;
    public static final long MAX_SECONDS = 86400;

    @NotNull
    @JsonProperty("start")
    @JsonDeserialize(using = DurationDeserializer.class)
    @JsonSerialize(using = DurationSerializer.class)
    private Duration start = Duration.ofSeconds(MIN_SECONDS);

    @NotNull
    @JsonProperty("end")
    @JsonDeserialize(using = DurationDeserializer.class)
    @JsonSerialize(using = DurationSerializer.class)
    private Duration end = Duration.ofSeconds(MAX_SECONDS);

    public long startSeconds() {
        return start.getSeconds();
    }

    public long endSeconds() {
        return end.getSeconds();
    }

    public TimeWindow(Duration start, Duration end) {
        this.start = start;
        this.end = end;
    }
}
