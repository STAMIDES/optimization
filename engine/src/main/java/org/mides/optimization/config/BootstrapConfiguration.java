package org.mides.optimization.config;

import com.google.ortools.Loader;
import jakarta.annotation.PostConstruct;
import org.springframework.context.annotation.Scope;
import org.springframework.stereotype.Component;

@Component
@Scope("singleton")
public class BootstrapConfiguration {
    @PostConstruct
    public void init() {
        Loader.loadNativeLibraries();
    }
}
