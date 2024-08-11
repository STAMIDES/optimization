package org.mides.optimization.service;

import org.mides.optimization.model.Problem;
import org.mides.optimization.model.Solution;

public interface IORToolsService {
    Solution solve(Problem problem, long[][] distanceMatrix, long[][] timeMatrix);
}
