import math
import numpy as np
import gtsam
from gtsam.symbol_shorthand import L, X

PRIOR_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1, 0.05]))
ODOMETRY_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.2, 0.2, 0.1]))
MEASUREMENT_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.05, 0.1]))

def add_pose(graph, initial_estimate):
    odometry = gtsam.Pose2(2.0, 0.0, 0.0)

    graph.add(
        gtsam.BetweenFactorPose2(
            X(4),
            X(5),
            odometry,
            ODOMETRY_NOISE
        )
    )

    x4_estimate = initial_estimate.atPose2(X(4))
    x5_estimate = x4_estimate.compose(odometry)

    initial_estimate.insert(X(5), x5_estimate)

    return graph, initial_estimate