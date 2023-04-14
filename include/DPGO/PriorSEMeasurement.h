/* ----------------------------------------------------------------------------
 * Copyright 2020, Massachusetts Institute of Technology, * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Yulun Tian, et al. (see README for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

#ifndef PRIORSEMEASUREMENT_H
#define PRIORSEMEASUREMENT_H

#include <DPGO/DPGO_types.h>

#include <Eigen/Dense>
#include <iostream>

namespace DPGO {

/** A simple struct that contains the elements of a Prior SE measurement
    on pose (r, p)
 */
struct PriorSEMeasurement {
  /** 0-based index of first robot */
  size_t robot_idx;

  /** 0-based index of first pose */
  size_t pose_idx;

  /** Rotational measurement */
  Matrix rotation;

  /** Translational measurement */
  Matrix translation;

	/** Priors need to be represented in lifted space**/
	Matrix liftingMatrix;

	/** The rank-relaxed lifted pose  **/
	Matrix liftedPose;

  /** Rotational measurement precision */
  double kappa;

  /** Translational measurement precision */
  double tau;

  /** If measurement weight is fixed */
  bool fixedWeight;

  /** Weight between (0,1) used in Graduated Non-Convexity */
  double weight;

  /** Simple default constructor; does nothing */
  PriorSEMeasurement() = default;

  /** Basic constructor */
  PriorSEMeasurement(size_t robot_idx, size_t pose_idx,
                        const Eigen::MatrixXd &rotation,
                        const Eigen::VectorXd &translation,
												const Eigen::MatrixXd &liftingMatrix,
                        double rotational_precision,
                        double translational_precision,
												bool fixedWeight=false, 
												double weight =1.0)
      : robot_idx(robot_idx),
        pose_idx(pose_idx),
        rotation(rotation),
        translation(translation),
				liftingMatrix(liftingMatrix),
        kappa(rotational_precision),
        tau(translational_precision),
        fixedWeight(fixedWeight),
        weight(weight) {
					size_t d = rotation.cols();
					Matrix pose(d, d+1);
					pose.block(0,0, d,d) = rotation;
					pose.block(0, d, d, 1) = translation;
					liftedPose = liftingMatrix * pose;
				}

  /** A utility function for streaming this struct to cout */
  inline friend std::ostream &operator<<(
      std::ostream &os, const PriorSEMeasurement &measurement) {
    os << "robot_idx: " << measurement.robot_idx << std::endl;
    os << "pose_idx: " << measurement.pose_idx << std::endl;
    os << "R: " << std::endl << measurement.rotation << std::endl;
    os << "t: " << std::endl << measurement.translation << std::endl;
    os << "Kappa: " << measurement.kappa << std::endl;
    os << "Tau: " << measurement.tau << std::endl;
    os << "Fixed weight: " << measurement.fixedWeight << std::endl;
    os << "Weight: " << measurement.weight << std::endl;

    return os;
  }
};
};  // namespace DPGO
#endif
