/*
 * Copyright (c) 2021 Christopher Doer
 * Copyright (c) 2014, Autonomous Systems Lab
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 * * Neither the name of the Autonomous Systems Lab, ETH Zurich nor the
 * names of its contributors may be used to endorse or promote products
 * derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef ROVIO_VELOCITYUPDATE_HPP_
#define ROVIO_VELOCITYUPDATE_HPP_

#include <cstdint>
#include <limits>

#include <ros/ros.h>

#include "lightweight_filtering/common.hpp"
#include "lightweight_filtering/Update.hpp"
#include "lightweight_filtering/State.hpp"
#include "rovio/FilterStates.hpp"

namespace rovio
{
/** \brief Class, defining the innovation.
 */
class VelocityInnovation : public LWF::State<LWF::VectorElement<3>>
{
public:
  typedef LWF::State<LWF::VectorElement<3>> Base;
  using Base::E_;
  static constexpr unsigned int _vel = 0;
  VelocityInnovation()
  {
    static_assert(_vel + 1 == E_, "Error with indices");
    this->template getName<_vel>() = "vel";
  };
  virtual ~VelocityInnovation(){};
  inline V3D& vel() { return this->template get<_vel>(); }
  inline const V3D& vel() const { return this->template get<_vel>(); }
};

/** \brief Class, dummy auxillary class for Zero velocity update
 */
class VelocityUpdateMeasAuxiliary : public LWF::AuxiliaryBase<VelocityUpdateMeasAuxiliary>
{
public:
  VelocityUpdateMeasAuxiliary(){};
  virtual ~VelocityUpdateMeasAuxiliary(){};
};

/**  \brief Empty measurement
 */
class VelocityUpdateMeas : public LWF::State<LWF::VectorElement<3>, VelocityUpdateMeasAuxiliary>
{
public:
  typedef LWF::State<LWF::VectorElement<3>, VelocityUpdateMeasAuxiliary> Base;
  using Base::E_;
  static constexpr unsigned int _vel = 0;
  static constexpr unsigned int _aux = _vel + 1;
  VelocityUpdateMeas()
  {
    static_assert(_aux + 1 == E_, "Error with indices");
    this->template getName<_vel>() = "vel";
    this->template getName<_aux>() = "aux";
  };
  virtual ~VelocityUpdateMeas(){};
  inline V3D& vel() { return this->template get<_vel>(); }
  inline const V3D& vel() const { return this->template get<_vel>(); }
};

/**  \brief Class holding the update noise.
 */
class VelocityUpdateNoise : public LWF::State<LWF::VectorElement<3>>
{
public:
  typedef LWF::State<LWF::VectorElement<3>> Base;
  using Base::E_;
  static constexpr unsigned int _vel = 0;
  VelocityUpdateNoise()
  {
    static_assert(_vel + 1 == E_, "Error with indices");
    this->template getName<_vel>() = "vel";
  };
  virtual ~VelocityUpdateNoise(){};
  inline V3D& vel() { return this->template get<_vel>(); }
  inline const V3D& vel() const { return this->template get<_vel>(); }
  inline void setNoise(const V3D& sigma)
  {
    this->template get<_vel>() = sigma;
    std::cout << "new noise " << this->template get<_vel>() << std::endl;
  }
};

/** \brief Outlier Detection.
 * ODEntry<Start entry, dimension of detection>
 */
class VelocityOutlierDetection
  : public LWF::OutlierDetection<LWF::ODEntry<VelocityInnovation::template getId<VelocityInnovation::_vel>(), 3>>
{
public:
  virtual ~VelocityOutlierDetection(){};
};

struct VelocityUpdateDiag
{
  double mahalanobis_distance = std::numeric_limits<double>::quiet_NaN();
  bool is_outlier             = false;
  uint64_t seq                = 0;
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/** \brief Class, holding the zero velocity update
 */
template <typename FILTERSTATE>
class VelocityUpdate : public LWF::Update<VelocityInnovation,
                                          FILTERSTATE,
                                          VelocityUpdateMeas,
                                          VelocityUpdateNoise,
                                          VelocityOutlierDetection,
                                          false>
{
public:
  typedef LWF::
      Update<VelocityInnovation, FILTERSTATE, VelocityUpdateMeas, VelocityUpdateNoise, VelocityOutlierDetection, false>
          Base;
  using Base::doubleRegister_;
  using Base::intRegister_;
  using Base::meas_;
  using Base::H_;
  using Base::Hlin_;
  using Base::Hn_;
  using Base::innVector_;
  using Base::Py_;
  using Base::Pyinv_;
  using Base::updnoiP_;
  using Base::y_;
  using Base::yIdentity_;
  typedef typename Base::mtState mtState;
  typedef typename Base::mtFilterState mtFilterState;
  typedef typename Base::mtInnovation mtInnovation;
  typedef typename Base::mtMeas mtMeas;
  typedef typename Base::mtNoise mtNoise;
  typedef typename Base::mtOutlierDetection mtOutlierDetection;

  QPD qAM_;  // Rotation between IMU (M) and coordinate frame where the velocity is expressed in (A)
  VelocityUpdateDiag last_diag_;

  /** \brief Constructor.
   *
   *   Loads and sets the needed parameters.
   */
  VelocityUpdate()
  {
    qAM_.setIdentity();
    intRegister_.removeScalarByStr("maxNumIteration");
    doubleRegister_.removeScalarByStr("alpha");
    doubleRegister_.removeScalarByStr("beta");
    doubleRegister_.removeScalarByStr("kappa");
    doubleRegister_.removeScalarByStr("updateVecNormTermination");
    doubleRegister_.registerQuaternion("qAM", qAM_);

    Base::disablePreAndPostProcessingWarning_ = true;
  }

  /** \brief Destructor
   */
  virtual ~VelocityUpdate(){};

  virtual void postProcess(mtFilterState& filterState,
                           const mtMeas& meas,
                           const mtOutlierDetection& outlierDetection,
                           bool& isFinished)
  {
    (void)filterState;
    (void)meas;
    last_diag_.mahalanobis_distance = outlierDetection.getMahalDistance(0);
    last_diag_.is_outlier           = outlierDetection.isOutlier(0);
    last_diag_.seq += 1;
    isFinished = true;
  }

  /**
   * @brief Updates the measurement noise
   * @param cov
   */
  void setMeasurementNoise(const Eigen::MatrixXd& cov) { Base::updnoiP_ = cov; }
  const VelocityUpdateDiag& getLastDiag() const { return last_diag_; }
  uint64_t getUpdateSeq() const { return last_diag_.seq; }

  bool computeCandidateNis(const mtFilterState& filterState,
                           const mtMeas& meas,
                           const Eigen::MatrixXd& cov,
                           double& nis)
  {
    nis = std::numeric_limits<double>::quiet_NaN();
    if (cov.rows() != mtNoise::D_ || cov.cols() != mtNoise::D_ || !cov.allFinite())
      return false;

    const mtMeas old_meas = meas_;
    const Eigen::MatrixXd old_cov = updnoiP_;

    meas_ = meas;
    updnoiP_ = cov;
    this->jacState(H_, filterState.state_);
    Hlin_ = H_;
    this->jacNoise(Hn_, filterState.state_);
    this->evalInnovationShort(y_, filterState.state_);
    Py_ = Hlin_ * filterState.cov_ * Hlin_.transpose() + Hn_ * updnoiP_ * Hn_.transpose();
    y_.boxMinus(yIdentity_, innVector_);

    const auto restore = [&]() {
      meas_ = old_meas;
      updnoiP_ = old_cov;
    };

    if (!Py_.allFinite() || !innVector_.allFinite())
    {
      restore();
      return false;
    }

    Pyinv_.setIdentity();
    Eigen::LLT<MXD> llt(Py_);
    if (llt.info() != Eigen::Success)
    {
      restore();
      return false;
    }
    llt.solveInPlace(Pyinv_);
    nis = (innVector_.transpose() * Pyinv_ * innVector_)(0);
    restore();
    return std::isfinite(nis);
  }

  /** \brief Compute the inovvation term
   *
   *  @param mtInnovation - Class, holding innovation data.
   *  @param state        - Filter %State.
   *  @param meas         - Not Used.
   *  @param noise        - Additive discrete Gaussian noise.
   *  @param dt           - Not used.
   */
  void evalInnovation(mtInnovation& y, const mtState& state, const mtNoise& noise) const
  {
    y.vel() = qAM_.rotate(state.MvM()) + meas_.vel() + noise.vel();  // Velocity of state has a minus sign
  }

  /** \brief Computes the Jacobian for the update step of the filter.
   *
   *  @param F     - Jacobian for the update step of the filter.
   *  @param state - Filter state.
   *  @param meas  - Not used.
   *  @param dt    - Not used.
   */
  void jacState(MXD& F, const mtState& state) const
  {
    F.setZero();
    F.template block<3, 3>(mtInnovation::template getId<mtInnovation::_vel>(),
                           mtState::template getId<mtState::_vel>()) = MPD(qAM_).matrix();
  }

  /** \brief Computes the Jacobian for the update step of the filter w.r.t. to the noise variables
   *
   *  @param G     - Jacobian for the update step of the filter.
   *  @param state - Filter state.
   *  @param meas  - Not used.
   *  @param dt    - Not used.
   */
  void jacNoise(MXD& G, const mtState& state) const
  {
    G.setZero();
    G.template block<3, 3>(mtInnovation::template getId<mtInnovation::_vel>(),
                           mtNoise::template getId<mtNoise::_vel>()) = Eigen::Matrix3d::Identity();
  }
};

}  // namespace rovio

#endif /* ROVIO_VELOCITYUPDATE_HPP_ */
