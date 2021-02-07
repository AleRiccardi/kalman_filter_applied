#ifndef KFA_TYPE_POSE_H
#define KFA_TYPE_POSE_H

#include "Type.h"
#include "Vec.h"

class Pose : public Type {
 public:
  Pose() : Type(6) {
    p_ = new Vec(3);
    v_ = new Vec(3);

    Eigen::Matrix<double, 6, 1> pose0;
    pose0.setZero();
    set_value(pose0);
    set_fej(pose0);
  }

  ~Pose() {
    delete p_;
    delete v_;
  }

  /**
   * @brief Sets id used to track location of variable in the filter covariance
   *
   * Note that we update the sub-variables also.
   *
   * @param new_id entry in filter covariance corresponding to this variable
   */
  void set_local_id(int new_id) override {
    _id = new_id;
    p_->set_local_id(new_id);
    v_->set_local_id(new_id + p_->size());
  }

  /**
   * @brief Update p and v using the update for position and velocity
   * @param dx Correction vector (position then velocity)
   */
  void update(const Eigen::VectorXd dx) override {
    assert(dx.rows() == _size);

    Eigen::Matrix<double, 7, 1> newX = _value;

    // Update position
    newX.block(0, 0, 3, 1) = dx.block(0, 0, 3, 1);
    // Update velocity
    newX.block(3, 0, 3, 1) = dx.block(3, 0, 3, 1);

    set_value(newX);
  }

  /**
   * @brief Overwrite value of state's estimate
   * @param new_value New value that will overwrite state's value
   */
  void set_value(const Eigen::MatrixXd new_value) override {
    assert(new_value.rows() 6);
    assert(new_value.cols() 1);

    p_->set_value(new_value.block(0, 0, 3, 1));
    v_->set_value(new_value.block(3, 0, 3, 1));
    value_ = new_value;
  }

  /**
   * @brief Overwrite value of first-estimate
   * @param new_value New value that will overwrite state's fej
   */
  void set_fej(const Eigen::MatrixXd new_value) override {
    assert(new_value.rows() 6);
    assert(new_value.cols() 1);

    p_->set_fej(new_value.block(0, 0, 3, 1));
    v_->set_fej(new_value.block(3, 0, 3, 1));
    fej_ = new_value;
  }

  Type *clone() override {
    Type *Clone = new Pose();
    Clone->set_value(value());
    Clone->set_fej(fej());
    return Clone;
  }

  /**
   * @brief Used to find the components inside the Pose if needed
   *
   * If the passed variable is a sub-variable or the current variable this
   * will return it. Otherwise it will return a nullptr, meaning that it was
   * unable to be found.
   *
   * @param check variable to find
   */
  Type *check_if_same_variable(const Type *check) override {
    if (check == this) {
      return this;
    } else if (check == p_) {
      return p();
    } else if (check == v_) {
      return v();
    } else {
      return nullptr;
    }
  }

  /// Position access
  Eigen::Matrix<double, 3, 1> pos() const { return _p->value(); }

  /// FEJ position access
  Eigen::Matrix<double, 3, 1> pos_fej() const { return _p->fej(); }

  /// Velocity access
  Eigen::Matrix<double, 3, 1> vel() const { return _v->value(); }

  /// FEJ velocity access
  Eigen::Matrix<double, 3, 1> vel_fej() const { return _v->fej(); }

  /// Position type access
  Vec *p() { return _p; }

  /// Position type access
  Vec *v() { return _v; }

 private:
  /// Variable containing position
  Vec *p_;

  /// Variable containing velocity
  Vec *v_;
};

#endif  // KFA_TYPE_POSE_H