#ifndef KFA_TYPE_VEC_H
#define KFA_TYPE_VEC_H

#include "Type.h"

/**
 * @brief Derived Type class that implements vector variables
 */
class Vec : public Type {
 public:
  /**
   * @brief Default constructor for Vec
   * @param dim Size of the vector (will be same as error state)
   */
  Vec(int dim) : Type(dim) {
    value_ = Eigen::VectorXd::Zero(dim);
    fej_ = Eigen::VectorXd::Zero(dim);
  }

  ~Vec() {}

  /**
   * @brief Implements the update operation through standard vector addition
   * @param dx Additive error state correction
   */
  void update(const Eigen::VectorXd dx) override {
    assert(dx.rows() == size_);
    set_value(value_ + dx);
  }

  /**
   * @brief Performs all the cloning
   */
  Type *clone() override {
    Type *Clone = new Vec(size_);
    Clone->set_value(value());
    Clone->set_fej(fej());
    return Clone;
  }
};

#endif  // KFA_TYPE_VEC_H