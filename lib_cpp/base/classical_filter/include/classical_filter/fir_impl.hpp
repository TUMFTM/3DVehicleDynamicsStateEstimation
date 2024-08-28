// Copyright 2023 Marcel Weinmann
#pragma once
#include "classical_filter/fir.hpp"
namespace tam::core::state
{
inline FIR::FIR(const Eigen::Ref<const Eigen::VectorXd> & coefficients)
{
    // initialize the filter coefficients
    filter_coefficients_ = coefficients;
    fifo_.resize(coefficients.size(), 0);
}

inline double FIR::step(double input)
{
    double output = 0.0;

    // update the FIFO of the FIR buffer
    fifo_.push_back(input);
    fifo_.pop_front();

    // calculate the dot product between FIR and coefficients
    for (int i=0; i < filter_coefficients_.size(); i++) {
        output += filter_coefficients_[i] * fifo_[i];
    }

    return output;
}
}  // namespace tam::core::state
