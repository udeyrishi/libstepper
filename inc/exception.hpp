/**
 Copyright (c) 2018 Udey Rishi. All rights reserved.
*/

#pragma once

#include <stdexcept>
#include <string>

namespace libstepper {

class IllegalStateError : public std::runtime_error {
public:
    IllegalStateError(const std::string &message) : runtime_error(message) {
    }
};

}