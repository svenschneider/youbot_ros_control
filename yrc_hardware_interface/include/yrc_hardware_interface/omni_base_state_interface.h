///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2012, hiDOF INC.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of hiDOF, Inc. nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////

/// \author Wim Meeussen
/// \author Sven Schneider

#ifndef YRC_HARDWARE_INTERFACE_OMNI_BASE_STATE_INTERFACE_H
#define YRC_HARDWARE_INTERFACE_OMNI_BASE_STATE_INTERFACE_H

#include <hardware_interface/internal/hardware_resource_manager.h>
#include <cassert>
#include <string>

namespace yrc_hardware_interface
{

/** A handle used to read the x/y/theta velocity state of an omni-directional base. */
class OmniBaseStateHandle
{
public:
  OmniBaseStateHandle() : name_(), vel_x_(0), vel_y_(0), vel_theta_(0) {}

  /**
   * \param name The name of the base
   * \param vel_x A pointer to the storage for the x-velocity
   * \param vel_y A pointer to the storage for the y-velocity
   * \param vel_theta A pointer to the storage for the rotational velocity
   */
  OmniBaseStateHandle(const std::string& name, const double* vel_x, const double* vel_y, const double* vel_theta)
    : name_(name), vel_x_(vel_x), vel_y_(vel_y), vel_theta_(vel_theta_)
  {
    if (!vel_x)
    {
      throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + name + "'. X-velocity data pointer is null.");
    }
    if (!vel_y)
    {
      throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + name + "'. Y-velocity data pointer is null.");
    }
    if (!vel_theta)
    {
      throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + name + "'. Theta velocity data pointer is null.");
    }
  }

  std::string getName() const {return name_;}
  double getVelocityX()  const {assert(vel_x_); return *vel_x_;}
  double getVelocityY()  const {assert(vel_y_); return *vel_y_;}
  double getVelocityTheta()    const {assert(vel_theta_); return *vel_theta_;}

private:
  std::string name_;
  const double* vel_x_;
  const double* vel_y_;
  const double* vel_theta_;
};

/** \brief Hardware interface to support reading the x/y/theta state of an omni-directional base.
 *
 */
class OmniBaseStateInterface : public hardware_interface::HardwareResourceManager<OmniBaseStateHandle> {};

}

#endif
