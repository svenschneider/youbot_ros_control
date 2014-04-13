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

#ifndef YRC_HARDWARE_INTERFACE_OMNI_BASE_COMMAND_INTERFACE_H
#define YRC_HARDWARE_INTERFACE_OMNI_BASE_COMMAND_INTERFACE_H

#include <cassert>
#include <string>
#include <hardware_interface/internal/hardware_resource_manager.h>
#include <yrc_hardware_interface/omni_base_state_interface.h>

namespace yrc_hardware_interface
{

/** \brief A handle used to read and command a single joint. */
class OmniBaseHandle : public OmniBaseStateHandle
{
public:
  OmniBaseHandle() : OmniBaseStateHandle(), cmd_vel_x_(0), cmd_vel_y_(0), cmd_vel_theta_(0) {}

  /**
   * \param bs The omni-directional base's state handle
   * \param cmd_vel_x A pointer to the storage for the longitudinal velocity.
   * \param cmd_vel_y A pointer to the storage for the transversal velocity.
   * \param cmd_vel_theta A pointer to the storage for the rotational velocity.
   */
  OmniBaseHandle(const OmniBaseStateHandle& bs, double* cmd_vel_x, double* cmd_vel_y, double* cmd_vel_theta)
    : OmniBaseStateHandle(bs), cmd_vel_x_(cmd_vel_x), cmd_vel_y_(cmd_vel_y), cmd_vel_theta_(cmd_vel_theta)
  {
    if (!cmd_vel_x)
    {
      throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + bs.getName() + "'."
              "Longitudinal velocity command data pointer is null.");
    }
    if (!cmd_vel_y)
    {
      throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + bs.getName() + "'."
              "Transversal velocity command data pointer is null.");
    }
    if (!cmd_vel_theta)
    {
      throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + bs.getName() + "'."
              "Rotational velocity command data pointer is null.");
    }
  }

  void setCommandVelocityX(double command) {assert(cmd_vel_x_); *cmd_vel_x_ = command;}
  void setCommandVelocityY(double command) {assert(cmd_vel_y_); *cmd_vel_y_ = command;}
  void setCommandVelocityTheta(double command) {assert(cmd_vel_theta_); *cmd_vel_theta_ = command;}
  double getCommandVelX() const {assert(cmd_vel_x_); return *cmd_vel_x_;}
  double getCommandVelY() const {assert(cmd_vel_y_); return *cmd_vel_y_;}
  double getCommandVelTheta() const {assert(cmd_vel_theta_); return *cmd_vel_theta_;}

private:
  double* cmd_vel_x_;
  double* cmd_vel_y_;
  double* cmd_vel_theta_;
};

/** \brief Hardware interface to support commanding an array of joints.
 *
 * This \ref HardwareInterface supports commanding the output of an array of
 * named joints. Note that these commands can have any semantic meaning as long
 * as they each can be represented by a single double, they are not necessarily
 * effort commands. To specify a meaning to this command, see the derived
 * classes like \ref EffortJointInterface etc.
 *
 * \note Getting a joint handle through the getHandle() method \e will claim that resource.
 *
 */
class OmniBaseCommandInterface : public hardware_interface::HardwareResourceManager<OmniBaseHandle, hardware_interface::ClaimResources> {};

}

#endif
