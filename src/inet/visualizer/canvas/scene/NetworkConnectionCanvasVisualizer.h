//
// Copyright (C) 2020 OpenSim Ltd.
//
// SPDX-License-Identifier: LGPL-3.0-or-later
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.
//

#ifndef __INET_NETWORKCONNECTIONCANVASVISUALIZER_H
#define __INET_NETWORKCONNECTIONCANVASVISUALIZER_H

#include "inet/common/geometry/common/CanvasProjection.h"
#include "inet/visualizer/base/NetworkConnectionVisualizerBase.h"

namespace inet {

namespace visualizer {

class INET_API NetworkConnectionCanvasVisualizer : public NetworkConnectionVisualizerBase
{
  protected:
    double zIndex = NaN;
    const CanvasProjection *canvasProjection = nullptr;

  protected:
    virtual void initialize(int stage) override;

    void createNetworkConnectionVisualization(cModule *startNetworkNode, cModule *endNetworkNode) override;
};

} // namespace visualizer

} // namespace inet

#endif

