//
// Copyright (C) 2013 OpenSim Ltd.
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

#ifndef __INET_LIFECYCLEOPERATION_H
#define __INET_LIFECYCLEOPERATION_H

#include "inet/common/INETDefs.h"

namespace inet {

class LifecycleController;
class IDoneCallback;

/**
 * Base class for operations used by the ILifecycle interface. Subclasses
 * represent "operations" like shutdown, suspend, failure, restart, etc.
 *
 * @see LifecycleController, ILifecycle
 */
class INET_API LifecycleOperation : public cObject, public noncopyable
{
  public:
    friend class LifecycleController;
    typedef std::map<std::string, std::string> StringMap;

  private:
    cModule *rootModule = nullptr;
    int currentStage = 0;
    std::vector<IDoneCallback *> pendingList;
    bool insideInitiateOperation = false;
    IDoneCallback *operationCompletionCallback = nullptr;

  public:
    LifecycleOperation() :
        rootModule(nullptr), currentStage(0), insideInitiateOperation(false), operationCompletionCallback(nullptr) {}

    /**
     * Initialize the operation using the parameters provided in the
     * string map. The implementation should destructively modify the map,
     * removing from it the parameters it understands. Ideally, the map should
     * be empty when this method returns; if it is not, the caller should
     * treat that as an error, and report the remaining parameters as
     * unrecognized by the operation.
     */
    virtual void initialize(cModule *module, StringMap& params) {
        cProperties *props = module->getProperties();
        if (props && (props->getAsBool("networkNode") || props->getAsBool("lifecycleSupport")))
            rootModule = module;
        else
            throw cRuntimeError("LifecycleOperation not accepted directly by '(%s)%s' module", module->getClassName(), module->getFullPath().c_str());
    }

    /**
     * Returns the module the operation is initiated on.
     */
    cModule *getRootModule() const { return rootModule; }

    /**
     * Returns the number of stages required by this operation.
     */
    virtual int getNumStages() const = 0;

    /**
     * Returns the current stage, an integer in 0..numStages-1.
     */
    int getCurrentStage() const { return currentStage; }
};

} // namespace inet

#endif

