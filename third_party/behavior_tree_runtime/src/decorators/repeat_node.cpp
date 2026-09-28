// ======================================================================================
// ======================================================================================

/* Copyright (C) 2015-2018 Michele Colledanchise -  All Rights Reserved
 * Copyright (C) 2018-2020 Davide Faconti, Eurecat -  All Rights Reserved
*
*   Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
*   to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
*   and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
*   The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*
*   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
*   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
*   WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "behaviortree_cpp/decorators/repeat_node.h"

namespace BT
{

// ======================================================================================
// ======================================================================================
RepeatNode::RepeatNode(const std::string& name, int NTries)
  : DecoratorNode(name, {})
  , num_cycles_(NTries)
  , repeat_count_(0)
  , read_parameter_from_ports_(false)
{
  setRegistrationID("Repeat");
}

// ======================================================================================
// ======================================================================================
RepeatNode::RepeatNode(const std::string& name, const NodeConfig& config)
  : DecoratorNode(name, config)
  , num_cycles_(0)
  , repeat_count_(0)
  , read_parameter_from_ports_(true)
{}

// ======================================================================================
// ======================================================================================
NodeStatus RepeatNode::tick()
{
  if(read_parameter_from_ports_)
  {
    if(!getInput(NUM_CYCLES, num_cycles_))
    {
      throw RuntimeError("Missing parameter [", NUM_CYCLES, "] in RepeatNode");
    }
  }

  bool do_loop = repeat_count_ < num_cycles_ || num_cycles_ == -1;
  setStatus(NodeStatus::RUNNING);

  while(do_loop)
  {
    NodeStatus const prev_status = child_node_->status();
    NodeStatus child_status = child_node_->executeTick();

    switch(child_status)
    {
      case NodeStatus::SUCCESS: {
        repeat_count_++;
        do_loop = repeat_count_ < num_cycles_ || num_cycles_ == -1;

        resetChild();

        if(requiresWakeUp() && prev_status == NodeStatus::IDLE && do_loop)
        {
          emitWakeUpSignal();
          return NodeStatus::RUNNING;
        }
      }
      break;

      case NodeStatus::FAILURE: {
        repeat_count_ = 0;
        resetChild();
        return (NodeStatus::FAILURE);
      }

      case NodeStatus::RUNNING: {
        return NodeStatus::RUNNING;
      }

      case NodeStatus::SKIPPED: {
        resetChild();
        return NodeStatus::SKIPPED;
      }

      case NodeStatus::IDLE: {
        throw LogicError("[", name(), "]: A children should not return IDLE");
      }
    }
  }

  repeat_count_ = 0;
  return NodeStatus::SUCCESS;
}

// ======================================================================================
// ======================================================================================
void RepeatNode::halt()
{
  repeat_count_ = 0;
  DecoratorNode::halt();
}

}  // namespace BT
