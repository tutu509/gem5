/*
 * Copyright (c) 2012 ARM Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Copyright (c) 2004-2005 The Regents of The University of Michigan
 * Copyright (c) 2013 Advanced Micro Devices, Inc.
 * Copyright (c) 2013 Mark D. Hill and David A. Wood
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Nathan Binkert
 *          Sascha Bischoff
 */

// This file will contain default statistics for the simulator that
// don't really belong to a specific simulator object

#include <fstream>
#include <iostream>
#include <list>

#include "base/callback.hh"
#include "base/hostinfo.hh"
#include "base/statistics.hh"
#include "base/time.hh"
#include "cpu/base.hh"
#include "sim/global_event.hh"
#include "sim/stat_control.hh"


//zlf
#include<map>
#include<vector>
#include "base/stats/text.hh"

//end
Stats::SparseHistogram Stack_Distance_Distribution_In_Order;
using namespace std;
//Stats::SpareHistogram Stack_Distance_Distribution_In_Order;
Stats::Formula simSeconds;
Stats::Value simTicks;
Stats::Value finalTick;
Stats::Value simFreq;

//zlf 2016-1-18
typedef std::map<unsigned int, std::vector<unsigned int>> Set_Map_Stack_In_Order;
Set_Map_Stack_In_Order set_map_stack_in_order;
typedef std::map<unsigned int ,unsigned int > Reorder_Map;
Reorder_Map reorder_map;
int setShift = 6;
int numSets = (32*1024)/(64*2);

int setMask = numSets-1;
int extractSet (unsigned int);

int extractSet (unsigned int addr_temp)
{
    return ((addr_temp)& setMask);
}
void swap_reorder(Reorder_Map &map)
{
    Reorder_Map map_temp;
    map.swap(map_temp);
}
void swap_stack(Set_Map_Stack_In_Order &map)
{
    Set_Map_Stack_In_Order map_temp;
    map.swap(map_temp);
}
void swap_vector(std::vector<unsigned int> &vec)
{
    std::vector<unsigned int > temp;
    vec.swap(temp);
}
void calculate();
void calculate()
{
 //   static unsigned int loads_number = 0; 
    int set_num_in_order = 0;
    Reorder_Map::iterator map_pos;
    std::vector<unsigned int>::iterator stack_pos_in_order;
    std::map<unsigned int ,std::vector<unsigned int>>::iterator swap_pos;
    for(map_pos = (reorder_map).begin();map_pos != reorder_map.end();map_pos++)
    {
  //      loads_number ++;
        int stack_distance_in_order = 0;
        int addr_temp0 = (*map_pos).second;
        set_num_in_order = extractSet(addr_temp0);
        int addr_temp1 = addr_temp0;
        if(set_map_stack_in_order.find(set_num_in_order) != (set_map_stack_in_order).end())
        {
            for(stack_pos_in_order = (set_map_stack_in_order)[set_num_in_order].end();stack_pos_in_order != (set_map_stack_in_order)[set_num_in_order].begin();)
            {
                stack_pos_in_order -- ;
                if (*stack_pos_in_order == addr_temp1)
                {
                    (set_map_stack_in_order)[set_num_in_order].erase(stack_pos_in_order);
                    Stack_Distance_Distribution_In_Order.sample(stack_distance_in_order);
                }
                else
                    stack_distance_in_order++;


            }
        }
        set_map_stack_in_order[extractSet(addr_temp0)].push_back(addr_temp1);

    }
//    std::cout<<"loads_number_in_order= "<<loads_number<<std::endl;
    
    for(swap_pos = set_map_stack_in_order.begin();swap_pos!= set_map_stack_in_order.end();swap_pos++)
    {
       swap_vector(swap_pos->second);
    }

    swap_stack(set_map_stack_in_order);
    swap_reorder(reorder_map);
    
}
//swap_reorder(reorder_map);

//end zlf 2016-1-18
namespace Stats {

Time statTime(true);
Tick startTick;

GlobalEvent *dumpEvent;

struct SimTicksReset : public Callback
{
    void process()
    {
        statTime.setTimer();
        startTick = curTick();
    }
};

double
statElapsedTime()
{
    Time now;
    now.setTimer();

    Time elapsed = now - statTime;
    return elapsed;
}

Tick
statElapsedTicks()
{
    return curTick() - startTick;
}

Tick
statFinalTick()
{
    return curTick();
}

SimTicksReset simTicksReset;

struct Global
{
    Stats::Formula hostInstRate;
    Stats::Formula hostOpRate;
    Stats::Formula hostTickRate;
    Stats::Value hostMemory;
    Stats::Value hostSeconds;

    Stats::Value simInsts;
    Stats::Value simOps;

    Global();
};

Global::Global()
{
    //zlf 2016-1-18
    Stack_Distance_Distribution_In_Order
        .init(0)
        .name("Stack_Distance_Distribution_In_Order")
        .desc("Stack_Distance_Distribution_In_Order")
        ;
    //end zlf 2016-1-18
    simInsts
        .functor(BaseCPU::numSimulatedInsts)
        .name("sim_insts")
        .desc("Number of instructions simulated")
        .precision(0)
        .prereq(simInsts)
        ;

    simOps
        .functor(BaseCPU::numSimulatedOps)
        .name("sim_ops")
        .desc("Number of ops (including micro ops) simulated")
        .precision(0)
        .prereq(simOps)
        ;

    simSeconds
        .name("sim_seconds")
        .desc("Number of seconds simulated")
        ;

    simFreq
        .scalar(SimClock::Frequency)
        .name("sim_freq")
        .desc("Frequency of simulated ticks")
        ;

    simTicks
        .functor(statElapsedTicks)
        .name("sim_ticks")
        .desc("Number of ticks simulated")
        ;

    finalTick
        .functor(statFinalTick)
        .name("final_tick")
        .desc("Number of ticks from beginning of simulation "
              "(restored from checkpoints and never reset)")
        ;

    hostInstRate
        .name("host_inst_rate")
        .desc("Simulator instruction rate (inst/s)")
        .precision(0)
        .prereq(simInsts)
        ;

    hostOpRate
        .name("host_op_rate")
        .desc("Simulator op (including micro ops) rate (op/s)")
        .precision(0)
        .prereq(simOps)
        ;

    hostMemory
        .functor(memUsage)
        .name("host_mem_usage")
        .desc("Number of bytes of host memory used")
        .prereq(hostMemory)
        ;

    hostSeconds
        .functor(statElapsedTime)
        .name("host_seconds")
        .desc("Real time elapsed on the host")
        .precision(2)
        ;

    hostTickRate
        .name("host_tick_rate")
        .desc("Simulator tick rate (ticks/s)")
        .precision(0)
        ;

    simSeconds = simTicks / simFreq;
    hostInstRate = simInsts / hostSeconds;
    hostOpRate = simOps / hostSeconds;
    hostTickRate = simTicks / hostSeconds;

    registerResetCallback(&simTicksReset);
}

void
initSimStats()
{
    static Global global;
}

/**
 * Event to dump and/or reset the statistics.
 */
class StatEvent : public GlobalEvent
{
  private:
    bool dump;
    bool reset;
    Tick repeat;

  public:
    StatEvent(Tick _when, bool _dump, bool _reset, Tick _repeat)
        : GlobalEvent(_when, Stat_Event_Pri, 0),
          dump(_dump), reset(_reset), repeat(_repeat)
    {
    }

    virtual void
    process()
    {
        //zlf 4 calculate 2016-1-18
        if_context_switch = true;
        calculate();
        //end zlf 2016-1-18
        if (dump)
            Stats::dump();

        if (reset)
            Stats::reset();

        if (repeat) {
            Stats::schedStatEvent(dump, reset, curTick() + repeat, repeat);
        }
    }

    const char *description() const { return "GlobalStatEvent"; }
};

void
schedStatEvent(bool dump, bool reset, Tick when, Tick repeat)
{
    // simQuantum is being added to the time when the stats would be
    // dumped so as to ensure that this event happens only after the next
    // sync amongst the event queues.  Asingle event queue simulation
    // should remain unaffected.
    dumpEvent = new StatEvent(when + simQuantum, dump, reset, repeat);
}

void
periodicStatDump(Tick period)
{
    /*
     * If the period is set to 0, then we do not want to dump periodically,
     * thus we deschedule the event. Else, if the period is not 0, but the event
     * has already been scheduled, we need to get rid of the old event before we
     * create a new one, as the old event will no longer be moved forward in the
     * event that we resume from a checkpoint.
     */
    if (dumpEvent != NULL && (period == 0 || dumpEvent->scheduled())) {
        // Event should AutoDelete, so we do not need to free it.
        dumpEvent->deschedule();
    }

    /*
     * If the period is not 0, we schedule the event. If this is called with a
     * period that is less than the current tick, then we shift the first dump
     * by curTick. This ensures that we do not schedule the event is the past.
     */
    if (period != 0) {
        // Schedule the event
        if (period >= curTick()) {
            schedStatEvent(true, true, (Tick)period, (Tick)period);
        } else {
            schedStatEvent(true, true, (Tick)period + curTick(), (Tick)period);
        }
    }
}

void
updateEvents()
{
    /*
     * If the dumpEvent has been scheduled, but is scheduled in the past, then
     * we need to shift the event to be at a valid point in time. Therefore, we
     * shift the event by curTick.
     */
    if (dumpEvent != NULL &&
        (dumpEvent->scheduled() && dumpEvent->when() < curTick())) {
        // shift by curTick() and reschedule
        Tick _when = dumpEvent->when();
        dumpEvent->reschedule(_when + curTick());
    }
}

} // namespace Stats
