/*
 * Copyright (c) 2012, 2014 ARM Limited
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
 * Copyright (c) 2004-2006 The Regents of The University of Michigan
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
 * Authors: Kevin Lim
 */

#ifndef __CPU_O3_MEM_DEP_UNIT_HH__
#define __CPU_O3_MEM_DEP_UNIT_HH__

#include <list>
#include <memory>
#include <set>
#include <unordered_map>

#include "base/statistics.hh"
#include "cpu/inst_seq.hh"
#include "debug/MemDepUnit.hh"

struct SNHash {
    size_t operator() (const InstSeqNum &seq_num) const {
        unsigned a = (unsigned)seq_num;
        unsigned hash = (((a >> 14) ^ ((a >> 2) & 0xffff))) & 0x7FFFFFFF;

        return hash;
    }
};

struct DerivO3CPUParams;

template <class Impl>
class InstructionQueue;

/**
 * Memory dependency unit class.  This holds the memory dependence predictor.
 * As memory operations are issued to the IQ, they are also issued to this
 * unit, which then looks up the prediction as to what they are dependent
 * upon.  This unit must be checked prior to a memory operation being able
 * to issue.  Although this is templated, it's somewhat hard to make a generic
 * memory dependence unit.  This one is mostly for store sets; it will be
 * quite limited in what other memory dependence predictions it can also
 * utilize.  Thus this class should be most likely be rewritten for other
 * dependence prediction schemes. //这里有个访存依赖关系预测器，这个单元必须要被检查在load / store 执行前，当每条访存指令被发射到IQ的同时，也能够发射到这个访存依赖关系单元，在每次访存被issue前都要进行监测依赖关系
 */
template <class MemDepPred, class Impl>
class MemDepUnit
{
  protected:
    std::string _name;

  public:
    typedef typename Impl::DynInstPtr DynInstPtr;

    /** Empty constructor. Must call init() prior to using in this case. */
    MemDepUnit();

    /** Constructs a MemDepUnit with given parameters. */
    MemDepUnit(DerivO3CPUParams *params);

    /** Frees up any memory allocated. */
    ~MemDepUnit();

    /** Returns the name of the memory dependence unit. */
    std::string name() const { return _name; }

    /** Initializes the unit with parameters and a thread id. */
    void init(DerivO3CPUParams *params, ThreadID tid);

    /** Registers statistics. */
    void regStats();

    /** Determine if we are drained. */
    bool isDrained() const; //排空，被排空的，这是个状态，而不是正在排空这个状态，因为使用的不是正在进行时。

    /** Perform sanity checks after a drain. */
    void drainSanityCheck() const;  //对排空操作进行稳定性检查

    /** Takes over from another CPU's thread. */
    void takeOverFrom();

    /** Sets the pointer to the IQ. */
    void setIQ(InstructionQueue<Impl> *iq_ptr);

    /** Inserts a memory instruction. */
    void insert(DynInstPtr &inst); 

    /** Inserts a non-speculative memory instruction. */
    void insertNonSpec(DynInstPtr &inst); //加入一个不会推测执行的访存指令，但是什么是不会推测执行的呢？

    /** Inserts a barrier instruction. */
    void insertBarrier(DynInstPtr &barr_inst); //原来memory barrier 的命令是在这里被加入的。是通过访存依赖关系单元完成的。

    /** Indicate that an instruction has its registers ready. */
    void regsReady(DynInstPtr &inst);  //这里是判定访存指令所需的寄存器已经到位。 //一定有函数能够调用这个初始化寄存ready的位置。可能会从两个维度来进行判定，一个是所需的物理寄存器没有存在竞争，一个是所需的依赖的数据计算结果进行判定。 

    /** Indicate that a non-speculative instruction is ready. */
    void nonSpecInstReady(DynInstPtr &inst); // 如果是依赖的访存指令的话，或者说是依赖于regready的访存指令，就是可以进行推测执行的访存指令。

    /** Reschedules an instruction to be re-executed. */
    void reschedule(DynInstPtr &inst); //看了一些重新调度指令似乎就是把全面判定的ready的命令进行排序。

    /** Replays all instructions that have been rescheduled by moving them to
     *  the ready list.
     */
    void replay();  //把所有已经被调度到ready list命令重新replay。

    /** Completes a memory instruction. */
    void completed(DynInstPtr &inst);  //要完成两个不同的命令

    /** Completes a barrier instruction. */
    void completeBarrier(DynInstPtr &inst);

    /** Wakes any dependents of a memory instruction. */
    void wakeDependents(DynInstPtr &inst);  //这个不懂，什么叫做唤醒的

    /** Squashes all instructions up until a given sequence number for a
     *  specific thread.
     */
    void squash(const InstSeqNum &squashed_num, ThreadID tid); // 这里清空所有的指令。除非，给定了一个某个线程的序号。

    /** Indicates an ordering violation between a store and a younger load. */
    void violation(DynInstPtr &store_inst, DynInstPtr &violating_load);  //在一个load / store中声明要给违反顺序。 //也就是给了一个信号，表明违反了访存的顺序。

    /** Issues the given instruction */
    void issue(DynInstPtr &inst);

    /** Debugging function to dump the lists of instructions. */
    void dumpLists();

  private:
    typedef typename std::list<DynInstPtr>::iterator ListIt;

    class MemDepEntry;

    typedef std::shared_ptr<MemDepEntry> MemDepEntryPtr;

    /** Memory dependence entries that track memory operations, marking
     *  when the instruction is ready to execute and what instructions depend
     *  upon it. //就是访存系统的记分板。
     */
    class MemDepEntry {
      public:
        /** Constructs a memory dependence entry. */
        MemDepEntry(DynInstPtr &new_inst)
            : inst(new_inst), regsReady(false), memDepReady(false),
              completed(false), squashed(false)
        {
#ifdef DEBUG
            ++memdep_count;

            DPRINTF(MemDepUnit, "Memory dependency entry created.  "
                    "memdep_count=%i %s\n", memdep_count, inst->pcState());
#endif
        }

        /** Frees any pointers. */
        ~MemDepEntry()
        {
            for (int i = 0; i < dependInsts.size(); ++i) {
                dependInsts[i] = NULL;
            }
#ifdef DEBUG
            --memdep_count;

            DPRINTF(MemDepUnit, "Memory dependency entry deleted.  "
                    "memdep_count=%i %s\n", memdep_count, inst->pcState());
#endif
        }

        /** Returns the name of the memory dependence entry. */
        std::string name() const { return "memdepentry"; }

        /** The instruction being tracked. */
        DynInstPtr inst;

        /** The iterator to the instruction's location inside the list. */
        ListIt listIt;

        /** A vector of any dependent instructions. */
        std::vector<MemDepEntryPtr> dependInsts;

        /** If the registers are ready or not. */
        bool regsReady;
        /** If all memory dependencies have been satisfied. */
        bool memDepReady;
        /** If the instruction is completed. */
        bool completed;
        /** If the instruction is squashed. */
        bool squashed;

        /** For debugging. */
#ifdef DEBUG
        static int memdep_count;
        static int memdep_insert;
        static int memdep_erase;
#endif
    };

    /** Finds the memory dependence entry in the hash map. */
    inline MemDepEntryPtr &findInHash(const DynInstPtr &inst);

    /** Moves an entry to the ready list. */
    inline void moveToReady(MemDepEntryPtr &ready_inst_entry);

    typedef std::unordered_map<InstSeqNum, MemDepEntryPtr, SNHash> MemDepHash;

    typedef typename MemDepHash::iterator MemDepHashIt;

    /** A hash map of all memory dependence entries. */
    MemDepHash memDepHash;

    /** A list of all instructions in the memory dependence unit. */
    std::list<DynInstPtr> instList[Impl::MaxThreads];

    /** A list of all instructions that are going to be replayed. */
    std::list<DynInstPtr> instsToReplay;  //为何要用一个链表去维护，难道不是整体清除，或者是重复，这里的replay不光是squash，而是重新访问，这里和分支预测是不同的，因为分支预测错误，就是执行错了方向而已，但是这些命令是需要被重新执行的。

    /** The memory dependence predictor.  It is accessed upon new
     *  instructions being added to the IQ, and responds by telling
     *  this unit what instruction the newly added instruction is dependent
     *  upon.
     */
    MemDepPred depPred;

    /** Is there an outstanding load barrier that loads must wait on. */
    bool loadBarrier;
    /** The sequence number of the load barrier. */
    InstSeqNum loadBarrierSN;
    /** Is there an outstanding store barrier that loads must wait on. */
    bool storeBarrier;
    /** The sequence number of the store barrier. */
    InstSeqNum storeBarrierSN;

    /** Pointer to the IQ. */
    InstructionQueue<Impl> *iqPtr;

    /** The thread id of this memory dependence unit. */
    int id;

    /** Stat for number of inserted loads. */
    Stats::Scalar insertedLoads;
    /** Stat for number of inserted stores. */
    Stats::Scalar insertedStores;
    /** Stat for number of conflicting loads that had to wait for a store. */
    Stats::Scalar conflictingLoads;
    /** Stat for number of conflicting stores that had to wait for a store. */
    Stats::Scalar conflictingStores;

   /****JKC 2015-11-16 ******/
   //Add for the number of memory replays.
    Stats::Scalar num_replays;
   //End
};

#endif // __CPU_O3_MEM_DEP_UNIT_HH__
