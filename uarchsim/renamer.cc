#include <iostream>
#include <inttypes.h>
#include <cassert>
#include <vector>
using std::vector;
#include <bits/stdc++.h>
#include <math.h>
#include <renamer.h>

using namespace std;

/////////////////////////////////////////////////////////////////////
// This is the constructor function.
// When a renamer object is instantiated, the caller indicates:
// 1. The number of logical registers (e.g., 32).
// 2. The number of physical registers (e.g., 128).
// 3. The maximum number of unresolved branches.
//    Requirement: 1 <= n_branches <= 64.
// 4. The maximum number of active instructions (Active List size).
//
// Tips:
//
// Assert the number of physical registers > number logical registers.
// Assert 1 <= n_branches <= 64.
// Assert n_active > 0.
// Then, allocate space for the primary data structures.
// Then, initialize the data structures based on the knowledge
// that the pipeline is intially empty (no in-flight instructions yet).
/////////////////////////////////////////////////////////////////////
renamer::renamer(uint64_t n_log_regs, uint64_t n_phys_regs, uint64_t n_branches, uint64_t n_active)
{
    //std::cout << "Inputs Passed -- " << n_log_regs << " " << n_phys_regs << " " << n_branches << " " << n_active << '\n';
    assert (n_phys_regs > n_log_regs);
    assert ((n_branches >= 1) && (n_branches <= 64));
    assert (n_active > 0);
    
    initializeRMT(n_log_regs);
    initializeAMT(n_log_regs);
    initializeFreeList(n_phys_regs, n_log_regs);
    initializeActiveList(n_active);
    initializePRF(n_phys_regs);
    initializePRFReadyBits(n_phys_regs);
    initializeCheckPoint(n_branches);
    initializecheckPointBufferCPR(n_phys_regs, n_log_regs, n_branches, n_active);
    //printf("\n* Completed renamer()\n");
    //printDetailedStates();
}

///////////////////
// Rename Stage. //
///////////////////

/////////////////////////////////////////////////////////////////////
// The Rename Stage must stall if there aren't enough free physical
// registers available for renaming all logical destination registers
// in the current rename bundle.
//
// Inputs:
// 1. bundle_dst: number of logical destination registers in
//    current rename bundle
//
// Return value:
// Return "true" (stall) if there aren't enough free physical
// registers to allocate to all of the logical destination registers
// in the current rename bundle.
/////////////////////////////////////////////////////////////////////
bool renamer::stall_reg(uint64_t bundle_dst)
{
    if (bundle_dst > noOfFreeRegistersInFreeList())
    {
        //printf("\n* Completed stall_reg(): Insufficient Free Registers (required=%llu vs available=%llu)\n", bundle_dst, noOfFreeRegistersInFreeList());
        return true;
    }
    else
    {
        //printf("\n* Completed stall_reg(): There are Free Registers (required=%llu vs available=%llu)\n", bundle_dst, noOfFreeRegistersInFreeList());
        return false;
    }
}

/////////////////////////////////////////////////////////////////////
// The Rename Stage must stall if there aren't enough free
// checkpoints for all branches in the current rename bundle.
//
// Inputs:
// 1. bundle_branch: number of branches in current rename bundle
//
// Return value:
// Return "true" (stall) if there aren't enough free checkpoints
// for all branches in the current rename bundle.
/////////////////////////////////////////////////////////////////////
bool renamer::stall_branch(uint64_t bundle_branch)
{
    if (bundle_branch > noOfFreeBranchesInCheckPoint())
    {
        //printf("\n* Completed stall_branch(): Insufficient Checkpoints (required=%llu vs available=%llu)\n", bundle_branch, noOfFreeBranchesInCheckPoint());
        return true;
    }
    else
    {
        //printf("\n* Completed stall_branch(): There are sufficient Checkpoints (required=%llu vs available=%llu)\n", bundle_branch, noOfFreeBranchesInCheckPoint());
        return false;
    }
}

/////////////////////////////////////////////////////////////////////
// This function is used to rename a single source register.
//
// Inputs:
// 1. log_reg: the logical register to rename
//
// Return value: physical register name
/////////////////////////////////////////////////////////////////////
uint64_t renamer::rename_rsrc(uint64_t log_reg)
{
    uint64_t phys_reg = RMT.entry[log_reg];
    inc_usage_counter(phys_reg);
    //printf("\n* Completed rename_rsrc() Renaming of Source Register r%llu to p%llu from RMT\n", log_reg, phys_reg);
    return phys_reg;
}

/////////////////////////////////////////////////////////////////////
// This function is used to rename a single destination register.
//
// Inputs:
// 1. log_reg: the logical register to rename
//
// Return value: physical register name
/////////////////////////////////////////////////////////////////////
uint64_t renamer::rename_rdst(uint64_t log_reg)
{
    unmap(RMT.entry[log_reg]);

    uint64_t phys_reg = popRegisterFromFreeList();
    RMT.entry[log_reg] = phys_reg;
    inc_usage_counter(phys_reg);
    map(phys_reg);
    //PRFReadyBits.entry[log_reg] = 1;
    //printf("* Completed rename_rdst() Renaming of Destination Register r%llu to p%llu in Free List\n", log_reg, phys_reg);
    return phys_reg;
}

/////////////////////////////////////////////////////////////////////
// This function creates a new branch checkpoint.
//
// Inputs: none.
//
// Output:
// 1. The function returns the branch's ID. When the branch resolves,
//    its ID is passed back to the renamer via "resolve()" below.
//
// Tips:
//
// Allocating resources for the branch (a GBM bit and a checkpoint):
// * Find a free bit -- i.e., a '0' bit -- in the GBM. Assert that
//   a free bit exists: it is the user's responsibility to avoid
//   a structural hazard by calling stall_branch() in advance.
// * Set the bit to '1' since it is now in use by the new branch.
// * The position of this bit in the GBM is the branch's ID.
// * Use the branch checkpoint that corresponds to this bit.
// 
// The branch checkpoint should contain the following:
// 1. Shadow Map Table (checkpointed Rename Map Table)
// 2. checkpointed Free List head pointer and its phase bit
// 3. checkpointed GBM
/////////////////////////////////////////////////////////////////////
/*uint64_t renamer::checkpoint()
{
    uint64_t branch_ID = getFreeBitGBM() - 1;
    assert ((branch_ID + 1) <= totalUnresolvedBranches);
    //printf("\nGBM=%llu, First Free Bit from the right=%llu", GBM, branch_ID);
    GBM = GBM | (1 << branch_ID);
    //printf(" and newGBM=%llu\n", GBM);
    checkPoints.RMT[branch_ID] = RMT;
    checkPoints.head[branch_ID] = freeList.head;
    checkPoints.headPhase[branch_ID] = freeList.headPhase;
    checkPoints.GBM[branch_ID] = GBM;
    assert (checkPoints.valid[branch_ID] == 0);
    checkPoints.valid[branch_ID] = 1;
    //printf("* Completed checkpoint() current state. branch_ID=%llu\n", branch_ID);
    //printDetailedStates();
    return branch_ID;
}*/
// P4 - checkpoint()
void renamer::checkpoint()
{
    // Asserting that the checkpoint at checkPointBufferCPR's tail is not valid (empty)
    assert(checkPointBufferCPR.CPR[checkPointBufferCPR.tail].valid == false);

    checkPointBufferCPR.RMT[checkPointBufferCPR.tail] = RMT;
    
    for (uint64_t i = 0; i < RMT.size; i++)
    {
        // RMT.entry[i] contains Physical Reg number and we increament all Phy Regs
        // which are currently mapped/present in RMT to Logical Registers
        inc_usage_counter(RMT.entry[i]);
    }

    checkPointBufferCPR.CPR[checkPointBufferCPR.tail] = CPR;

    if (checkPointBufferCPR.tail < checkPointBufferCPR.size - 1)
    {
        checkPointBufferCPR.tail++;
    }
    else
    {
        checkPointBufferCPR.tail = 0;
        checkPointBufferCPR.tailPhase  = !(checkPointBufferCPR.tailPhase);
    }

    //printf("* Completed checkpoint() current state. branch_ID=%llu\n", branch_ID);
}

// P4-D get_checkpoint_ID
unsigned int renamer::get_checkPoint_ID(bool load, bool store, bool branch, bool amo, bool csr)
{
    unsigned int latest_checkpoint_ID = 0;

    if (checkPointBufferCPR.tail == 0)
    {
        latest_checkpoint_ID = checkPointBufferCPR.size - 1;
    }
    else
    {
        latest_checkpoint_ID = checkPointBufferCPR.tail - 1;
    }

    assert(checkPointBufferCPR.CPR[latest_checkpoint_ID].valid == true);

    checkPointBufferCPR.CPR[latest_checkpoint_ID].loadFlag = load;
    checkPointBufferCPR.CPR[latest_checkpoint_ID].storeFlag = store;
    checkPointBufferCPR.CPR[latest_checkpoint_ID].branchFlag = branch;
    checkPointBufferCPR.CPR[latest_checkpoint_ID].amoFlag = amo;
    checkPointBufferCPR.CPR[latest_checkpoint_ID].csrFlag = csr;
    checkPointBufferCPR.CPR[latest_checkpoint_ID].exceptionBit = 0;

    if (load == true)
    {
        checkPointBufferCPR.CPR[latest_checkpoint_ID].load_count++;
    }
    if (store == true)
    {
        checkPointBufferCPR.CPR[latest_checkpoint_ID].store_count++;
    }
    if (branch == true)
    {
        checkPointBufferCPR.CPR[latest_checkpoint_ID].branch_count++;
    }

    return latest_checkpoint_ID;
}

//P4-D free_checkpoint()
void renamer::free_checkpoint()
{
    uint64_t oldest_checkpoint_ID = checkPointBufferCPR.head;
    assert(checkPointBufferCPR.CPR[oldest_checkpoint_ID].uncomp_instr == 0);
    assert(checkPointBufferCPR.CPR[oldest_checkpoint_ID].valid == true);
    checkPointBufferCPR.CPR[oldest_checkpoint_ID].valid = false;

    // -------------------------------------------------------------------------------------- //
    // P4 - TODO
    // I think since we need to free this checkpoint we need to decreament the usage counters
    // in CPRs of all future checkpoints (head+1 to tail-1) and Current CPR
    int checkpoint_ID = oldest_checkpoint_ID;
    checkpoint_ID = nextIndexCPR(checkpoint_ID);
    while (checkpoint_ID != checkPointBufferCPR.tail)   // All newer CPRs
    {
        for (uint64_t i = 0; i < RMT.size; i++)
        {
            checkPointBufferCPR.CPR[checkpoint_ID].usageCounter[checkPointBufferCPR.RMT[oldest_checkpoint_ID].entry[i]]--;
        }
        checkpoint_ID = nextIndexCPR(checkpoint_ID);
    }
    // Current CPR --- You need to do this once because you increamented the counters when you checkpointed the CPR
    for (uint64_t i = 0; i < RMT.size; i++)
    {
        dec_usage_counter(checkPointBufferCPR.RMT[oldest_checkpoint_ID].entry[i]);
    }
    // ------------------------------------------------------------------------------------- //

    if(checkPointBufferCPR.head == checkPointBufferCPR.size -1) {
        checkPointBufferCPR.headPhase = !(checkPointBufferCPR.headPhase);
        checkPointBufferCPR.head = 0;
    }
    else if(checkPointBufferCPR.head < checkPointBufferCPR.size -1) {
        checkPointBufferCPR.head++;
    }
}

/*uint64_t renamer::checkpoint()
{
    uint64_t branch_ID = getFreeBitGBM() - 1;
    assert ((branch_ID + 1) <= totalUnresolvedBranches);
    //printf("\nGBM=%llu, First Free Bit from the right=%llu", GBM, branch_ID);
    GBM = GBM | (1 << branch_ID);
    //printf(" and newGBM=%llu\n", GBM);
    checkPoints.RMT[branch_ID] = RMT;
    checkPoints.head[branch_ID] = freeList.head;
    checkPoints.headPhase[branch_ID] = freeList.headPhase;
    checkPoints.GBM[branch_ID] = GBM;
    assert (checkPoints.valid[branch_ID] == 0);
    checkPoints.valid[branch_ID] = 1;
    //printf("* Completed checkpoint() current state. branch_ID=%llu\n", branch_ID);
    //printDetailedStates();
    return branch_ID;
}*/

bool renamer::stall_checkpoint(uint64_t bundle_chkpts)
{
    if (bundle_chkpts > noOfAvailableCheckpointsCPR())
    {
        //printf("\n* Completed stall_dispatch(): Insufficient Active List entries (required=%llu vs available=%llu)\n", bundle_inst, noOfFreeEntriesInActiveList());
        return true;
    }
    else
    {
        //printf("\n* Completed stall_dispatch(): sufficient Active List entries (required=%llu vs available=%llu)\n", bundle_inst, noOfFreeEntriesInActiveList());
        return false;
    }
}

/////////////////////
// Dispatch Stage. //
/////////////////////

/////////////////////////////////////////////////////////////////////
// The Dispatch Stage must stall if there are not enough free
// entries in the Active List for all instructions in the current
// dispatch bundle.
//
// Inputs:
// 1. bundle_inst: number of instructions in current dispatch bundle
//
// Return value:
// Return "true" (stall) if the Active List does not have enough
// space for all instructions in the dispatch bundle.
/////////////////////////////////////////////////////////////////////
bool renamer::stall_dispatch(uint64_t bundle_inst)
{
    if (bundle_inst > noOfFreeEntriesInActiveList())
    {
        //printf("\n* Completed stall_dispatch(): Insufficient Active List entries (required=%llu vs available=%llu)\n", bundle_inst, noOfFreeEntriesInActiveList());
        return true;
    }
    else
    {
        //printf("\n* Completed stall_dispatch(): sufficient Active List entries (required=%llu vs available=%llu)\n", bundle_inst, noOfFreeEntriesInActiveList());
        return false;
    }
}

/////////////////////////////////////////////////////////////////////
// This function dispatches a single instruction into the Active
// List.
//
// Inputs:
// 1. dest_valid: If 'true', the instr. has a destination register,
//    otherwise it does not. If it does not, then the log_reg and
//    phys_reg inputs should be ignored.
// 2. log_reg: Logical register number of the instruction's
//    destination.
// 3. phys_reg: Physical register number of the instruction's
//    destination.
// 4. load: If 'true', the instr. is a load, otherwise it isn't.
// 5. store: If 'true', the instr. is a store, otherwise it isn't.
// 6. branch: If 'true', the instr. is a branch, otherwise it isn't.
// 7. amo: If 'true', this is an atomic memory operation.
// 8. csr: If 'true', this is a system instruction.
// 9. PC: Program counter of the instruction.
//
// Return value:
// Return the instruction's index in the Active List.
//
// Tips:
//
// Before dispatching the instruction into the Active List, assert
// that the Active List isn't full: it is the user's responsibility
// to avoid a structural hazard by calling stall_dispatch()
// in advance.
/////////////////////////////////////////////////////////////////////
uint64_t renamer::dispatch_inst(bool dest_valid, uint64_t log_reg, uint64_t phys_reg, 
                                    bool load, bool store, bool branch, bool amo, bool csr, uint64_t PC)
{
    TS_activeListEntries entry;
    (dest_valid == true) ? entry.destinationFlag = 1 : entry.destinationFlag = 0;
    entry.logicalRegisterNumber = log_reg;
    entry.physicalRegisterNumber = phys_reg;
    
    entry.completedBit = 0;
    entry.exceptionBit = 0;
    entry.loadViolationBit = 0;
    entry.branchMispredictionBit = 0;
    entry.valueMispredictionBit = 0;

    (load == true) ? entry.loadFlag = 1 : entry.loadFlag = 0;
    (store == true) ? entry.storeFlag = 1 : entry.storeFlag = 0;
    (branch == true) ? entry.branchFlag = 1 : entry.branchFlag = 0;
    (amo == true) ? entry.amoFlag = 1 : entry.amoFlag = 0;
    (csr == true) ? entry.csrFlag = 1 : entry.csrFlag = 0;

    entry.PC = PC;

    assert ((activeList.head != activeList.tail) || (activeList.headPhase == activeList.tailPhase));

    activeList.entry[activeList.tail] = entry;
    uint64_t indexInActiveList = activeList.tail;
    if (activeList.tail < activeList.size - 1)
    {
        activeList.tail++;
    }
    else
    {
        assert (activeList.tailPhase == activeList.headPhase);

        activeList.tail = 0;
        activeList.tailPhase  = !(activeList.tailPhase);
    }

    //printf("\n* Completed dispatch_inst() inserting entry in Active List at AL_index=%llu\n", indexInActiveList);
    //printDetailedStates();
    return indexInActiveList;
}

/////////////////////////////////////////////////////////////////////
// Test the ready bit of the indicated physical register.
// Returns 'true' if ready.
/////////////////////////////////////////////////////////////////////
bool renamer::is_ready(uint64_t phys_reg)
{
    //printf("\n* Completed is_ready() for PRF Ready Bit entry at p%llu value=%llu\n", phys_reg, PRFReadyBits.entry[phys_reg]);
    return PRFReadyBits.entry[phys_reg];
}

/////////////////////////////////////////////////////////////////////
// Clear the ready bit of the indicated physical register.
/////////////////////////////////////////////////////////////////////
void renamer::clear_ready(uint64_t phys_reg)
{
    PRFReadyBits.entry[phys_reg] = 0;
    //printf("\n* Completed clear_ready() to PRF Ready Bit entry at p%llu value=%llu\n", phys_reg, PRFReadyBits.entry[phys_reg]);
}

/////////////////////////////////////////////////////////////
// Functions related to the RegisterRead and Execute Stage //
/////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////
// Return the contents (value) of the indicated physical register.
/////////////////////////////////////////////////////////////////////
uint64_t renamer::read(uint64_t phys_reg)
{
    //printf("\n* Completed read() for PRF entry at p%llu value=%llu\n", phys_reg, PRF.entry[phys_reg]);
    return PRF.entry[phys_reg];
}
/////////////////////////////////////////////////////////////////////
// Set the ready bit of the indicated physical register.
/////////////////////////////////////////////////////////////////////
void renamer::set_ready(uint64_t phys_reg)
{
    PRFReadyBits.entry[phys_reg] = 1;
    //printf("\n* Completed set_ready() to PRF Ready Bit entry at p%llu value=%llu\n", phys_reg, PRFReadyBits.entry[phys_reg]);
}

//////////////////////////////////////////
// Functions related to Writeback Stage //
//////////////////////////////////////////

/////////////////////////////////////////////////////////////////////
// Write a value into the indicated physical register.
/////////////////////////////////////////////////////////////////////
void renamer::write(uint64_t phys_reg, uint64_t value)
{
    //printf("\n* Completed write() to PRF entry at p%llu value=%llu\n", phys_reg, value);
    PRF.entry[phys_reg] = value;
}

/////////////////////////////////////////////////////////////////////
// Set the completed bit of the indicated entry in the Active List.
/////////////////////////////////////////////////////////////////////
/*void renamer::set_complete(uint64_t AL_index)
{
    activeList.entry[AL_index].completedBit = 1;
    //printf("\n* Completed set_complete() AL_index=%llu\n", AL_index);
}*/
//P4-D
void renamer::set_complete(uint64_t checkpoint_ID){
    checkPointBufferCPR.CPR[checkpoint_ID].uncomp_instr--;
}

/////////////////////////////////////////////////////////////////////
// This function is for handling branch resolution.
//
// Inputs:
// 1. AL_index: Index of the branch in the Active List.
// 2. branch_ID: This uniquely identifies the branch and the
//    checkpoint in question.  It was originally provided
//    by the checkpoint function.
// 3. correct: 'true' indicates the branch was correctly
//    predicted, 'false' indicates it was mispredicted
//    and recovery is required.
//
// Outputs: none.
//
// Tips:
//
// While recovery is not needed in the case of a correct branch,
// some actions are still required with respect to the GBM and
// all checkpointed GBMs:
// * Remember to clear the branch's bit in the GBM.
// * Remember to clear the branch's bit in all checkpointed GBMs.
//
// In the case of a misprediction:
// * Restore the GBM from the branch's checkpoint. Also make sure the
//   mispredicted branch's bit is cleared in the restored GBM,
//   since it is now resolved and its bit and checkpoint are freed.
// * You don't have to worry about explicitly freeing the GBM bits
//   and checkpoints of branches that are after the mispredicted
//   branch in program order. The mere act of restoring the GBM
//   from the checkpoint achieves this feat.
// * Restore the RMT using the branch's checkpoint.
// * Restore the Free List head pointer and its phase bit,
//   using the branch's checkpoint.
// * Restore the Active List tail pointer and its phase bit
//   corresponding to the entry after the branch's entry.
//   Hints:
//   You can infer the restored tail pointer from the branch's
//   AL_index. You can infer the restored phase bit, using
//   the phase bit of the Active List head pointer, where
//   the restored Active List tail pointer is with respect to
//   the Active List head pointer, and the knowledge that the
//   Active List can't be empty at this moment (because the
//   mispredicted branch is still in the Active List).
// * Do NOT set the branch misprediction bit in the Active List.
//   (Doing so would cause a second, full squash when the branch
//   reaches the head of the Active List. We don’t want or need
//   that because we immediately recover within this function.)
/////////////////////////////////////////////////////////////////////
void renamer::resolve(uint64_t AL_index, uint64_t branch_ID, bool correct)
{
    uint64_t branch_ID_bit_mask = pow(2,branch_ID);
    if (correct == true)
    {
        GBM = GBM & (~branch_ID_bit_mask);

        checkPoints.valid[branch_ID] = 0;
        //std::cout << '\n';
        for (uint64_t i = 0; i < checkPoints.size; i++)
        {
            if (checkPoints.valid[i] == 1)
            {
                //std::cout << "noldGBM=" << std::bitset<32>(checkPoints.GBM[i]) << ", ";
                checkPoints.GBM[i] = checkPoints.GBM[i] & (~branch_ID_bit_mask);
                //std::cout << "newGBM=" << std::bitset<32>(checkPoints.GBM[i]) << '\n';
            }
        }

        //printf("* Completed resolve() Branch Resolution: Correct Branch Path branch_ID=%llu & AL_index=%llu\n", branch_ID, AL_index);
        assert(activeList.entry[AL_index].branchFlag == 1);
    }
    else
    {   
        GBM = checkPoints.GBM[branch_ID];
        //std::cout << '\n';
        //std::cout << "noldGBM=" << std::bitset<32>(GBM) << ", ";
        GBM = GBM & (~branch_ID_bit_mask);
        //std::cout << "newGBM=" << std::bitset<32>(GBM) << '\n';
        
        //std::cout << '\n';
        for (uint64_t i = 0; i < checkPoints.size; i++)
        {
            if (checkPoints.valid[i] == 1)
            {
                if ((checkPoints.GBM[i] & branch_ID_bit_mask) != 0)
                {
                    //std::cout << checkPoints.GBM[i] << " - " << (checkPoints.GBM[i] & branch_ID_bit_mask) << "\n";
                    checkPoints.valid[i] = 0;
                    uint64_t indi_branch_ID_bit_mask = pow(2,i);
                    GBM = GBM & (~indi_branch_ID_bit_mask);
                }
            }
        }

        RMT = checkPoints.RMT[branch_ID];
        freeList.head = checkPoints.head[branch_ID];
        freeList.headPhase = checkPoints.headPhase[branch_ID];

        //initializePRFReadyBits(PRFReadyBits.size);

        //activeList.entry[AL_index].branchMispredictionBit = 1;
        (AL_index == (activeList.size - 1) ) ? activeList.tail = 0 : activeList.tail = AL_index + 1;
        
        if (activeList.tail > activeList.head)
        {
            activeList.tailPhase = activeList.headPhase;
        }
        else
        {
            activeList.tailPhase = !(activeList.headPhase);
        }
        //printf("* Completed resolve() Branch Resolution: Incorrect Prediction branch_ID=%llu & AL_index=%llu\n", branch_ID, AL_index);
    }
    //printDetailedStates();
}

//////////////////////////////////////////
// Functions related to Retire Stage.   //
//////////////////////////////////////////

///////////////////////////////////////////////////////////////////
// This function allows the caller to examine the instruction at the head
// of the Active List.
//
// Input arguments: none.
//
// Return value:
// * Return "true" if the Active List is NOT empty, i.e., there
//   is an instruction at the head of the Active List.
// * Return "false" if the Active List is empty, i.e., there is
//   no instruction at the head of the Active List.
//
// Output arguments:
// Simply return the following contents of the head entry of
// the Active List.  These are don't-cares if the Active List
// is empty (you may either return the contents of the head
// entry anyway, or not set these at all).
// * completed bit
// * exception bit
// * load violation bit
// * branch misprediction bit
// * value misprediction bit
// * load flag (indicates whether or not the instr. is a load)
// * store flag (indicates whether or not the instr. is a store)
// * branch flag (indicates whether or not the instr. is a branch)
// * amo flag (whether or not instr. is an atomic memory operation)
// * csr flag (whether or not instr. is a system instruction)
// * program counter of the instruction
/////////////////////////////////////////////////////////////////////

//bool renamer::precommit(bool &completed, bool &exception, bool &load_viol, bool &br_misp, bool &val_misp,
//	                bool &load, bool &store, bool &branch, bool &amo, bool &csr, uint64_t &PC)
//{
//    if ((activeList.head == activeList.tail) && (activeList.headPhase == activeList.tailPhase))
//    {
//        //printf("\n* Completed precommit(): empty Active list\n");
//        return false;
//    }
//    else
//    {
//        TS_activeListEntries headEntry = activeList.entry[activeList.head];
//        completed = headEntry.completedBit;
//        exception = headEntry.exceptionBit;
//        load_viol = headEntry.loadViolationBit;
//        br_misp = headEntry.branchMispredictionBit;
//        val_misp = headEntry.valueMispredictionBit;
//        load = headEntry.loadFlag;
//        store = headEntry.storeFlag;
//        branch = headEntry.branchFlag;
//        amo = headEntry.amoFlag;
//        csr = headEntry.csrFlag;
//        PC = headEntry.PC;
//        //printf("\n* Completed precommit(): passed Active List Head entry at AL_index=%llu\n", activeList.head);
//        //printDetailedStates();
//        return true;
//    }
//}
// P4-D
bool renamer::precommit(uint64_t &chkpt_id, uint64_t &num_loads, uint64_t &num_stores, uint64_t &num_branches, bool &amo, bool &csr, bool &exception)
{
    //assert(checkPointBufferCPR.head <= chkpt_id);
    //assert(checkPointBufferCPR.tail > chkpt_id);

    TS_CPREntries chkpnt = checkPointBufferCPR.CPR[chkpt_id];
    num_loads = chkpnt.load_count;
    num_stores = chkpnt.store_count;
    num_branches = chkpnt.branch_count;
    amo = chkpnt.amoFlag;
    csr = chkpnt.csrFlag;
    exception = chkpnt.exceptionBit;

    uint64_t oldest_checkpoint_ID = checkPointBufferCPR.head;
    uint64_t next_oldest_checkpoint_ID;
    if (oldest_checkpoint_ID < checkPointBufferCPR.size - 1)
    {
        next_oldest_checkpoint_ID = oldest_checkpoint_ID + 1;
    }
    else
    {
        next_oldest_checkpoint_ID = 0;
    }

    if (checkPointBufferCPR.CPR[next_oldest_checkpoint_ID].valid==true && checkPointBufferCPR.CPR[oldest_checkpoint_ID].uncomp_instr==0)
    {
        return true;
    }
    else
    {
        return false;
    }
}

/////////////////////////////////////////////////////////////////////
// This function commits the instruction at the head of the Active List.
//
// Tip (optional but helps catch bugs):
// Before committing the head instruction, assert that it is valid to
// do so (use assert() from standard library). Specifically, assert
// that all of the following are true:
// - there is a head instruction (the active list isn't empty)
// - the head instruction is completed
// - the head instruction is not marked as an exception
// - the head instruction is not marked as a load violation
// It is the caller's (pipeline's) duty to ensure that it is valid
// to commit the head instruction BEFORE calling this function
// (by examining the flags returned by "precommit()" above).
// This is why you should assert() that it is valid to commit the
// head instruction and otherwise cause the simulator to exit.
/////////////////////////////////////////////////////////////////////
//void renamer::commit()
//{
//    uint64_t AL_index = activeList.head;
//
//    assert ((AL_index != activeList.tail) || (activeList.headPhase != activeList.tailPhase));
//    assert(activeList.entry[AL_index].completedBit == 1);
//    assert(activeList.entry[AL_index].exceptionBit != 1);
//    assert(activeList.entry[AL_index].loadViolationBit != 1);
//
//    if (activeList.entry[AL_index].destinationFlag == true)
//    {
//        uint64_t logicalRegisterNumber = activeList.entry[AL_index].logicalRegisterNumber;
//        
//        uint64_t freeReg = AMT.entry[logicalRegisterNumber];
//        AMT.entry[logicalRegisterNumber] = activeList.entry[AL_index].physicalRegisterNumber;
//        //PRFReadyBits.entry[logicalRegisterNumber] = 1;
//        
//        freeList.entry[freeList.tail] = freeReg;
//        //PRFReadyBits.entry[freeReg] = 0;
//        if (freeList.tail < freeList.size - 1)
//		{
//			freeList.tail++;
//		}
//		else
//		{
//			assert (freeList.tailPhase == freeList.headPhase);
//
//			freeList.tail = 0;
//			freeList.tailPhase  = !(freeList.tailPhase);
//		}
//    }
//
//    if (activeList.head < activeList.size - 1)
//    {
//        activeList.head++;
//    }
//    else
//    {
//        assert (activeList.tailPhase != activeList.headPhase);
//
//        activeList.head = 0;
//        activeList.headPhase  = !(activeList.headPhase);
//    }
//
//    //printf("\n* Completed commit() Retire of the head of Active List at AL_index=%llu\n", AL_index);
//    //printDetailedStates();
//}
// P4-D
void renamer::commit(uint64_t log_reg) {
    uint64_t oldest_checkpoint_ID = checkPointBufferCPR.head;
    // Num of Logical Regs = RMT.size
    assert(log_reg < RMT.size);
    uint64_t phys_reg = checkPointBufferCPR.RMT[oldest_checkpoint_ID].entry[log_reg];
    dec_usage_counter(phys_reg);  // Current CPR
}

//////////////////////////////////////////////////////////////////////
// Squash the renamer class.
//
// Squash all instructions in the Active List and think about which
// sructures in your renamer class need to be restored, and how.
//
// After this function is called, the renamer should be rolled-back
// to the committed state of the machine and all renamer state
// should be consistent with an empty pipeline.
/////////////////////////////////////////////////////////////////////

//P4-D
void renamer::squash()
{
    uint64_t oldest_checkpoint_ID = checkPointBufferCPR.head;
    RMT = checkPointBufferCPR.RMT[oldest_checkpoint_ID];
    
    //rollbackUnmappedandUsagebits(oldest_checkpoint_ID);
    CPR = checkPointBufferCPR.CPR[oldest_checkpoint_ID];

    if (oldest_checkpoint_ID < checkPointBufferCPR.size - 1)
    {
        checkPointBufferCPR.tail = checkPointBufferCPR.head + 1;
        checkPointBufferCPR.tailPhase = checkPointBufferCPR.headPhase;
    }
    else
    {
        checkPointBufferCPR.tail = 0;
        checkPointBufferCPR.tailPhase = !(checkPointBufferCPR.headPhase);
    }

    //GBM = 0;
    //activeList.tail = activeList.head;
    //activeList.tailPhase = activeList.headPhase;

    // P4 TODO - FREELIST recovery from Unmapped Bit in oldest CPR checkpoint
    //freeList.head = freeList.tail;
    //freeList.headPhase = !(freeList.tailPhase);

    uint64_t numFreeRegs = 0;
    for (uint64_t i = 0; i < CPR.size; i++)
	{
        if (CPR.unmappedBit[i] == 0)
        {
            freeList.entry[numFreeRegs] = i;
            numFreeRegs++;
        }
    }

    assert (numFreeRegs == freeList.size);

    freeList.head = 0;
    freeList.headPhase = 0;
    freeList.tail = 0;
    freeList.tailPhase = 1;

    //initializeCheckPointValues();
    //initializePRFReadyBits(PRFReadyBits.size);

    //printf("\n* Completed squash() of the pipeline\n");
    //printDetailedStates();
}

//////////////////////////////////////////
// Functions not tied to specific stage //
//////////////////////////////////////////
/*void renamer::set_exception(uint64_t AL_index)
{
    activeList.entry[AL_index].exceptionBit = 1;
    //printf("\n* Completed set_exception() AL_index=%llu\n", AL_index);
}*/

void renamer::set_exception(uint64_t checkpoint_ID){
    checkPointBufferCPR.CPR[checkpoint_ID].exceptionBit = 1;
}

void renamer::set_load_violation(uint64_t AL_index)
{
    activeList.entry[AL_index].loadViolationBit = 1;
    //printf("\n* Completed set_load_violation() AL_index=%llu\n", AL_index);
}

void renamer::set_branch_misprediction(uint64_t AL_index)
{
    activeList.entry[AL_index].branchMispredictionBit = 1;
    //printf("\n* Completed set_branch_misprediction() AL_index=%llu\n", AL_index);
}

void renamer::set_value_misprediction(uint64_t AL_index)
{
    activeList.entry[AL_index].valueMispredictionBit = 1;
    //printf("\n* Completed set_value_misprediction() AL_index=%llu\n", AL_index);
}

/////////////////////////////////////////////////////////////////////
// Query the exception bit of the indicated entry in the Active List.
/////////////////////////////////////////////////////////////////////
bool renamer::get_exception(uint64_t AL_index)
{
    //printf("\n* Completed get_exception(): passed exceptionBit AL_index=%llu\n", AL_index);
    return activeList.entry[AL_index].exceptionBit;
}
