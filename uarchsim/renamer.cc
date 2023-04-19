#include <iostream>
#include <inttypes.h>
#include <cassert>
#include <vector>
using std::vector;
#include <bits/stdc++.h>
#include <math.h>
#include <renamer.h>

using namespace std;

renamer::renamer(uint64_t n_log_regs, uint64_t n_phys_regs, uint64_t n_branches, uint64_t n_active)
{
    //std::cout << "Inputs Passed -- " << n_log_regs << " " << n_phys_regs << " " << n_branches << " " << n_active << '\n';
    assert (n_phys_regs > n_log_regs);
    assert ((n_branches >= 1) && (n_branches <= 64));
    assert (n_active > 0);
    
    initializeRMT(n_log_regs);
    initializeFreeList(n_phys_regs, n_log_regs);
    initializePRF(n_phys_regs);
    initializePRFReadyBits(n_phys_regs);
    initializecheckPointBuffer(n_phys_regs, n_log_regs, n_branches, n_active);
    //printf("\n* Completed renamer()\n");
    //printDetailedStates();
}

///////////////////
// Rename Stage. //
///////////////////
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
// This function is used to rename a single source register.
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
/////////////////////////////////////////////////////////////////////
// P4 - checkpoint()
void renamer::checkpoint()
{
    // Asserting that the checkpoint at checkPointBuffer's tail is not valid (empty)
    assert(checkPointBuffer.valid[checkPointBuffer.tail] == false);
    
    checkPointBuffer.valid[checkPointBuffer.tail] == true;
    checkPointBuffer.RMT[checkPointBuffer.tail] = RMT;
    checkPointBuffer.CPR[checkPointBuffer.tail] = CPR;
    
    for (uint64_t i = 0; i < RMT.size; i++)
    {
        // RMT.entry[i] contains Physical Reg number and we increament all Phy Regs
        // which are currently mapped/present in RMT to Logical Registers
        inc_usage_counter(RMT.entry[i]);
    }

    checkPointBuffer.tail = (checkPointBuffer.tail + 1) % checkPointBuffer.size;
    if (checkPointBuffer.tail ==  0)
        checkPointBuffer.tailPhase  = !(checkPointBuffer.tailPhase);

    //printf("* Completed checkpoint() current state. branch_ID=%llu\n", branch_ID);
}

// P4-D get_chkpt_id
unsigned int renamer::get_chkpt_id(bool load, bool store, bool branch, bool amo, bool csr)
{
    unsigned int latest_chkpt_id = 0;

    if (checkPointBuffer.tail == 0)
    {
        latest_chkpt_id = checkPointBuffer.size - 1;
    }
    else
    {
        latest_chkpt_id = checkPointBuffer.tail - 1;
    }

    assert(checkPointBuffer.valid[latest_chkpt_id] == true);

    checkPointBuffer.CPR[latest_chkpt_id].loadFlag = load;
    checkPointBuffer.CPR[latest_chkpt_id].storeFlag = store;
    checkPointBuffer.CPR[latest_chkpt_id].branchFlag = branch;
    checkPointBuffer.CPR[latest_chkpt_id].amoFlag = amo;
    checkPointBuffer.CPR[latest_chkpt_id].csrFlag = csr;
    checkPointBuffer.CPR[latest_chkpt_id].exceptionBit = 0;

    if (load == true)
    {
        checkPointBuffer.CPR[latest_chkpt_id].load_count++;
    }
    if (store == true)
    {
        checkPointBuffer.CPR[latest_chkpt_id].store_count++;
    }
    if (branch == true)
    {
        checkPointBuffer.CPR[latest_chkpt_id].branch_count++;
    }

    return latest_chkpt_id;
}

//P4-D free_checkpoint()
void renamer::free_checkpoint()
{
    uint64_t oldest_chkpt_id = checkPointBuffer.head;
    assert(checkPointBuffer.CPR[oldest_chkpt_id].uncomp_instr == 0);
    assert(checkPointBuffer.valid[oldest_chkpt_id] == true);
    checkPointBuffer.valid[oldest_chkpt_id] = false;

    // -------------------------------------------------------------------------------------- //
    // P4 - TODO
    // I think since we need to free this checkpoint we need to decreament the usage counters
    // in CPRs of all future checkpoints (head+1 to tail-1) and Current CPR
    int chkpt_id = oldest_chkpt_id;
    chkpt_id = (chkpt_id + 1) % checkPointBuffer.size;
    while (chkpt_id != checkPointBuffer.tail)   // All newer CPRs
    {
        for (uint64_t i = 0; i < RMT.size; i++)
        {
            checkPointBuffer.CPR[chkpt_id].usageCounter[checkPointBuffer.RMT[oldest_chkpt_id].entry[i]]--;
        }
        chkpt_id = (chkpt_id + 1) % checkPointBuffer.size;
    }
    // Current CPR --- You need to do this once because you increamented the counters when you checkpointed the CPR
    for (uint64_t i = 0; i < RMT.size; i++)
    {
        dec_usage_counter(checkPointBuffer.RMT[oldest_chkpt_id].entry[i]);
    }
    // ------------------------------------------------------------------------------------- //

    checkPointBuffer.head = (checkPointBuffer.head + 1) % checkPointBuffer.size;
    if(checkPointBuffer.head == 0)
        checkPointBuffer.headPhase = !(checkPointBuffer.headPhase);
    
    assert (checkPointBuffer.head != checkPointBuffer.tail);
}

bool renamer::stall_checkpoint(uint64_t bundle_chkpts)
{
    if (bundle_chkpts > noOfFreeCheckpoints())
    {
        //printf("\n* Completed stall_checkpoint(): Insufficient Active List entries (required=%llu vs available=%llu)\n", bundle_inst, noOfFreeEntriesInActiveList());
        return true;
    }
    else
    {
        //printf("\n* Completed stall_checkpoint(): sufficient Active List entries (required=%llu vs available=%llu)\n", bundle_inst, noOfFreeEntriesInActiveList());
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
    dec_usage_counter(phys_reg);
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
    dec_usage_counter(phys_reg);
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
void renamer::set_complete(uint64_t chkpt_id) {
    checkPointBuffer.CPR[chkpt_id].uncomp_instr--;
}

/////////////////////////////////////////////////////////////////////
// This function is for handling branch resolution.
/////////////////////////////////////////////////////////////////////
//void renamer::resolve(uint64_t AL_index, uint64_t branch_ID, bool correct)
uint64_t renamer::rollback(uint64_t chkpt_id, bool next, uint64_t &total_loads, uint64_t &total_stores, uint64_t &total_branches)
{
    if (next == true)
    {
        chkpt_id = (chkpt_id + 1) % checkPointBuffer.size;
    }

    assert (checkPointBuffer.valid[chkpt_id] == true);

    RMT = checkPointBuffer.RMT[chkpt_id];
    
    checkPointBuffer.CPR[chkpt_id].branchFlag = 0;
    checkPointBuffer.CPR[chkpt_id].amoFlag = 0;
    checkPointBuffer.CPR[chkpt_id].csrFlag = 0;
    checkPointBuffer.CPR[chkpt_id].exceptionBit = 0;
    checkPointBuffer.CPR[chkpt_id].uncomp_instr = 0;
    checkPointBuffer.CPR[chkpt_id].load_count = 0;
    checkPointBuffer.CPR[chkpt_id].store_count = 0;
    checkPointBuffer.CPR[chkpt_id].branch_count = 0;
    checkPointBuffer.CPR[chkpt_id].loadFlag = 0;
    checkPointBuffer.CPR[chkpt_id].storeFlag = 0;
    CPR = checkPointBuffer.CPR[chkpt_id];

    uint64_t squash_mask = 1;
    uint64_t youngest_chkpt_id = 0;
    (checkPointBuffer.tail == 0) ? youngest_chkpt_id = checkPointBuffer.size - 1 : youngest_chkpt_id = checkPointBuffer.tail - 1;
    uint64_t b = youngest_chkpt_id - chkpt_id;
    for (uint64_t i = 0; i < youngest_chkpt_id; i++)
    {
        squash_mask << 1;
        if (b != 0)
        {
            squash_mask = squash_mask | 1;
            b--;
        }
    }

    // rollback chkpt_id to newest checkpoint - clear
    uint64_t id = (id + 1) % checkPointBuffer.size;
    while (id != checkPointBuffer.tail)
    {
        checkPointBuffer.valid[id] = false;
        id = (id + 1) % checkPointBuffer.size;
    }

    // Oldest checkpoint to chkpt_id - sum counters
    id = checkPointBuffer.head;
    while (id != id)
    {
        total_loads += checkPointBuffer.CPR[id].load_count;
        total_stores += checkPointBuffer.CPR[id].store_count;
        total_branches += checkPointBuffer.CPR[id].branch_count;
        id = (id + 1) % checkPointBuffer.size;
    }

    // rollback tail to rollback chkpt_id
    checkPointBuffer.tail = chkpt_id;
    if (chkpt_id < checkPointBuffer.head)
    {
        checkPointBuffer.tailPhase = !(checkPointBuffer.headPhase);
    }
    else
    {
        checkPointBuffer.tailPhase = checkPointBuffer.headPhase;
    }

    return squash_mask;
}

//////////////////////////////////////////
// Functions related to Retire Stage.   //
//////////////////////////////////////////

///////////////////////////////////////////////////////////////////
// This function allows the caller to examine the instruction at the head
// of the Active List.
///////////////////////////////////////////////////////////////////
// P4-D
bool renamer::precommit(uint64_t &chkpt_id, uint64_t &num_loads, uint64_t &num_stores, uint64_t &num_branches, bool &amo, bool &csr, bool &exception)
{
    //assert(checkPointBuffer.head <= chkpt_id);
    //assert(checkPointBuffer.tail > chkpt_id);

    TS_CPREntries chkpnt = checkPointBuffer.CPR[chkpt_id];
    num_loads = chkpnt.load_count;
    num_stores = chkpnt.store_count;
    num_branches = chkpnt.branch_count;
    amo = chkpnt.amoFlag;
    csr = chkpnt.csrFlag;
    exception = chkpnt.exceptionBit;

    uint64_t oldest_chkpt_id = checkPointBuffer.head;
    uint64_t next_oldest_chkpt_id;
    if (oldest_chkpt_id < checkPointBuffer.size - 1)
    {
        next_oldest_chkpt_id = oldest_chkpt_id + 1;
    }
    else
    {
        next_oldest_chkpt_id = 0;
    }

    if (checkPointBuffer.valid[next_oldest_chkpt_id]==true && checkPointBuffer.CPR[oldest_chkpt_id].uncomp_instr==0)
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
/////////////////////////////////////////////////////////////////////
// P4-D
void renamer::commit(uint64_t log_reg) {
    uint64_t oldest_chkpt_id = checkPointBuffer.head;
    // Num of Logical Regs = RMT.size
    assert(log_reg < RMT.size);
    uint64_t phys_reg = checkPointBuffer.RMT[oldest_chkpt_id].entry[log_reg];
    dec_usage_counter(phys_reg);  // Current CPR
}

//////////////////////////////////////////////////////////////////////
// Squash the renamer class.
/////////////////////////////////////////////////////////////////////
// P4 - D
void renamer::squash()
{
    uint64_t oldest_chkpt_id = checkPointBuffer.head;
    RMT = checkPointBuffer.RMT[oldest_chkpt_id];
    CPR = checkPointBuffer.CPR[oldest_chkpt_id];

    if (oldest_chkpt_id < checkPointBuffer.size - 1)
    {
        checkPointBuffer.tail = checkPointBuffer.head + 1;
        checkPointBuffer.tailPhase = checkPointBuffer.headPhase;
    }
    else
    {
        checkPointBuffer.tail = 0;
        checkPointBuffer.tailPhase = !(checkPointBuffer.headPhase);
    }

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

    //printf("\n* Completed squash() of the pipeline\n");
    //printDetailedStates();
}

//////////////////////////////////////////
// Functions not tied to specific stage //
//////////////////////////////////////////
void renamer::set_exception(uint64_t chkpt_id) {
    checkPointBuffer.CPR[chkpt_id].exceptionBit = 1;
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
