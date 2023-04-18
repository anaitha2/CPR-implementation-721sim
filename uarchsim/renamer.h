#include <iostream>
#include <inttypes.h>
#include <cassert>
#include <vector>
using std::vector;
#include <bits/stdc++.h>
#include <math.h>

using namespace std;

class renamer {
private:
	/////////////////////////////////////////////////////////////////////
	// Put private class variables here.
	/////////////////////////////////////////////////////////////////////

	/////////////////////////////////////////////////////////////////////
	// Structure 1: Rename Map Table
	// Entry contains: physical register mapping
	/////////////////////////////////////////////////////////////////////
	// RMT [logical reg number] = physical register number
	struct TS_MapTable {
		vector<uint64_t> entry;
		uint64_t size;
	};
	
	TS_MapTable RMT;

	void initializeRMT(uint64_t n_log_regs)
	{
		RMT.size = n_log_regs;
		// Assigning Physical Registers i = 0 to Total_Logical_Registers - 1 to the Logical Registers i = 0 to Total_Logical_Registers - 1
		for (uint64_t i=0 ; i<RMT.size; i++)
			RMT.entry.push_back(i);
	}

	/////////////////////////////////////////////////////////////////////
	// Structure 2: Architectural Map Table
	// Entry contains: physical register mapping
	/////////////////////////////////////////////////////////////////////
	// AMT [logical reg number] = physical register number
	TS_MapTable AMT;

	void initializeAMT(uint64_t n_log_regs)
	{
		AMT.size = n_log_regs;
		// Assigning Physical Registers i = 0 to Total_Logical_Registers - 1 to the Logical Registers i = 0 to Total_Logical_Registers - 1
		for (uint64_t i=0 ; i<AMT.size; i++)
			AMT.entry.push_back(i);
	}
	
	/////////////////////////////////////////////////////////////////////
	// Structure 3: Free List
	//
	// Entry contains: physical register number
	//
	// Notes:
	// * Structure includes head, tail, and their phase bits.
	/////////////////////////////////////////////////////////////////////
	struct {
		vector<uint64_t> entry;
		uint64_t head, tail;
		bool headPhase, tailPhase;
		uint64_t size;
	} freeList;

	void initializeFreeList(uint64_t n_phys_regs, uint64_t n_log_regs)
	{
		freeList.size = n_phys_regs - n_log_regs;

		// PRF numbers that are not in AMT/RMT
		// Assigning Physical Registers i = Total_Logical_Registers to Total_Physical_Registers - 1
		for (uint64_t i = RMT.size ; i < n_phys_regs; i++)
			freeList.entry.push_back(i);
		
		freeList.head = 0;
		freeList.headPhase = 0;
		freeList.tail = 0;
		freeList.tailPhase = 1;
	}

	uint64_t noOfFreeRegistersInFreeList()
	{
		if (freeList.headPhase == freeList.tailPhase)
		{
			return freeList.tail - freeList.head;
		}
		else
		{
			return freeList.size - (freeList.head - freeList.tail);
		}
	}

	uint64_t popRegisterFromFreeList()
	{
		assert ((freeList.head != freeList.tail) || (freeList.headPhase != freeList.tailPhase));
		uint64_t physicalRegisterNumber = freeList.entry[freeList.head];
		//printf("\nPop a Reg from FreeList: Head=%llu, PhysicalRegNo=%llu", freeList.head, physicalRegisterNumber);
		if (freeList.head < freeList.size - 1)
		{
			freeList.head++;
		}
		else
		{
			assert (freeList.tailPhase != freeList.headPhase);

			freeList.head = 0;
			freeList.headPhase  = !(freeList.headPhase);
		}
		//printf(" and now newHead=%llu\n", freeList.head);
		return physicalRegisterNumber;
	}

	/////////////////////////////////////////////////////////////////////
	// Structure 4: Active List
	//
	// Entry contains:
	//
	// ----- Fields related to destination register.
	// 1. destination flag (indicates whether or not the instr. has a
	//    destination register)
	// 2. logical register number of the instruction's destination
	// 3. physical register number of the instruction's destination
	// ----- Fields related to completion status.
	// 4. completed bit
	// ----- Fields for signaling offending instructions.
	// 5. exception bit
	// 6. load violation bit
	//    * Younger load issued before an older conflicting store.
	//      This can happen when speculative memory disambiguation
	//      is enabled.
	// 7. branch misprediction bit
	//    * At present, not ever set by the pipeline. It is simply
	//      available for deferred-recovery Approaches #1 or #2.
	//      Project 1 uses Approach #5, however.
	// 8. value misprediction bit
	//    * At present, not ever set by the pipeline. It is simply
	//      available for deferred-recovery Approaches #1 or #2,
	//      if value prediction is added (e.g., research projects).
	// ----- Fields indicating special instruction types.
	// 9. load flag (indicates whether or not the instr. is a load)
	// 10. store flag (indicates whether or not the instr. is a store)
	// 11. branch flag (indicates whether or not the instr. is a branch)
	// 12. amo flag (whether or not instr. is an atomic memory operation)
	// 13. csr flag (whether or not instr. is a system instruction)
	// ----- Other fields.
	// 14. program counter of the instruction
	//
	// Notes:
	// * Structure includes head, tail, and their phase bits.
	/////////////////////////////////////////////////////////////////////
	struct TS_activeListEntries {
		bool destinationFlag;
		uint64_t logicalRegisterNumber;
		uint64_t physicalRegisterNumber;
		bool completedBit;
		bool exceptionBit;
		bool loadViolationBit;
		bool branchMispredictionBit;
		bool valueMispredictionBit;
		bool loadFlag;
		bool storeFlag;
		bool branchFlag;
		bool amoFlag;
		bool csrFlag;
		uint64_t PC;
	};

	struct {
		vector<TS_activeListEntries> entry;
		uint64_t head, tail;
		bool headPhase, tailPhase;
		uint64_t size;
	} activeList;

	void initializeActiveList(uint64_t n_active)
	{
		activeList.size = n_active;
		activeList.head = 0;
		activeList.headPhase = 0;
		activeList.tail = 0;
		activeList.tailPhase = 0;
		
		TS_activeListEntries emptyEntry;
		emptyEntry.destinationFlag = 0;
		emptyEntry.logicalRegisterNumber = 0;
		emptyEntry.physicalRegisterNumber = 0;
		emptyEntry.completedBit = 0;
		emptyEntry.exceptionBit = 0;
		emptyEntry.loadViolationBit = 0;
		emptyEntry.branchMispredictionBit = 0;
		emptyEntry.valueMispredictionBit = 0;
		emptyEntry.loadFlag = 0;
		emptyEntry.storeFlag = 0;
		emptyEntry.branchFlag = 0;
		emptyEntry.amoFlag = 0;
		emptyEntry.csrFlag = 0;
		emptyEntry.PC = 0;

		for (int i = 0; i < activeList.size; i++)
		{
			activeList.entry.push_back(emptyEntry);
		}
	}

	uint64_t noOfFreeEntriesInActiveList()
	{
		if (activeList.headPhase == activeList.tailPhase)
		{
			return activeList.size - (activeList.tail - activeList.head);
		}
		else
		{
			return activeList.head - activeList.tail;
		}
	}

	/////////////////////////////////////////////////////////////////////
	// P4 - CPR Structure
	/////////////////////////////////////////////////////////////////////
	// P4-D 

	struct TS_CPREntries {
		vector <uint64_t> unmappedBit;
		vector <uint64_t> usageCounter;
		uint64_t size;	// Indicates the total number of physical registers
		
		bool loadFlag;
		bool storeFlag;
		bool branchFlag;
		bool amoFlag;
		bool csrFlag;
		bool exceptionBit;
		
		uint64_t  uncomp_instr;                        
	    uint64_t  load_count;
	    uint64_t  store_count;
	    uint64_t  branch_count;

		bool valid;
	} CPR;

	void initializeCPR(uint64_t n_phys_regs, uint64_t n_log_regs)
	{
		CPR.size = n_phys_regs;
		for (uint64_t i = 0; i < CPR.size; i++)
		{
			if (i < n_log_regs)
			{
				// First n_log_regs Logical regs have Unmapped Bit of 0 and Usage counter of 1
				CPR.unmappedBit.push_back(0);
				CPR.usageCounter.push_back(1);
			}
			else
			{
				CPR.unmappedBit.push_back(1);
				CPR.usageCounter.push_back(0);
			}
		}

		CPR.uncomp_instr = 0;
		CPR.load_count = 0;
	    CPR.store_count = 0;
	    CPR.branch_count = 0;

		CPR.valid = 1;
	}
	
	/////////////////////////////////////////////////////////////////////
	// Structure 5: Physical Register File
	// Entry contains: value
	//
	// Notes:
	// * The value must be of the following type: uint64_t
	//   (#include <inttypes.h>, already at top of this file)
	/////////////////////////////////////////////////////////////////////
	TS_MapTable PRF;

	void initializePRF(uint64_t n_phys_regs)
	{
		PRF.size = n_phys_regs;
		// Assigning Values = 0 to Total_Physical_Registers - 1 to Physical Registers i = 0 to Total_Physical_Registers - 1
		for (uint64_t i=0 ; i<PRF.size; i++)
			PRF.entry.push_back(i);
	}

	/////////////////////////////////////////////////////////////////////
	// Structure 6: Physical Register File Ready Bit Array
	// Entry contains: ready bit
	/////////////////////////////////////////////////////////////////////
	TS_MapTable PRFReadyBits;
	
	void initializePRFReadyBits(uint64_t n_phys_regs)
	{
		PRFReadyBits.size = n_phys_regs;
		// Set all PRF Ready Bits to 1
		if (PRFReadyBits.entry.size() == 0)
		{
			for (uint64_t i=0 ; i<PRFReadyBits.size; i++)
			{
				PRFReadyBits.entry.push_back(0);
			}
		}
		for (uint64_t i=0; i<AMT.size; i++)
		{
			assert (AMT.entry[i] < PRFReadyBits.size);
			PRFReadyBits.entry[AMT.entry[i]] = 1;
		}
	}
	
	/////////////////////////////////////////////////////////////////////
	// Structure 7: Global Branch Mask (GBM)
	//
	// The Global Branch Mask (GBM) is a bit vector that keeps track of
	// all unresolved branches. A '1' bit corresponds to an unresolved
	// branch. The "branch ID" of the unresolved branch is its position
	// in the bit vector.
	//
	// The GBM serves two purposes:
	//
	// 1. It provides a means for allocating checkpoints to unresolved
	//    branches. There are as many checkpoints as there are bits in
	//    the GBM. If all bits in the GBM are '1', then there are no
	//    free bits, hence, no free checkpoints. On the other hand, if
	//    not all bits in the GBM are '1', then any of the '0' bits
	//    are free and the corresponding checkpoints are free.
	//    
	// 2. Each in-flight instruction needs to know which unresolved
	//    branches it depends on, i.e., which unresolved branches are
	//    logically before it in program order. This information
	//    makes it possible to squash instructions that are after a
	//    branch, in program order, and not instructions before the
	//    branch. This functionality will be implemented using
	//    branch masks, as was done in the MIPS R10000 processor.
	//    An instruction's initial branch mask is the value of the
	//    the GBM when the instruction is renamed.
	//
	// The simulator requires an efficient implementation of bit vectors,
	// for quick copying and manipulation of bit vectors. Therefore, you
	// must implement the GBM as type "uint64_t".
	// (#include <inttypes.h>, already at top of this file)
	// The "uint64_t" type contains 64 bits, therefore, the simulator
	// cannot support a processor configuration with more than 64
	// unresolved branches. The maximum number of unresolved branches
	// is configurable by the user of the simulator, and can range from
	// 1 to 64.
	/////////////////////////////////////////////////////////////////////
	uint64_t totalUnresolvedBranches;
	uint64_t GBM;

	uint64_t getFreeBitGBM()
	{
		uint64_t g = GBM;
		uint64_t bit = 0;
		if ((g & (g + 1)) == 0)
		{
			bit = (log2(g+1) + 1);
		}
		else
		{
			bit = (log2((~g) & (g+1)) + 1);
		}
		return bit;
	}

	/////////////////////////////////////////////////////////////////////
	// Structure 8: Branch Checkpoints
	//
	// Each branch checkpoint contains the following:
	// 1. Shadow Map Table (checkpointed Rename Map Table)
	// 2. checkpointed Free List head pointer and its phase bit
	// 3. checkpointed GBM
	/////////////////////////////////////////////////////////////////////
	struct {
		vector<TS_MapTable> RMT;
		vector<uint64_t> head;
		vector<uint64_t> headPhase;
		vector<uint64_t> GBM;
		vector<bool> valid;
		uint64_t size;
	} checkPoints;

	void initializeCheckPoint(uint64_t n_branches)
	{
		totalUnresolvedBranches = n_branches;
		checkPoints.size = n_branches;
		TS_MapTable t;
		for (uint64_t i = 0; i < checkPoints.size; i++)
		{
			checkPoints.RMT.push_back(t);
			checkPoints.head.push_back(0);
			checkPoints.headPhase.push_back(0);
			checkPoints.GBM.push_back(0);
			checkPoints.valid.push_back(0);
		}
	}
	
	void initializeCheckPointValues()
	{
		for (uint64_t i = 0; i < checkPoints.size; i++)
		{
			checkPoints.head[i] = 0;
			checkPoints.headPhase[i] = 0;
			checkPoints.GBM[i] = 0;
			checkPoints.valid[i] = 0;
		}
	}

	uint64_t noOfFreeBranchesInCheckPoint()
	{
		uint64_t freeRegisters = 0;
		for (uint64_t i = 0; i < checkPoints.size; i++)
		{
			if (checkPoints.valid[i] == 0)
			{
				freeRegisters++;
			}
		}
		return freeRegisters;
	}

	
	/////////////////////////////////////////////////////////////////////
	// P4 - Checkpoint Buffer
	/////////////////////////////////////////////////////////////////////
	struct {
		vector<TS_CPREntries> CPR;
		vector<TS_MapTable> RMT;

		uint64_t head, tail;
		bool headPhase, tailPhase;
		uint64_t size;	// Indicates the total number of checkpoints
		uint64_t rob_size;	// Indicates the total number of instructions that can be checkpointed
		uint64_t max_instr_bw_checkpoints;	// Indicates the maximum number of instructions between 2 checkpoints
	} checkPointBufferCPR;

	void initializecheckPointBufferCPR(uint64_t n_phys_regs, uint64_t n_log_regs, uint64_t n_checkpoints, uint64_t n_active)
	{
		checkPointBufferCPR.size = n_checkpoints;
		checkPointBufferCPR.rob_size = n_active;
		checkPointBufferCPR.max_instr_bw_checkpoints = n_active / n_checkpoints;
		checkPointBufferCPR.head = 0;
		checkPointBufferCPR.tail = 1;
		checkPointBufferCPR.headPhase = 0;
		checkPointBufferCPR.tailPhase = 0;

		// First checkpoint has the info of the initial state
		checkPointBufferCPR.CPR.push_back(CPR);
		checkPointBufferCPR.RMT.push_back(RMT);

		// Rest 'checkPointBufferCPR.size(total number of checkpoints) - 1' checkpoints are empty
		TS_CPREntries emptyEntry;
		emptyEntry.size = n_phys_regs;
		emptyEntry.valid = false;
		TS_MapTable emptyRMT;
		for (int i = 0; i < checkPointBufferCPR.size-1; i++)
		{
			checkPointBufferCPR.CPR.push_back(emptyEntry);
			checkPointBufferCPR.RMT.push_back(emptyRMT);
		}
	}

	uint64_t noOfAvailableCheckpointsCPR()
	{
		if (checkPointBufferCPR.headPhase == checkPointBufferCPR.tailPhase)
		{
			return checkPointBufferCPR.size - (checkPointBufferCPR.tail - checkPointBufferCPR.head);
		}
		else
		{
			return checkPointBufferCPR.tail - checkPointBufferCPR.head;
		}
	}
	//P4-D

	void rollbackUnmappedandUsagebits(uint64_t checkpoint_ID) {
		for (uint64_t i = 0; i < RMT.size; i++) {
			checkPointBufferCPR.CPR[checkpoint_ID].usageCounter[RMT.entry[i]] = 1;
			checkPointBufferCPR.CPR[checkpoint_ID].usageCounter[RMT.entry[i]] = 0;
		}
	}

	/////////////////////////////////////////////////////////////////////
	// Private functions.
	// e.g., a generic function to copy state from one map to another.
	/////////////////////////////////////////////////////////////////////

public:
	////////////////////////////////////////
	// Public functions.
	////////////////////////////////////////

	void inc_usage_counter(uint64_t phys_reg)
	{
		CPR.usageCounter[phys_reg]++;
	}
	void dec_usage_counter(uint64_t phys_reg)
	{
		CPR.usageCounter[phys_reg]--;
	}
	void unmap(uint64_t phys_reg)	// no longer in RMT
	{
		CPR.unmappedBit[phys_reg] = 1;
	}
	void map(uint64_t phys_reg)	// While adding physical reg to RMT
	{
		CPR.unmappedBit[phys_reg] = 0;
	}
	uint64_t nextIndexCPR(uint64_t index)
	{
		if (index < checkPointBufferCPR.size - 1)
		{
			index++;
		}
		else
		{
			index = 0;
		}
		
		return index;
	}
	uint64_t prevIndexCPR(uint64_t index)
	{
		if (index > 0)
		{
			index--;
		}
		else
		{
			index = checkPointBufferCPR.size - 1;
		}
		
		return index;
	}

	void increamentHeadInFreeList()
	{
		if (freeList.head < freeList.size - 1)
		{
			freeList.head++;
		}
		else
		{
			// head is at the last index
			// If tail is at last index and headPhase==tailPhase then we cannot advance head
			// because the free list is empty and there are no registers to pop
			assert( !((freeList.tail == freeList.size - 1) && (freeList.tailPhase == freeList.headPhase)) );
			freeList.head = 0;
			freeList.headPhase  = !(freeList.headPhase);
		}
	}
	void increamentTailInFreeList()
	{
		if (freeList.tail < freeList.size - 1)
		{
			freeList.tail++;
		}
		else
		{
			// tail is at the last index
			// If head is at last index and tailPhase!=headPhase then we cannot advance tail
			// because the free list is full
			assert( !((freeList.head == freeList.size - 1) && (freeList.tailPhase != freeList.headPhase)) );
			freeList.tail = 0;
			freeList.tailPhase  = !(freeList.tailPhase);
		}
	}

	void increamentUncompletedInstr(uint64_t checkpoint_ID)
	{
		checkPointBufferCPR.CPR[checkpoint_ID].uncomp_instr++;
	}

	uint64_t get_max_instr_bw_checkpoints()
	{
		return checkPointBufferCPR.max_instr_bw_checkpoints;
	}

	void printRMTState()
	{
		std::cout << "-------- RMT State --------\n";
		uint64_t m = 0;
		for (uint64_t i = 0 ; i < RMT.size; i++)
		{
			std::cout << 'r' << i << "(p" << RMT.entry[i] << ")" << "\t";
			if (m < 9)
			{
				m++;
			}
			else
			{
				std::cout << '\n';
				m = 0;
			}
		}
		std::cout << "---------------------------\n";
	}

	void printAMTState()
	{
		std::cout << "-------- AMT State --------\n";
		uint64_t m = 0;
		for (uint64_t i = 0 ; i < AMT.size; i++)
		{
			std::cout << 'r' << i << "(p" << AMT.entry[i] << ")" << "\t";
			if (m < 9)
			{
				m++;
			}
			else
			{
				std::cout << '\n';
				m = 0;
			}
		}
		std::cout << "---------------------------\n";
	}

	void printFreeListState() 
	{
		std::cout << "-------- FreeList State --------\n";
		printf("HEAD=%llu, TAIL=%llu, HEADPHASE=%llu, TAILPHASE=%llu, SIZE=%llu\n", freeList.head, freeList.tail, freeList.headPhase, freeList.tailPhase, freeList.size);
		uint64_t m = 0;
		for (uint64_t i = 0 ; i < freeList.size; i++)
		{
			std::cout << i << "(p" << freeList.entry[i] << ")" << "\t";
			if (m < 9)
			{
				m++;
			}
			else
			{
				std::cout << '\n';
				m = 0;
			}
		}
		std::cout << "--------------------------------\n";
	}

	void printPRFState() 
	{
		std::cout << "-------- PRF State --------\n";
		uint64_t m = 0;
		for (uint64_t i = 0 ; i < PRF.size; i++)
		{
			std::cout << 'p' << i << "(" << PRF.entry[i] << ")" << "\t";
			if (m < 9)
			{
				m++;
			}
			else
			{
				std::cout << '\n';
				m = 0;
			}
		}
		std::cout << "---------------------------\n";
	}

	void printPRFReadyBitState() 
	{
		std::cout << "-------- PRF Ready Bit State --------\n";
		uint64_t m = 0;
		for (uint64_t i = 0 ; i < PRFReadyBits.size; i++)
		{
			std::cout << 'p' << i << "(" << PRFReadyBits.entry[i] << ")" << "\t";
			if (m < 9)
			{
				m++;
			}
			else
			{
				std::cout << '\n';
				m = 0;
			}
		}
		std::cout << "-------------------------------------\n";
	}

	void printActiveListState()
	{
		std::cout << "-------- Active List State --------\n";
		printf("HEAD=%llu, TAIL=%llu, HEADPHASE=%llu, TAILPHASE=%llu, SIZE=%llu\n", activeList.head, activeList.tail, activeList.headPhase, activeList.tailPhase, activeList.size);
		if (activeList.head <= activeList.tail)
		{
			for (uint64_t i = activeList.head; i < activeList.tail; i++)
			{
				std::cout << i << "(destinationFlag=" << activeList.entry[i].destinationFlag << ", r" << activeList.entry[i].logicalRegisterNumber << ", p" << activeList.entry[i].physicalRegisterNumber << ", completedBit=" << activeList.entry[i].completedBit << ", " << activeList.entry[i].exceptionBit << ", " << activeList.entry[i].loadViolationBit << ", branchMispredictionBit=" << activeList.entry[i].branchMispredictionBit << ", " << activeList.entry[i].valueMispredictionBit << ", loadFlag=" << activeList.entry[i].loadFlag << ", storeFlag=" << activeList.entry[i].storeFlag << ", branchFlag=" << activeList.entry[i].branchFlag << ", " << activeList.entry[i].amoFlag << ", " << activeList.entry[i].csrFlag << ", " << activeList.entry[i].PC << ")\n";
			}
		}
		else
		{
			for (uint64_t i = activeList.head; i < activeList.size; i++)
			{
				std::cout << i << "(destinationFlag=" << activeList.entry[i].destinationFlag << ", r" << activeList.entry[i].logicalRegisterNumber << ", p" << activeList.entry[i].physicalRegisterNumber << ", completedBit=" << activeList.entry[i].completedBit << ", " << activeList.entry[i].exceptionBit << ", " << activeList.entry[i].loadViolationBit << ", branchMispredictionBit=" << activeList.entry[i].branchMispredictionBit << ", " << activeList.entry[i].valueMispredictionBit << ", loadFlag=" << activeList.entry[i].loadFlag << ", storeFlag=" << activeList.entry[i].storeFlag << ", branchFlag=" << activeList.entry[i].branchFlag << ", " << activeList.entry[i].amoFlag << ", " << activeList.entry[i].csrFlag << ", " << activeList.entry[i].PC << ")\n";
			}
			for (uint64_t i = 0; i < activeList.tail; i++)
			{
				std::cout << i << "(destinationFlag=" << activeList.entry[i].destinationFlag << ", r" << activeList.entry[i].logicalRegisterNumber << ", p" << activeList.entry[i].physicalRegisterNumber << ", completedBit=" << activeList.entry[i].completedBit << ", " << activeList.entry[i].exceptionBit << ", " << activeList.entry[i].loadViolationBit << ", branchMispredictionBit=" << activeList.entry[i].branchMispredictionBit << ", " << activeList.entry[i].valueMispredictionBit << ", loadFlag=" << activeList.entry[i].loadFlag << ", storeFlag=" << activeList.entry[i].storeFlag << ", branchFlag=" << activeList.entry[i].branchFlag << ", " << activeList.entry[i].amoFlag << ", " << activeList.entry[i].csrFlag << ", " << activeList.entry[i].PC << ")\n";
			}
		}
		std::cout << "-----------------------------------\n";
	}

	void printCheckpointState()
	{
		std::cout << "-------- Checkpoints State --------\n";
		printf("CheckPoint Size=%llu\n", checkPoints.size);
		for(uint64_t i = 0; i < checkPoints.size ; i++)
		{
			if (checkPoints.valid[i] == 1)
			{
				std::cout << "branch_id(" << i << "), ";
				std::cout << "head(" << checkPoints.head[i] << "), ";
				std::cout << "headPhase(" << checkPoints.headPhase[i] << "), ";
				std::cout << "GBM(" << std::bitset<32>(checkPoints.GBM[i]) << "), ";
				std::cout << "valid(" << checkPoints.valid[i] << ")\n";
			}
		}
		std::cout << "-----------------------------------\n";
	}

	void printDetailedStates()
	{
		printRMTState();
		printAMTState();
		printFreeListState();
		printActiveListState();
		printPRFState();
		printPRFReadyBitState();
		printCheckpointState();
	}
	
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
	renamer(uint64_t n_log_regs, uint64_t n_phys_regs, uint64_t n_branches, uint64_t n_active);

	/////////////////////////////////////////////////////////////////////
	// This is the destructor, used to clean up memory space and
	// other things when simulation is done.
	// I typically don't use a destructor; you have the option to keep
	// this function empty.
	/////////////////////////////////////////////////////////////////////
	~renamer();

	//////////////////////////////////////////
	// Functions related to Rename Stage.   //
	//////////////////////////////////////////

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
	bool stall_reg(uint64_t bundle_dst);

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
	bool stall_branch(uint64_t bundle_branch);

	/////////////////////////////////////////////////////////////////////
	// This function is used to get the branch mask for an instruction.
	/////////////////////////////////////////////////////////////////////
	uint64_t get_branch_mask()
	{
		//std::cout << "\n* Completed get_branch_mask() to read GBM=" << std::bitset<32>(GBM) << '\n';
		return GBM;
	}

	/////////////////////////////////////////////////////////////////////
	// This function is used to rename a single source register.
	//
	// Inputs:
	// 1. log_reg: the logical register to rename
	//
	// Return value: physical register name
	/////////////////////////////////////////////////////////////////////
	uint64_t rename_rsrc(uint64_t log_reg);

	/////////////////////////////////////////////////////////////////////
	// This function is used to rename a single destination register.
	//
	// Inputs:
	// 1. log_reg: the logical register to rename
	//
	// Return value: physical register name
	/////////////////////////////////////////////////////////////////////
	uint64_t rename_rdst(uint64_t log_reg);

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
	void checkpoint();
	
	//P4-D get_checkPoint_ID declaration
	unsigned int get_checkPoint_ID(bool load, bool store, bool branch, bool amo, bool csr);
	
	void free_checkpoint();

	bool stall_checkpoint(uint64_t bundle_chkpts);

	//////////////////////////////////////////
	// Functions related to Dispatch Stage. //
	//////////////////////////////////////////

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
	bool stall_dispatch(uint64_t bundle_inst);

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
	uint64_t dispatch_inst(bool dest_valid, uint64_t log_reg, uint64_t phys_reg, bool load, bool store,
	                       bool branch, bool amo, bool csr, uint64_t PC);

	/////////////////////////////////////////////////////////////////////
	// Test the ready bit of the indicated physical register.
	// Returns 'true' if ready.
	/////////////////////////////////////////////////////////////////////
	bool is_ready(uint64_t phys_reg);

	/////////////////////////////////////////////////////////////////////
	// Clear the ready bit of the indicated physical register.
	/////////////////////////////////////////////////////////////////////
	void clear_ready(uint64_t phys_reg);


	//////////////////////////////////////////
	// Functions related to the Reg. Read   //
	// and Execute Stages.                  //
	//////////////////////////////////////////

	/////////////////////////////////////////////////////////////////////
	// Return the contents (value) of the indicated physical register.
	/////////////////////////////////////////////////////////////////////
	uint64_t read(uint64_t phys_reg);

	/////////////////////////////////////////////////////////////////////
	// Set the ready bit of the indicated physical register.
	/////////////////////////////////////////////////////////////////////
	void set_ready(uint64_t phys_reg);


	//////////////////////////////////////////
	// Functions related to Writeback Stage.//
	//////////////////////////////////////////

	/////////////////////////////////////////////////////////////////////
	// Write a value into the indicated physical register.
	/////////////////////////////////////////////////////////////////////
	void write(uint64_t phys_reg, uint64_t value);

	/////////////////////////////////////////////////////////////////////
	// Set the completed bit of the indicated entry in the Active List.
	/////////////////////////////////////////////////////////////////////
	//void set_complete(uint64_t AL_index);
    // P4-D
	void set_complete(uint64_t checkpoint_ID);
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
	void resolve(uint64_t AL_index, uint64_t branch_ID, bool correct);

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
	//bool precommit(bool &completed, bool &exception, bool &load_viol, bool &br_misp, bool &val_misp,
	//               bool &load, bool &store, bool &branch, bool &amo, bool &csr, uint64_t &PC);
    // P4-D
    bool precommit(uint64_t &chkpt_id, uint64_t &num_loads, uint64_t &num_stores, 
                   uint64_t &num_branches, bool &amo, bool &csr, bool &exception);

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
	//void commit();
	// P4-D
	void commit(uint64_t log_reg);

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
	void squash();

	//////////////////////////////////////////
	// Functions not tied to specific stage.//
	//////////////////////////////////////////

	/////////////////////////////////////////////////////////////////////
	// Functions for individually setting the exception bit,
	// load violation bit, branch misprediction bit, and
	// value misprediction bit, of the indicated entry in the Active List.
	/////////////////////////////////////////////////////////////////////
	//void set_exception(uint64_t AL_index);
	//P4-D
	void set_exception(uint64_t checkpoint_ID);
	void set_load_violation(uint64_t AL_index);
	void set_branch_misprediction(uint64_t AL_index);
	void set_value_misprediction(uint64_t AL_index);

	/////////////////////////////////////////////////////////////////////
	// Query the exception bit of the indicated entry in the Active List.
	/////////////////////////////////////////////////////////////////////
	bool get_exception(uint64_t AL_index);
};

/*uint64_t getIndexOfValueInVector(vector<uint64_t> x, uint64_t value)
{
    auto it = find(x.begin(), x.end(), value);
    if (it != x.end())
    {
        uint64_t index = it - x.begin();
		return index;
    }
    else
	{
        return -1;
    }
}*/
