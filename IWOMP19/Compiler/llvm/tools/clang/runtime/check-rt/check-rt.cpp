#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <unordered_map>
#include "check-rt.h"

#include "DebugInfo.h"
#include "MemoryProfiling.h"

void* currLoopPC = nullptr;

#define unlikely(x)   __builtin_expect((x),0)
#define likely(x)     __builtin_expect((x),1)

namespace { // anonymous

static unsigned long currentIteration = 0;
static unsigned long stackPointer = 0;
static void* enterPC = nullptr;

enum InstType {
  LOAD = 0,
  STORE,
};

std::unordered_map<void*, DebugInfo> mapPCtoDebugInfo;

} // anonymous namespace

void __initCheckRuntime() {
  //std::cerr << "==== hello from __initCheckRuntime\n";
  initDebugInfo();
}

void __termCheckRuntime() {
  reportDependencies();
  //std::cerr << "==== hello from __termCheckRuntime\n";
  termDebugInfo();
}

void __enterParallelRegion(int tID, int numThreads) {
  currentIteration = 0;
  stackPointer = 0;
  enterPC = __builtin_extract_return_addr(__builtin_return_address(0));
  //std::cerr << "==== hello from __enterParallelRegion\n";
}

void __exitParallelRegion() {
  unsigned long numIterations = currentIteration;
  void * exitPC = __builtin_extract_return_addr(__builtin_return_address(0));
  collectDependencies(numIterations, enterPC, exitPC);
  //std::cerr << "==== hello from __exitParallelRegion\n";
}

void __function_entry(int lineNumber, void *pc) {
  //std::cerr << "==== hello from __function_entry\n";
  //std::cerr << filename << ": " << lineNumber << '\n';
  if (stackPointer == 0) {
    currLoopPC = pc;
  }
  stackPointer++;
}

void __function_exit() {
  //std::cerr << "==== hello from __function_exit\n";
  if (unlikely(stackPointer == 0)) {
    std::cerr << "fatal error: stackPointer == 0 and __function_exit called!\n";
    exit(EXIT_FAILURE);
  }
  stackPointer--;
  if (stackPointer == 0) {
    currLoopPC = nullptr;
  }
}

void __start_iter_prof(unsigned long iter) {
  //std::cerr << "==== hello from __start_iter_prof(";
  //std::cerr << iter << ", " << __threadID__ << ")\n";
  currentIteration = iter;
}

void __stop_iter_prof(unsigned long iter) {
  //std::cerr << "==== hello from __stop_iter_prof(";
  //std::cerr << iter << ", " << __threadID__ << ")\n";
}

void __check_dependence(const void *addr, unsigned char type) {
  //std::cerr << "==== hello from __check_dependence";
  //std::cerr << "currentIteration: " << currentThreadIteration[__threadID__];
  //std::cerr << '\n';
  void * pc = __builtin_extract_return_addr(__builtin_return_address(0));
  //std::cerr << "PC: " << (unsigned long)pc << '\n';

  if (type == InstType::LOAD) {
    MemRead(const_cast<void*>(addr), currentIteration, pc);
  } else {
    MemWrite(const_cast<void*>(addr), currentIteration, pc);
  }
}
