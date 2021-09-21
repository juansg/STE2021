#include <stdio.h>
#include <pthread.h>
#include <unordered_map>
#include <list>
#include <string>

#include "DebugInfo.h"

#define BUFFER_SIZE 4096

struct Instruction {
	int lineNumber;
	unsigned long lastIt;
  const char* filename;
};

enum DependenceType {
  RAW = 0,
  WAR,
  WAW,
  INVALID
};

const char* dependenceTypeToC_str[] {
  "RAW",
  "WAR",
  "WAW",
  "INVALID"
};

struct Dependence {
  unsigned long numberOfOcurrencies;
  DependenceType type;
  void* PC_A;
  void* PC_B;
  unsigned long lastIteration;
  float lcp;
};

struct ParallelRegion {
  void* enterPC;
  void* exitPC;
  std::list<Dependence> dependences;
};

struct LineDepInfo {
  std::string depID;
  std::string direction;
};

static std::list< ParallelRegion > parallelRegions;

static std::unordered_map<void *, std::unordered_map<void *, Dependence> > mapInstructionToMapOfDependencies;

static std::unordered_map<void *, std::unordered_map<void *, unsigned long> > mapAddressToMapOfReads;
static std::unordered_map<void *, std::unordered_map<void *, unsigned long> > mapAddressToMapOfWrites;

extern void* currLoopPC;

static
std::string getDepID() {
  static char dependenceID = 'A';
  static unsigned int depIDCounter = 1;
  if (dependenceID > 'Z') {
    depIDCounter++;
    dependenceID = 'A';
  }
  return std::string(depIDCounter, dependenceID++);
}

static void createDepInfo(DependenceType type,
    LineDepInfo& A, LineDepInfo& B) {
  switch (type) {
    case DependenceType::RAW:
      A.depID = B.depID = getDepID();
      A.direction = ">>";
      B.direction = "<<";
      break;
    case DependenceType::WAR:
      A.depID = B.depID = getDepID();
      A.direction = "<<";
      B.direction = ">>";
      break;
    case DependenceType::WAW:
      A.depID = B.depID = getDepID();
      A.direction = "<<<";
      B.direction = "";
      break;
    case DependenceType::INVALID:
    default:
      fprintf(stderr, "fatal error: invalid DependenceType\n");
      exit(EXIT_FAILURE);
      break;
  }
}

static void printLoop(const char* fileName, unsigned int start, unsigned end,
    std::unordered_map<unsigned int, std::list<LineDepInfo>>& mapL2LDI) {
  char *lineBuffer = nullptr;
  size_t lineSize = 0;
  FILE* fileStream = fopen(fileName, "r");
  unsigned int currentLine = 0;
  ssize_t status = getline(&lineBuffer, &lineSize, fileStream);
  while (status > 0 && (currentLine+1) < start) {
    lineSize = 0;
    free(lineBuffer);
    currentLine++;
    status = getline(&lineBuffer, &lineSize, fileStream);
  }
  if (status > 0) {
    unsigned int i;
    for (i = start; i < end; i++) {
      size_t j;
      for (j = 0; j < lineSize; j++) {
        if (lineBuffer[j] == '\t') {
          lineBuffer[j] = ' ';
        }
      }
      std::string s(lineBuffer);
      if (s.find("__start_iter_prof") == std::string::npos
          && s.find("__stop_iter_prof") == std::string::npos) {
        std::unordered_map<unsigned int, std::list<LineDepInfo>>::iterator it;
        it = mapL2LDI.find(i);
        if (it != mapL2LDI.end()) {
          fprintf(stderr, "\n");
          for (LineDepInfo& LDI : it->second) {
            fprintf(stderr, "[%s]%s ",
                LDI.depID.c_str(), LDI.direction.c_str());
          }
          fprintf(stderr, "\n");
        }
        fprintf(stderr, "%s", lineBuffer);
        if (it != mapL2LDI.end()) {
          fprintf(stderr, "\n");
        }
      }
      lineSize = 0;
      free(lineBuffer);
      status = getline(&lineBuffer, &lineSize, fileStream);
      if (status < 0) break;
    }
  }
  fclose(fileStream);
}

void collectDependencies(unsigned long numIterations,
    void* enterPC, void* exitPC) {

  if (mapInstructionToMapOfDependencies.size() > 0) {
    parallelRegions.push_back(ParallelRegion());
    ParallelRegion& PR = parallelRegions.back();
    PR.enterPC = enterPC;
    PR.exitPC = exitPC;
    for (std::pair<void*, std::unordered_map<void*, Dependence>> A :
        mapInstructionToMapOfDependencies) {
      for (std::pair<void*, Dependence> B : A.second) {
        Dependence &D = B.second;
        D.lcp = 100.0*((float)D.numberOfOcurrencies/(float)numIterations);
        PR.dependences.push_back(D);
      }
    }
  }
  mapAddressToMapOfWrites.clear();
  mapAddressToMapOfReads.clear();
  mapInstructionToMapOfDependencies.clear();
}

void reportDependencies() {

  fprintf(stderr,"\n\n==== Dependence Checker Report Start ====\n");
  if (parallelRegions.size() > 0) {
    for (ParallelRegion& PR : parallelRegions) {
      DebugInfo DI_enter( (bfd_vma)PR.enterPC );
      DebugInfo DI_exit( (bfd_vma)PR.exitPC );
      getDebugInfo(&DI_enter);
      getDebugInfo(&DI_exit);
      if ( !DI_enter.found || !DI_exit.found ) {
        fprintf(stderr, "fatal error: failed to get DebugInfo from PC\n");
        exit(EXIT_FAILURE);
      }
      std::unordered_map<unsigned int, std::list<LineDepInfo>> mapLineToLineDepInfo;
      std::unordered_map<unsigned int, std::list<LineDepInfo>>::iterator it;
      for (Dependence& D : PR.dependences) {
        DebugInfo DI_A( (bfd_vma)D.PC_A );
        DebugInfo DI_B( (bfd_vma)D.PC_B );
        getDebugInfo(&DI_A);
        getDebugInfo(&DI_B);
        if ( !DI_A.found || !DI_B.found ) {
          fprintf(stderr, "fatal error: failed to get DebugInfo from PC\n");
          exit(EXIT_FAILURE);
        }
        LineDepInfo depInfo_A, depInfo_B;
        createDepInfo(D.type, depInfo_A, depInfo_B);
        it = mapLineToLineDepInfo.find(DI_A.line);
        if (it == mapLineToLineDepInfo.end()) {
          std::list<LineDepInfo> L;
          L.push_back(depInfo_A);
          mapLineToLineDepInfo[DI_A.line] = L;
        } else {
          it->second.push_back(depInfo_A);
        }
        if (D.type != DependenceType::WAW) {
          it = mapLineToLineDepInfo.find(DI_B.line);
          if (it == mapLineToLineDepInfo.end()) {
            std::list<LineDepInfo> L;
            L.push_back(depInfo_B);
            mapLineToLineDepInfo[DI_B.line] = L;
          } else {
            it->second.push_back(depInfo_B);
          }
        }
      }
      printLoop(DI_enter.filename, DI_enter.line,
          DI_exit.line, mapLineToLineDepInfo);
    }
    parallelRegions.clear();
  } else {
    fprintf(stderr, "No dependencies found!\n");
  }
  fprintf(stderr, "==== Dependence Checker Report End   ====\n");
}

void MemRead(void *readAddr, unsigned long iterationID, void *PC) {

  std::unordered_map<void *, std::unordered_map<void *, unsigned long> >::iterator itMap;
  itMap = mapAddressToMapOfWrites.find(readAddr);

  // if we in a function in the call stack of the loop body
  // then use the currLoopPC (PC of the root call) to track dependences
  void* currPC = (currLoopPC != nullptr) ? currLoopPC : PC;
  currPC = (void*)((uintptr_t)currPC-8);

	if (itMap != mapAddressToMapOfWrites.end()) {
    std::unordered_map<void *, unsigned long> writesMap = (*itMap).second;

    for (std::pair<void*, unsigned long> w  : writesMap) {

      void* writePC = w.first;
      const unsigned long lastIteration = w.second;

      if (lastIteration == iterationID) continue;
      std::unordered_map<void*, Dependence>::iterator depedenceMapIterator;
      void *A = (void*)std::min((uintptr_t)currPC, (uintptr_t)writePC);
      void *B = A == currPC ? writePC : currPC;
      depedenceMapIterator =
        mapInstructionToMapOfDependencies[A].find(B);
      if (depedenceMapIterator ==
          mapInstructionToMapOfDependencies[A].end()) {
        Dependence dep;
				if (iterationID > lastIteration) {
          DependenceType depType = DependenceType::RAW;
          dep = {1 /* numberOfOccurrencies */,
            depType, currPC, writePC, iterationID};
        } else {
          DependenceType depType = DependenceType::WAR;
          dep = {1 /* numberOfOccurrencies */,
            depType, writePC, currPC, iterationID};
        }
        mapInstructionToMapOfDependencies[A][B] = dep;
      } else {
        Dependence& dep = depedenceMapIterator->second;
        if (dep.lastIteration != iterationID) {
          dep.lastIteration = iterationID;
          dep.numberOfOcurrencies++;
        }
      }
    }
	}

  itMap = mapAddressToMapOfReads.find(readAddr);
	if (itMap == mapAddressToMapOfReads.end()) {
    std::unordered_map<void *, unsigned long> auxMap;
			auxMap.insert(std::pair<void *, unsigned long>(currPC, iterationID));
			mapAddressToMapOfReads.insert(std::pair<void *,
          std::unordered_map<void *, unsigned long> >(readAddr, auxMap));
	}
	else {
		if (itMap->second.count(currPC) == 0) {
      itMap = mapAddressToMapOfReads.find(readAddr);
			itMap->second.insert(std::pair<void *, unsigned long>(currPC,
            iterationID));
		}
		else {
      mapAddressToMapOfReads[readAddr][currPC] = iterationID;
    }
	}

	return;
}

void MemWrite(void *writeAddress, unsigned long iterationID, void *PC) {

  std::unordered_map<void *, std::unordered_map<void *, unsigned long> >::iterator itMap;
	itMap = mapAddressToMapOfReads.find(writeAddress);

  // if we in a function in the call stack of the loop body
  // then use the currLoopPC (PC of the root call) to track dependences
  void* currPC = (currLoopPC != nullptr) ? currLoopPC : PC;
  currPC = (void*)((uintptr_t)currPC-8);

	if (itMap != mapAddressToMapOfReads.end()) {

    std::unordered_map<void *, unsigned long> readsMap = (*itMap).second;

    for (std::pair<void*, unsigned long> r  : readsMap) {

      void* readPC = r.first;
      const unsigned long lastIteration = r.second;

      if (lastIteration == iterationID) continue;
      std::unordered_map<void*, Dependence>::iterator depedenceMapIterator;
      void *A = (void*)std::min((uintptr_t)currPC, (uintptr_t)readPC);
      void *B = A == currPC ? readPC : currPC;
      depedenceMapIterator =
        mapInstructionToMapOfDependencies[A].find(B);
      if (depedenceMapIterator ==
          mapInstructionToMapOfDependencies[A].end()) {
        Dependence dep;
				if (iterationID > lastIteration) {
          DependenceType depType = DependenceType::WAR;
          dep = {1 /* numberOfOccurrencies */,
            depType, currPC, readPC, iterationID};
        } else {
          DependenceType depType = DependenceType::RAW;
          dep = {1 /* numberOfOccurrencies */,
            depType, readPC, currPC, iterationID};
        }
        mapInstructionToMapOfDependencies[A][B] = dep;
      } else {
        Dependence& dep = depedenceMapIterator->second;
        if (dep.lastIteration != iterationID) {
          dep.lastIteration = iterationID;
          dep.numberOfOcurrencies++;
        }
      }
    }
	}

	itMap = mapAddressToMapOfWrites.find(writeAddress);

	if (itMap != mapAddressToMapOfWrites.end()) {

    std::unordered_map<void *, unsigned long> writesMap = (*itMap).second;

    for (std::pair<void*, unsigned long> w  : writesMap) {

      void* writePC = w.first;
      const unsigned long lastIteration = w.second;

      if (lastIteration == iterationID) continue;
      std::unordered_map<void*, Dependence>::iterator depedenceMapIterator;
      void *A = (void*)std::min((uintptr_t)currPC, (uintptr_t)writePC);
      void *B = A == PC ? writePC : currPC;
      depedenceMapIterator =
        mapInstructionToMapOfDependencies[A].find(B);
      if (depedenceMapIterator ==
          mapInstructionToMapOfDependencies[A].end()) {
        DependenceType depType = DependenceType::WAW;
        Dependence dep = {1 /* numberOfOccurrencies */,
          depType, currPC, writePC, iterationID};
        mapInstructionToMapOfDependencies[A][B] = dep;
      } else {
        Dependence& dep = depedenceMapIterator->second;
        if (dep.lastIteration != iterationID) {
          dep.lastIteration = iterationID;
          dep.numberOfOcurrencies++;
        }
      }
    }
	}

  itMap = mapAddressToMapOfWrites.find(writeAddress);
	if (itMap == mapAddressToMapOfWrites.end()) {
    std::unordered_map<void *, unsigned long> auxMap;
			auxMap.insert(std::pair<void *, unsigned long>(currPC, iterationID));
			mapAddressToMapOfWrites.insert(std::pair<void *,
          std::unordered_map<void *, unsigned long> >(writeAddress, auxMap));
	}
	else {
		if (itMap->second.count(currPC) == 0) {
      itMap = mapAddressToMapOfWrites.find(writeAddress);
      itMap->second.insert(std::pair<void *, unsigned long>(currPC,
            iterationID));
		}
		else {
      mapAddressToMapOfWrites[writeAddress][currPC] = iterationID;
    }
	}

	return;
}
