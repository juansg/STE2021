#define DEBUG_TYPE "printCode"
#include "llvm/Pass.h"
#include "llvm/IR/Module.h"
#include "llvm/IR/Function.h"
#include "llvm/IR/BasicBlock.h"
#include "llvm/IR/Instruction.h"
#include "llvm/ADT/StringExtras.h"
#include "llvm/ADT/Statistic.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/ADT/DenseMap.h"
#include "llvm/IR/User.h"
#include "llvm/IR/Instructions.h"
#include "llvm/IR/Constants.h"
#include "llvm/Analysis/LoopInfo.h"
#include <set>
#include <vector>
#include <map>
#include "llvm/ADT/SmallVector.h"
#include "llvm/Transforms/Instrumentation.h"
#include "llvm/IR/IRBuilder.h"

using namespace llvm;

namespace {
    
	bool isInsideRegion;

    std::vector<std::vector<const llvm::Value *> > instMap;
    std::set<const llvm::Value *> depMap;
    std::vector<llvm::Value *> iterators;
    std::vector<llvm::Instruction *> comparations;
    std::vector<llvm::Instruction *> comparationsEnd;
    //std::vector<std::vector<llvm::AllocaInst *> > buffers;
    std::map<const llvm::Value *, int> instComps;
    std::map<const llvm::Value *, llvm::AllocaInst *> buffers;
    std::map<const llvm::Value *, llvm::Value *> NEWiterators;
    std::map<const llvm::Value *, std::set<const llvm::User *> > deps;
    //llvm::DenseMap<llvm::Value *, > depMap;
    //llvm::DenseMap<const llvm::Value *, const std::vector<int> > depMap;
//    void print_elem(const llvm::Instruction* i) {
//        llvm::errs() << instMap.lookup(i) << " ";
//    }
    class component {
    public:
        std::set<const llvm::Instruction*> insts;
    };
}

namespace {

struct OMPUseDependenceAnalysis : public FunctionPass {

public:
  static char ID; // Pass identification, replacement for typeid
  OMPUseDependenceAnalysis() : FunctionPass(ID) {
	  initializeOMPUseDependenceAnalysisPass(*PassRegistry::getPassRegistry());
	}

  void getAnalysisUsage(AnalysisUsage &AU) const override {
    AU.setPreservesCFG();
  }

  virtual bool runOnFunction(Function &F);
  virtual void print(std::ostream &O, const Module *M) const;
};

}
