#include "llvm/Pass.h"
#include "llvm/IR/Module.h"
#include "llvm/IR/Function.h"
#include "llvm/IR/BasicBlock.h"
#include "llvm/IR/Instructions.h"
#include "llvm/IR/Attributes.h"
#include "llvm/IR/DebugInfoMetadata.h"
#include "llvm/IR/Constants.h"
#include "llvm/IR/GlobalValue.h"
#include "llvm/IR/GlobalVariable.h"
#include "llvm/IR/LLVMContext.h"
#include "llvm/IR/IRBuilder.h"
#include "llvm/IR/TypeBuilder.h"
#include "llvm/IR/DerivedTypes.h"
#include "llvm/IR/InstIterator.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Transforms/Utils/ModuleUtils.h"

#include "llvm/Transforms/OMPCheckPass.h"

#include <set>
#include <cxxabi.h>

#define DEBUG_TYPE "ompcheckpass"

using namespace llvm;

namespace {

enum InstType {
  LOAD = 0,
  STORE,
};

	void defineRuntimeConstructor(Module &module) {
		
		LLVMContext &context = module.getContext();
		StringRef functionName = StringRef("__initCheckRuntime");
		FunctionType *functionType =
			TypeBuilder<void(void), false>::get(context);
		AttributeSet attrSet = AttributeSet().addAttribute(context,
        AttributeSet::FunctionIndex, Attribute::NoInline);
		Constant *ret = module.getOrInsertFunction(functionName, functionType,
			attrSet);
		// setting lowest priority (16bit = 65535)
		appendToGlobalCtors(module, dyn_cast<Function>(ret), 65535);
	}
	
	void defineRuntimeDestructor(Module &module) {
		
		LLVMContext &context = module.getContext();
		StringRef functionName = StringRef("__termCheckRuntime");
		FunctionType *functionType =
			TypeBuilder<void(void), false>::get(context);
		AttributeSet attrSet = AttributeSet().addAttribute(context,
        AttributeSet::FunctionIndex, Attribute::NoInline);
		Constant *ret = module.getOrInsertFunction(functionName, functionType,
			attrSet);
		// setting lowest priority (16bit = 65535)
		appendToGlobalDtors(module, dyn_cast<Function>(ret), 65535);
	}

  void analyzeInstruction(const Value& v,
      std::set<const Instruction*>& instructionsToSkip) {
    std::set<const User*> users;
    for (const User* u : v.users()) {
      if (users.count(u) == 0) {
        users.insert(u);
      }
    }
    bool changed;
    do {
      changed = false;
      for (const User* user : users) {
        for (const User* u : user->users()) {
          if (users.count(u) == 0) {
            users.insert(u);
            changed = true;
          }
        }
      }
    } while (changed);
    for (const User* u : users) {
      if (isa<LoadInst>(u) || isa<StoreInst>(u)) {
        const Instruction* inst = static_cast<const Instruction*>(u);
        instructionsToSkip.insert(inst);
        //llvm::errs() << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\n";
        //inst->dump();
        //llvm::errs() << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\n";
      }
    }
  }

  void analyzeFunctionArguments(Function::ArgumentListType& argList,
      std::set<const Instruction*>& instructionsToSkip) {
    for (const Argument& arg : argList) {
      if ( ! arg.hasAttribute(Attribute::AttrKind::StructRet) ) {
        continue;
      }
      const Value& v = static_cast<const Value&>(arg);
      analyzeInstruction(v, instructionsToSkip);
    }
  }

  bool analyzeFunctionCall(StringRef targetName, CallInst& call,
      std::set<const Instruction*>& instructionsToSkip) {

    int status;
    const char* demangleName =
      abi::__cxa_demangle(targetName.str().c_str(),
          nullptr, nullptr, &status);
    if (status != 0) return false;
    std::string functionName(demangleName);
    if (functionName.find("new") != std::string::npos) {
      //llvm::errs() << "@@@ call to " << functionName << '\n';
      //call.dump();
      analyzeInstruction(call, instructionsToSkip);
      //llvm::errs() << "@@@ call to " << functionName << '\n';
      return true;
    }
    return false;
  }
	
	struct OMPCheck : public ModulePass {

    Function* checkDependenceFunction;
    Function* functionExitFunction;
    Value* globalVariableFileName;
public:	
		static char ID;
		OMPCheck() : ModulePass(ID) {
      initializeOMPCheckPass(*PassRegistry::getPassRegistry());
    }
    virtual const char* getPassName() const override {
      return "OpenMP Check Pass";
    }

    Function* addCheckDependenceFunctionDefinition(Module& M) {
      LLVMContext& context = M.getContext();
      
      StringRef functionName = StringRef("__check_dependence");
      FunctionType *functionType =
        TypeBuilder<void(const void*, unsigned char), false>::get(context);
      AttributeSet attrSet = AttributeSet().addAttribute(context,
        AttributeSet::FunctionIndex, Attribute::NoInline);
      Constant *ret = M.getOrInsertFunction(functionName, functionType,
        attrSet);

      if (isa<Function>(ret)) {
        //errs() <<  "successfully inserted ";
        //errs() << functionName << " definition!" << '\n';
        //static_cast<Function*>(ret)->dump();
        return static_cast<Function*>(ret);
      } else {
        errs() <<  "failed to insert ";
        errs() << functionName << " definition!" << '\n';
        return NULL;
      }
    }

    Function* addFunctionExitFunction(Module& M) {
      LLVMContext& context = M.getContext();

      StringRef functionName = StringRef("__function_exit");
      FunctionType *functionType =
        TypeBuilder<void(void), false>::get(context);
      AttributeSet attrSet = AttributeSet().addAttribute(context,
        AttributeSet::FunctionIndex, Attribute::NoInline);
      Constant *ret = M.getOrInsertFunction(functionName, functionType,
        attrSet);

      if (isa<Function>(ret)) {
        //errs() <<  "successfully inserted ";
        //errs() << functionName << " definition!" << '\n';
        //static_cast<Function*>(ret)->dump();
        return static_cast<Function*>(ret);
      } else {
        errs() <<  "failed to insert ";
        errs() << functionName << " definition!" << '\n';
        return NULL;
      }
    }

    void insertProfilingCall(Instruction& currInst,
        const Value* pointerOperand, enum InstType type) {

      // we assume that every load/store has a successor
      Instruction* nextInst = currInst.getNextNode();
      currInst.dump();

      std::vector<Value*> Args(2);

      LLVMContext& context = currInst.getContext();

      Type* voidPtrTy = Type::getInt8PtrTy(context, /*?AddressSpace?*/0);

      if ( pointerOperand->getType() == voidPtrTy ) {
        Args[0] = const_cast<Value*>(pointerOperand);
      } else {
        Value* S = const_cast<Value*>(pointerOperand);
        Args[0] = CastInst::CreatePointerCast(S, voidPtrTy, "castToVoidPtrTy", nextInst);
      }

      Args[1] = ConstantInt::get(Type::getInt8Ty(context),
          type == InstType::STORE, /*isSigned*/false);

      // instructions which do not produce register should not have name
      CallInst::Create(checkDependenceFunction, Args, /*name*/"", nextInst);
    }

    void insertExitCall(Function& F) {
      for (BasicBlock& BB : F.getBasicBlockList()) {
        for (Instruction& I : BB.getInstList()) {
          if (isa<ReturnInst>(I)) {
            CallInst::Create(functionExitFunction, None, /*name*/"", &I);
          }
        }
      }
    }

		bool runOnModule(Module &M) override {

      Function* main = M.getFunction("main");

      checkDependenceFunction = addCheckDependenceFunctionDefinition(M);
      functionExitFunction = addFunctionExitFunction(M);

      Constant* fileNameConstant =
        ConstantDataArray::getString(M.getContext(), M.getName());
      globalVariableFileName = new llvm::GlobalVariable(M,
          fileNameConstant->getType(),
          /*isConstant*/true, GlobalValue::LinkageTypes::InternalLinkage,
          fileNameConstant);

      // only define (con|de)structor in the "main" module
      if (main != nullptr) {
  			defineRuntimeConstructor(M);
	  		defineRuntimeDestructor(M);
      }

      unsigned insideProfilingRegion = 0;
      bool insideFunctionToProfile = false;

      StringRef moduleFileName = M.getSourceFileName();

      std::vector<Instruction*> instructionsToDelete;
      std::vector<Instruction*> callsInsideLoop;
      std::set<Function*> functionsWithInstrumentedExit;

      Module::FunctionListType &functionList = M.getFunctionList();
			for (Function &F : functionList) {
        llvm::DISubprogram* subprogram = F.getSubprogram();
				if ( !F.isIntrinsic() && !F.isDeclaration() && subprogram
            && (moduleFileName.compare(subprogram->getFilename()) == 0)) {
          //llvm::errs() << "Function filename " << F.getSubprogram()->getFilename() << '\n';
          std::set<const Instruction*> loadStoreInstructionsToSkip;
          //llvm::errs() << "FUNCTION: " << F.getName() << '\n';
          analyzeFunctionArguments(F.getArgumentList(), loadStoreInstructionsToSkip);
					Function::BasicBlockListType &BBList = F.getBasicBlockList();
          callsInsideLoop.clear();
					for (BasicBlock &BB : BBList) {
            BasicBlock::InstListType &InstList = BB.getInstList();
            for (Instruction &I : InstList) {
              if (isa<CallInst>(I)) {
                CallInst &call = static_cast<CallInst&>(I);
                const Function* targetFunction = call.getCalledFunction();
                if (targetFunction != nullptr) {
                  StringRef targetName = targetFunction->getName();
                  if (insideFunctionToProfile) {
                    bool skip = analyzeFunctionCall(targetName, call,
                        loadStoreInstructionsToSkip);
                    if (skip) continue;
                  }
                  if(targetName.compare("__start_iter_prof") == 0) {
                    //errs() << "=== found call to __start_iter_prof\n";
                    insideProfilingRegion++;
                    continue;
                  }
                  if (targetName.compare("__stop_iter_prof") == 0) {
                    //errs() << "=== found call to __stop_iter_prof\n";
                    insideProfilingRegion--;
                    continue;
                  }
                  if (targetName.compare("__function_entry") == 0) {
                    //errs() << "=== found call to __function_entry\n";
                    insideFunctionToProfile = true;
                    Value* operand0 = call.getArgOperand(0);
                    if ( ! isa<ConstantInt>(operand0) ) {
                      errs() << "fatal error: __function_entry arg[0] is not ConstantInt!\n";
                      continue;
                    }
                    ConstantInt* constInt =
                      static_cast<ConstantInt*>(operand0);
                    if ( constInt->getZExtValue() == 0 ) {
                      instructionsToDelete.push_back(&I);
                    } else {
                      // Only instrument once the exits of each function called
                      // inside loop
                      if (functionsWithInstrumentedExit.count(&F) == 0) {
                        insertExitCall(F);
                        functionsWithInstrumentedExit.insert(&F);
                      }
                    }
                    continue;
                  }
                } else {
                  //llvm::errs() << "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n";
                  //llvm::errs() << "WARNING: indirect function call found!";
                  //llvm::errs() << " Results might be innacurate!" << '\n';
                  //llvm::errs() << "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n";
                  continue;
                }
              }
              if (isa<AllocaInst>(I)) {
                analyzeInstruction(I, loadStoreInstructionsToSkip);
              }
              if (loadStoreInstructionsToSkip.count(
                    static_cast<const Instruction*>(&I)) > 0
                  && insideFunctionToProfile) {
                continue;
              }
              if (insideProfilingRegion > 0 || insideFunctionToProfile) {
                if (isa<LoadInst>(I)) {
                    const LoadInst& loadInst = static_cast<LoadInst&>(I);
                    const Value* addr = loadInst.getPointerOperand();
                    if (!isa<AllocaInst>(addr)) {
                      insertProfilingCall(I, addr, InstType::LOAD);
                    }
                } else if (isa<StoreInst>(I)) {
                  const StoreInst& storeInst = static_cast<StoreInst&>(I);
                  const Value* addr = storeInst.getPointerOperand();
                  if (!isa<AllocaInst>(addr)) {
                    insertProfilingCall(I, addr, InstType::STORE);
                  }
                }
              }
            } // FOR EACH INSTRUCTION
          } // FOR EACH BASICBLOCK
        } // !F.isIntrinsic() && !F.isDeclaration()
        insideFunctionToProfile = false;
      } // FOR EACH FUNCTION

      for (Instruction* I : instructionsToDelete){
        I->eraseFromParent();
      }

      // Module was modified
			return true;
		}
	};
}

char OMPCheck::ID = 0;

INITIALIZE_PASS(
    OMPCheck, "ompcheckpass",
    "LLVM pass to instrument OpenMP for check loops.", false, false)

namespace llvm {
  Pass* createOMPCheckPass() {
    return new OMPCheck();
  }
} // llvm namespace
