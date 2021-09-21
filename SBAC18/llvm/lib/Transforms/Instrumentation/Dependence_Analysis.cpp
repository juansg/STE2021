#include "Dependence_Analysis.h"
#include <cxxabi.h>

using namespace llvm;

//**********************************************************************
// runOnFunction
//**********************************************************************

bool OMPUseDependenceAnalysis::runOnFunction(Function &F) {
  isInsideRegion = false;

    std::vector<const Value *> *tmp;
	Value *buffer_size;
	Instruction *ret;
	//LoadInst *LoadBS;

	std::map<int, std::set<const Value *> > depsComp;
	std::map<const Value *, std::map<int, Value *> > loads;
	std::set<const Value *> done;
  std::set<const Value*> ignoreInstructions;

  int n_component = 0;
  if(F.getName().startswith(".omp_outlined.")) {
    F.dump();
    for (Argument& arg : F.getArgumentList()) {
      llvm::errs() << "ARG: " << arg << '\n';
      for (const User* user : arg.users()) {
        if (isa<StoreInst>(user)) {
          const StoreInst* storeInst =
            static_cast<const StoreInst*>(user);
          const Value* pointerOperand =
            storeInst->getPointerOperand();
          llvm::errs() << "\tUSES: " << *pointerOperand << '\n';
          ignoreInstructions.insert(static_cast<const Value*>(pointerOperand));
        }
      }
      bool changed;
      do {
        changed = false;
        for (const Value* user : ignoreInstructions) {
          for (const Value* u : user->users()) {
            if (ignoreInstructions.count(u) == 0) {
              llvm::errs() << "\tUSES: " << *u << '\n';
              ignoreInstructions.insert(u);
              changed = true;
            }
          }
        }
      } while(changed);
    }

    Module *M = F.getParent();
    //GlobalListType lista = M->getGlobalList();
    for(auto gb = M->global_begin(); gb != M->global_end(); gb++) {
      std::string str = (*gb).getName();
      if((*gb).getName().rfind("bdx_buffer_size") < str.size()) {
        llvm::errs() << (*gb).getName() << "\n";
        buffer_size = &(*gb);
      }
      //Type* ITy = Type::getInt32Ty(Context);
      //IRBuilder<> IR(ai);
      //LoadBS = IR.CreateLoad(*gb);
      //Constant* AllocSize = ConstantExpr::getSizeOf(Ty);
      //AllocSize = ConstantExpr::getTruncOrBitCast(AllocSize, ITy);
      //Instruction* Malloc = CallInst::CreateMalloc(Builder->GetInsertBlock(),ITy, Ty, AllocSize,nullptr, nullptr, "");
    }
    //Value * v = M->getGlobalVariable("main.__bdx_buffer_size", true);
    //		llvm::errs() << "VAL: " << v << "\n";
    tmp = new std::vector<const Value *>();
    for (Function::iterator BB = F.begin(), E = F.end(); BB != E; BB++) {
      for (BasicBlock::iterator I=BB->begin(), E= BB->end(); I!=E; I++) {
        /*if(isa<InvokeInst>(*I))  {
          InvokeInst *II = dyn_cast<InvokeInst>(&(*I));
          if(isa<Function>(II->getCalledValue())) {
            size_t pos = (II->getCalledFunction())->getName().find("omp_get_thread_num");
            if((pos < (II->getCalledFunction())->getName().size()) && pos > 0) {
              char temp[100];
              int status;
              char * trueName = abi::__cxa_demangle((II->getCalledFunction())->getName().data(), 0, 0, &status);
              //							Function *fID = M->getFunction(trueName);
              //							errs() << trueName << "\n" << fID->getName() << "\n";
            }
          }
        }*/
        if(isa<CallInst>(*I))  {
          CallInst *CI = dyn_cast<CallInst>(&(*I));
          if(isa<Function>(CI->getCalledValue())) {
            if((CI->getCalledFunction())->getName().find("__bdx_stage_begin__")
                < (CI->getCalledFunction())->getName().size()) {
              Value *iterator = CI->getArgOperand(0);
              iterators.push_back(iterator);
              comparations.push_back(CI);
              errs() << "===" <<  *iterator << "\n";
              errs() << "Stage begin on Basic Block: " << &(*BB) << "\n";
              isInsideRegion = true;
              n_component++;
              if(tmp->size() > 0)
                instMap.push_back(*tmp);
              tmp = new std::vector<const Value *>();
              //I++;
              //CI->eraseFromParent();
              continue;
            }
            else if((CI->getCalledFunction())->getName().find("__bdx_stage_end__")
                < (CI->getCalledFunction())->getName().size()) {
              errs() << "Stage end on Basic Block: " << &(*BB) << "\n";
              //isInsideRegion = false;
              comparationsEnd.push_back(CI);
              tmp->push_back(&(*I));
              instMap.push_back(*tmp);
              n_component++;
              tmp = new std::vector<const Value *>();
              //I++;
              //CI->eraseFromParent();
              continue;
            }
          }
        }
        if(isa<ReturnInst>(*I))  {
          ret = &(*I);
        }
        if(isInsideRegion){
          instComps[&(*I)] = n_component;
          //tmp->insert(&(*I));
          tmp->push_back(&(*I));
        }
      }
    }

    int i = 0;
    int buffer = 0;
    Instruction *startF = &(*((F.begin())->begin()));
    auto itEnd = (F.begin())->end();
    itEnd--;
    itEnd--;
    Instruction *endBB1 = &(*itEnd);

    std::set<const Value *> tmp2;

    IRBuilder<> IR(endBB1);
    LoadInst *LoadBS = IR.CreateLoad(buffer_size);
    errs() << "n_component: " << n_component << "\n";
    int k = 0;
    for(auto c = instMap.begin();c != instMap.end();c++, i++) {
      if(k++ % 2 == 1) {continue;}
      errs() << "---------------------------------------------------\n";
      for (auto it = c->begin(); it != c->end(); it++) {
        std::set<const llvm::User *> depsUpd;
        const Value *v = *it;
        const StoreInst *si;

        depsUpd.clear();

        if(isa<StoreInst>(*it))  {
          si = dyn_cast<StoreInst>(*it);
          v = si->getPointerOperand();
        }
        else
          v = *it;

        if ( isa<Argument>(v)  || ignoreInstructions.count(v) != 0) {
          errs() << "v is an argument of omp_outline. (v = " << *v << ")\n";
          // A buffer should not be created for shared variables,
          // because the programmer is responsable to synchronize
          // shared memory accesses
          continue;
        }

        //if(buffers[v]) {continue;}
        std::vector<const User *> uses;
        errs() << *v << ":\n";
        for(auto u = v->users().begin(); u != v->users().end(); u++) {
          uses.push_back(*u);
          errs() << "\t\t" << **u << "\n";
        }

        int compTemp = instComps[*it];

        /*				if(!isa<StoreInst>(*it))
                  errs() << *v << " (" << compTemp << "):\n";
                  else
                  errs() << *si << " (" << instComps[*it] << "):\n";*/
        //				errs() << "V: " << *v << "\n";
        for(auto u = uses.begin(); u != uses.end(); u++) {
          const Value *v2 = *u;

          int compTemp2 = instComps[v2];

          if (compTemp2 == instComps[*it] && compTemp2 != n_component && compTemp2 != 0) {
            //						errs() << "\t\t- " << *v2 << "\n";
            if (compTemp2 % 2 == 1){// && isa<StoreInst>(v2)) {
              if(depsUpd.count(*u) == 0)
                depsUpd.insert(*u);
            }
          }

            if(compTemp2 > instComps[*it] && compTemp2 != n_component && compTemp2 != 0) {
              //						errs() << "\t\t- " << *v2 << "\n";
              if(compTemp2 % 2 == 1) {
                if(tmp2.count(v) == 0) {
                  errs() << *(*it) << " (" << instComps[*it] << ")|(" << compTemp2 << ") " << *v2 << "\n\n";
                  tmp2.insert(v);
                }

                if(deps.count(v) == 0) {
                  std::set<const llvm::User *> tmpDEPS;
                  deps[v] = tmpDEPS;
                }

                if(deps[v].count(*u) == 0)
                  deps[v].insert(*u);


                AllocaInst *ai;
                if ( ! buffers[v] ) {
                  errs() << "\nCreate buffer for " << *v << " in component " << i << "\n\n";
                  Type *ty;
                  if ( isa<StoreInst>(*it) ) {
                    ty = si->getValueOperand()->getType();
                  } else {
                    ty = v->getType();
                  }
                  std::string name = "__bdx_buffer_" + std::to_string(buffer);
                  ai = new AllocaInst(PointerType::get(ty, 0), nullptr , 8, name , startF);
                  Type* ITy = Type::getInt32Ty(F.getContext());
                  //Type* ITy = v2->getType();
                  //Constant* AllocSize = ConstantExpr::getSizeOf(Ty);
                  //Constant *AllocSize = ConstantExpr::getTruncOrBitCast(LoadBS, ITy);
                  Value *li = new LoadInst(ai, "", endBB1);
                  Value *size;
                  if ( ty->isPointerTy() )
                    size = ConstantInt::get(Type::getInt32Ty(v->getContext()), 8);
                  else
                    size = ConstantInt::get(Type::getInt32Ty(v->getContext()),
                        ty->getPrimitiveSizeInBits()/8);
                  errs() << "SIZE: " << *size << "\n";
                  Value * castsize = CastInst::CreateIntegerCast(LoadBS, Type::getInt32Ty(v->getContext()), false, "", endBB1);
                  Value * mult = BinaryOperator::Create(Instruction::BinaryOps::Mul, size , castsize, "", endBB1);
                  Value* Malloc = CallInst::CreateMalloc(endBB1, ITy, ty, mult, nullptr, nullptr, "");
                  Value *si = new StoreInst(Malloc, ai, endBB1);
                  errs() << *Malloc << "\n";
                  buffers[v] = ai;
                  buffer++;
                  //								auto endBB2 = F.end();
                  //								endBB2--;
                  //auto ret = endBB2->end();
                  //ret--;
                  Value *free = CallInst::CreateFree(Malloc, &(*(ret)));
                }


              }
              else {
                std::vector<const User *> uses2;
                for(auto u2 = v2->users().begin(); u2 != v2->users().end(); u2++) {
                  uses2.push_back(*u2);
                }

                for(auto u2 = uses2.begin(); u2 != uses2.end(); u2++) {
                  const Value *v4 = *u2;
                  int compTemp3 = instComps[v4];
                  if((compTemp3 % 2 == 1)){// && compTemp != compTemp3) {
                    if(tmp2.count(v) == 0) {
                      errs() << *(*it) << " (" << instComps[*it] << ")|(" << compTemp2 << ")|(" << compTemp3 << ") " << *v4 << "\n\n";
                      tmp2.insert(v);
                    }

                    if(deps.count(v) == 0) {
                      std::set<const llvm::User *> tmpDEPS;
                      deps[v] = tmpDEPS;
                    }

                    if(deps[v].count(*u2) == 0)
                      deps[v].insert(*u2);


                    AllocaInst *ai;
                    if ( ! buffers[v] ) {
                      errs() << "\nCreate buffer for " << *v << " in component " << i << "\n\n";
                      Type *ty;
                      if ( isa<StoreInst>(*it) ) {
                        ty = si->getValueOperand()->getType();
                      } else {
                        ty = v->getType();
                      }
                      std::string name = "__bdx_buffer_" + std::to_string(buffer);
                      ai = new AllocaInst(PointerType::get(ty, 0), nullptr , 8, name , startF);
                      Type* ITy = Type::getInt32Ty(F.getContext());
                      //Type* ITy = v2->getType();
                      //Constant* AllocSize = ConstantExpr::getSizeOf(Ty);
                      //Constant *AllocSize = ConstantExpr::getTruncOrBitCast(LoadBS, ITy);
                      Value *li = new LoadInst(ai, "", endBB1);
                      Value *size;
                      if ( ty->isPointerTy() )
                        size = ConstantInt::get(Type::getInt32Ty(v->getContext()), 8);
                      else
                        size = ConstantInt::get(Type::getInt32Ty(v->getContext()),
                            ty->getPrimitiveSizeInBits()/8);
                      errs() << "SIZE: " << *size << "\n";
                      Value * castsize = CastInst::CreateIntegerCast(LoadBS, Type::getInt32Ty(v->getContext()), false, "", endBB1);
                      Value * mult = BinaryOperator::Create(Instruction::BinaryOps::Mul, size , castsize, "", endBB1);
                      Value* Malloc = CallInst::CreateMalloc(endBB1, ITy, ty, mult, nullptr, nullptr, "");
                      Value *si = new StoreInst(Malloc, ai, endBB1);
                      errs() << *Malloc << "\n";
                      buffers[v] = ai;
                      buffer++;
                      //										auto endBB2 = F.end();
                      //										endBB2--;
                      //auto ret = endBB2->end();
                      //ret--;
                      Value *free = CallInst::CreateFree(Malloc, &(*(ret)));
                    }
                  }
                  }
                }
              }

              //					if(v2 != nullptr)
              //						errs() << "\t" << *v2 << " (" << compTemp2 << ")\n";

              /*                    int comp = instComps[v2]-1;
                                    if(comp != -1 && i != comp) {
                                    if(deps.count(v) == 0) {
                                    std::set<const llvm::User *> tmpDEPS;
                                    deps[v] = tmpDEPS;
                                    }
              //if(deps[v].count(*u))
              deps[v].insert(*u);
              }

              if(comp != -1 && i == comp && buffers[v]) {
              deps[v].insert(*u);
              }

              //if(isa<StoreInst>(v2)) break;

              AllocaInst *ai;
              if(!buffers[v] && comp != -1 && i != comp) {
              errs() << "\nCreate buffer for " << *v << " in component " << i << "\n\n";
              Type *ty = PointerType::get(v2->getType(), 0);
              std::string name = "__bdx_buffer_" + std::to_string(buffer);
              ai = new AllocaInst(ty, nullptr , 8, name , startF);
              Type* ITy = Type::getInt32Ty(F.getContext());
              //Type* ITy = v2->getType();
              //Constant* AllocSize = ConstantExpr::getSizeOf(Ty);
              //Constant *AllocSize = ConstantExpr::getTruncOrBitCast(LoadBS, ITy);
              Value *li = new LoadInst(ai, "", endBB1);
              Value *size;
              if((v2->getType())->isPointerTy())
              size = ConstantInt::get(Type::getInt32Ty(v->getContext()), 8);
              else
              size = ConstantInt::get(Type::getInt32Ty(v->getContext()), (v2->getType())->getPrimitiveSizeInBits()/8);
              Value * castsize = CastInst::CreateIntegerCast(LoadBS, Type::getInt32Ty(v->getContext()), false, "", endBB1);
              Value * mult = BinaryOperator::Create(Instruction::BinaryOps::Mul, size , castsize, "", endBB1);
              Value* Malloc = CallInst::CreateMalloc(endBB1, ITy, v2->getType(), mult, nullptr, nullptr, "");
              Value *si = new StoreInst(Malloc, ai, endBB1);
              errs() << *Malloc << "\n";
              buffers[v] = ai;
              buffer++;
              auto endBB2 = F.end();
              endBB2--;
              auto ret = endBB2->end();
              ret--;
              Value *free = CallInst::CreateFree(Malloc, &(*(ret)));

              //if(deps[v].count(v3))
              deps[v].insert(v3);
              }*/

              /*					if(buffers[v] && compTemp2 == instComps[*it] && compTemp2 != n_component && compTemp2 != 0) {
                          if(compTemp2 % 2 == 1) {
                          if(deps[v].count(*u) == 0) {
                          deps[v].insert(*u);
                          }
                          }
                          }*/

            }


            //				for(auto u = deps[v].begin(); u != deps[v].end(); u++) {
            //					errs() << *v << " / " << *(*u) << "\n";
            //				}


            if(buffers[v] && done.count(v) == 0) {

              errs() << "COMP: " << *comparations[compTemp-1] << "\n";

              Value *gep;
              LoadInst *loadbuffer, *loadbuffer2;
              auto st = it;
              st++;
              Instruction *st2 = const_cast<Instruction *>(dyn_cast<Instruction>(*st));

              Value *innerIterator = iterators[compTemp/2];
              BinaryOperator *bi = BinaryOperator::Create(Instruction::BinaryOps::SRem, innerIterator, LoadBS, "", const_cast<Instruction *>(dyn_cast<Instruction>(comparations[compTemp/2])));
              SExtInst *innerIteratorSEXT;
              std::vector<Value *> idxArray;
              if(bi->getType() != Type::getInt64Ty(v->getContext())) {
                innerIteratorSEXT = new SExtInst(bi, Type::getInt64Ty(v->getContext()), "", const_cast<Instruction *>(dyn_cast<Instruction>(comparations[compTemp/2])));
                idxArray = {
                  //ConstantInt::get(Type::getInt32Ty(v->getContext()), 0),
                  innerIteratorSEXT};
              }
              else {
                idxArray = {
                  //ConstantInt::get(Type::getInt32Ty(v->getContext()), 0),
                  bi};
              }
              loadbuffer = new LoadInst(buffers[v], "", false, 8, const_cast<Instruction *>(dyn_cast<Instruction>(comparations[compTemp/2])));
              gep = GetElementPtrInst::CreateInBounds(loadbuffer, idxArray, "", const_cast<Instruction *>(dyn_cast<Instruction>(comparations[compTemp/2])));

              loads[v][compTemp/2] = gep;
              errs() << "ACESSING BUFFER: " << *v << " in component " << compTemp << " after instruction" << *(*st) << "\n";

              depsComp[compTemp/2].insert(v);
              //const_cast<Value *>(v)->replaceAllUsesWith(loadbuffer2);
              //if(!isa<StoreInst>(*it))  {
              //Value *sti = new StoreInst(const_cast<Value *>(v), gep, false, loadbuffer2);
              //					Value *sti = new StoreInst(const_cast<Value *>(v), gep, false, const_cast<Instruction *>(dyn_cast<Instruction>(comparationsEnd[compTemp/2])));

              for(auto u = depsUpd.begin(); u != depsUpd.end(); u++) {
                const Value *v2 = *u;
                User *v3 = const_cast<User *>(*u);
                v3->replaceUsesOfWith(const_cast<Value *>(v), loads[v][compTemp/2]);

                //StoreInst *siUpd = const_cast<StoreInst *>(dyn_cast<StoreInst>(v2));
                //Value *val = siUpd->getValueOperand();


                //Value *sti = new StoreInst(val, loads[v][compTemp/2], false, siUpd);
              }

              done.insert(v);
              //}
              //				Value *sti = IR.CreateStore(const_cast<Value *>(v), gep, false);
            }

            //				errs() << "\n\n";

            std::set<int> used;
            std::map<int, Value *> geps;

            LoadInst *loadbuffer, *loadbuffer2;
            for(auto u = deps[v].begin(); u != deps[v].end(); u++) {
              const Value *v2 = *u;
              User *v3 = const_cast<User *>(*u);
              GetElementPtrInst *gep;

              int comp = instComps[v2];
              //                    if(used.count(comp/2) == 0) {
              if(depsComp[comp/2].count(v) == 0) {

                used.insert(comp/2);
                depsComp[comp/2].insert(v);

                Value *innerIterator = iterators[comp/2];
                BinaryOperator *bi = BinaryOperator::Create(Instruction::BinaryOps::SRem, innerIterator, LoadBS, "", const_cast<Instruction *>(dyn_cast<Instruction>(comparations[comp/2])));
                SExtInst *innerIteratorSEXT;
                std::vector<Value *> idxArray;
                if(bi->getType() != Type::getInt64Ty(v->getContext())) {
                  innerIteratorSEXT = new SExtInst(bi, Type::getInt64Ty(v->getContext()), "", const_cast<Instruction *>(dyn_cast<Instruction>(comparations[comp/2])));
                  idxArray = {
                    //ConstantInt::get(Type::getInt32Ty(v->getContext()), 0),
                    innerIteratorSEXT};
                }
                else {
                  idxArray = {
                    //ConstantInt::get(Type::getInt32Ty(v->getContext()), 0),
                    bi};
                }
                loadbuffer = new LoadInst(buffers[v], "", false, 8, const_cast<Instruction *>(dyn_cast<Instruction>(comparations[comp/2])));
                //loadbuffer2 = new LoadInst(loadbuffer, "", false, 8, const_cast<Instruction *>(dyn_cast<Instruction>(comparations[comp/2])));
                //errs() << *loadbuffer << " | " << *loadbuffer2 << "\n";
                //                        gep = GetElementPtrInst::CreateInBounds(v2->getType(), loadbuffer2, tmpV, "", const_cast<Instruction *>(dyn_cast<Instruction>(comparations[comp])));
                gep = GetElementPtrInst::CreateInBounds(loadbuffer, idxArray, "", const_cast<Instruction *>(dyn_cast<Instruction>(comparations[comp/2])));

                //						gep = GetElementPtrInst::CreateInBounds(loadbuffer, tmpV, "", const_cast<Instruction *>(dyn_cast<Instruction>(comparations[comp])));
                //gep = dyn_cast<GetElementPtrInst>(IR.CreateInBoundsGEP(loadbuffer2, innerIteratorSEXT, ""));
                loads[v][comp/2] = gep;
              }


              errs() << "REPLACING: " << *v << " in " << *v2 << " by " << *loads[v][comp/2] << "\n";

              //v3->replaceUsesOfWith(const_cast<Value *>(v), geps[comp/2]);
              v3->replaceUsesOfWith(const_cast<Value *>(v), loads[v][comp/2]);

              errs() << "REPLACED: " << *v3 << "\n";
            }
            }
          }
          F.dump();
        }



        return false;
}

//**********************************************************************
// print (do not change this method)
//
// If this pass is run with -f -analyze, this method will be called
// after each call to runOnFunction.
//**********************************************************************
void OMPUseDependenceAnalysis::print(std::ostream &O, const Module *M) const {
    O << "This is printCode.\n";
}


char OMPUseDependenceAnalysis::ID = 0;

// register the printCode class:
//  - give it a command-line argument
//  - a name
//  - a flag saying that we don't modify the CFG
//  - a flag saying this is not an analysis pass
INITIALIZE_PASS(OMPUseDependenceAnalysis, "use-transforms", "OMPUseDependenceAnalysis analysis for 'use' clause",
	false, false)

namespace llvm {

FunctionPass *createOMPUseDependenceAnalysisPass() {
  return new OMPUseDependenceAnalysis();
}

} // End llvm namespace
