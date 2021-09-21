#ifndef LLVM_TRANSFORMS_OMP_USE_PASS_H
#define LLVM_TRANSFORMS_OMP_USE_PASS_H

namespace llvm {

class FunctionPass;

FunctionPass *createOMPUseDependenceAnalysisPass();

} // End llvm namespace

#endif /* LLVM_TRANSFORMS_OMP_USE_PASS_H */
