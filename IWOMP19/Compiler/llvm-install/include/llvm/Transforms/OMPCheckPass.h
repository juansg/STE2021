#ifndef LLVM_TRANSFORMS_OMP_CHECK_PASS_H
#define LLVM_TRANSFORMS_OMP_CHECK_PASS_H

namespace llvm {

class Pass;

Pass *createOMPCheckPass();

} // End llvm namespace

#endif
