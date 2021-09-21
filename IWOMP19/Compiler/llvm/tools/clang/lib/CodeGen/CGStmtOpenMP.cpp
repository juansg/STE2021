//===--- CGStmtOpenMP.cpp - Emit LLVM Code from Statements ----------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This contains code to emit OpenMP nodes as LLVM code.
//
//===----------------------------------------------------------------------===//

#include "CGCleanup.h"
#include "CGOpenMPRuntime.h"
#include "CodeGenFunction.h"
#include "CodeGenModule.h"
#include "TargetInfo.h"
#include "clang/AST/Stmt.h"
#include "clang/AST/StmtOpenMP.h"
#include "clang/AST/DeclOpenMP.h"
#include "clang/Rewrite/Core/Rewriter.h"
#include "llvm/IR/CallSite.h"
#include "clang/Frontend/CompilerInstance.h"
#include "llvm/Support/ThreadPool.h"
#include "llvm/Support/Host.h"
#include "clang/Basic/TargetBuiltins.h"


using namespace clang;
using namespace CodeGen;

namespace {
/// Lexical scope for OpenMP executable constructs, that handles correct codegen
/// for captured expressions.
class OMPLexicalScope final : public CodeGenFunction::LexicalScope {
  void emitPreInitStmt(CodeGenFunction &CGF, const OMPExecutableDirective &S) {
    for (const auto *C : S.clauses()) {
      if (auto *CPI = OMPClauseWithPreInit::get(C)) {
        if (auto *PreInit = cast_or_null<DeclStmt>(CPI->getPreInitStmt())) {
          for (const auto *I : PreInit->decls()) {
            if (!I->hasAttr<OMPCaptureNoInitAttr>())
              CGF.EmitVarDecl(cast<VarDecl>(*I));
            else {
              CodeGenFunction::AutoVarEmission Emission =
                  CGF.EmitAutoVarAlloca(cast<VarDecl>(*I));
              CGF.EmitAutoVarCleanups(Emission);
            }
          }
        }
      }
    }
  }
  CodeGenFunction::OMPPrivateScope InlinedShareds;

  static bool isCapturedVar(CodeGenFunction &CGF, const VarDecl *VD) {
    return CGF.LambdaCaptureFields.lookup(VD) ||
           (CGF.CapturedStmtInfo && CGF.CapturedStmtInfo->lookup(VD)) ||
           (CGF.CurCodeDecl && isa<BlockDecl>(CGF.CurCodeDecl));
  }

public:
  OMPLexicalScope(CodeGenFunction &CGF, const OMPExecutableDirective &S,
                  bool AsInlined = false)
      : CodeGenFunction::LexicalScope(CGF, S.getSourceRange()),
        InlinedShareds(CGF) {
    emitPreInitStmt(CGF, S);
    if (AsInlined) {
      if (S.hasAssociatedStmt()) {
        auto *CS = cast<CapturedStmt>(S.getAssociatedStmt());
        for (auto &C : CS->captures()) {
          if (C.capturesVariable() || C.capturesVariableByCopy()) {
            auto *VD = C.getCapturedVar();
            DeclRefExpr DRE(const_cast<VarDecl *>(VD),
                            isCapturedVar(CGF, VD) ||
                                (CGF.CapturedStmtInfo &&
                                 InlinedShareds.isGlobalVarCaptured(VD)),
                            VD->getType().getNonReferenceType(), VK_LValue,
                            SourceLocation());
            InlinedShareds.addPrivate(VD, [&CGF, &DRE]() -> Address {
              return CGF.EmitLValue(&DRE).getAddress();
            });
          }
        }
        (void)InlinedShareds.Privatize();
      }
    }
  }
};

/// Private scope for OpenMP loop-based directives, that supports capturing
/// of used expression from loop statement.
class OMPLoopScope : public CodeGenFunction::RunCleanupsScope {
  void emitPreInitStmt(CodeGenFunction &CGF, const OMPLoopDirective &S) {
    if (auto *LD = dyn_cast<OMPLoopDirective>(&S)) {
      if (auto *PreInits = cast_or_null<DeclStmt>(LD->getPreInits())) {
        for (const auto *I : PreInits->decls())
          CGF.EmitVarDecl(cast<VarDecl>(*I));
      }
    }
  }

public:
  OMPLoopScope(CodeGenFunction &CGF, const OMPLoopDirective &S)
      : CodeGenFunction::RunCleanupsScope(CGF) {
    emitPreInitStmt(CGF, S);
  }
};

} // namespace

llvm::Value *CodeGenFunction::getTypeSize(QualType Ty) {
  auto &C = getContext();
  llvm::Value *Size = nullptr;
  auto SizeInChars = C.getTypeSizeInChars(Ty);
  if (SizeInChars.isZero()) {
    // getTypeSizeInChars() returns 0 for a VLA.
    while (auto *VAT = C.getAsVariableArrayType(Ty)) {
      llvm::Value *ArraySize;
      std::tie(ArraySize, Ty) = getVLASize(VAT);
      Size = Size ? Builder.CreateNUWMul(Size, ArraySize) : ArraySize;
    }
    SizeInChars = C.getTypeSizeInChars(Ty);
    if (SizeInChars.isZero())
      return llvm::ConstantInt::get(SizeTy, /*V=*/0);
    Size = Builder.CreateNUWMul(Size, CGM.getSize(SizeInChars));
  } else
    Size = CGM.getSize(SizeInChars);
  return Size;
}

void CodeGenFunction::GenerateOpenMPCapturedVars(
    const CapturedStmt &S, SmallVectorImpl<llvm::Value *> &CapturedVars) {
  const RecordDecl *RD = S.getCapturedRecordDecl();
  auto CurField = RD->field_begin();
  auto CurCap = S.captures().begin();
  for (CapturedStmt::const_capture_init_iterator I = S.capture_init_begin(),
                                                 E = S.capture_init_end();
       I != E; ++I, ++CurField, ++CurCap) {
    if (CurField->hasCapturedVLAType()) {
      auto VAT = CurField->getCapturedVLAType();
      auto *Val = VLASizeMap[VAT->getSizeExpr()];
      CapturedVars.push_back(Val);
    } else if (CurCap->capturesThis())
      CapturedVars.push_back(CXXThisValue);
    else if (CurCap->capturesVariableByCopy()) {
      llvm::Value *CV =
          EmitLoadOfLValue(EmitLValue(*I), SourceLocation()).getScalarVal();

      // If the field is not a pointer, we need to save the actual value
      // and load it as a void pointer.
      if (!CurField->getType()->isAnyPointerType()) {
        auto &Ctx = getContext();
        auto DstAddr = CreateMemTemp(
            Ctx.getUIntPtrType(),
            Twine(CurCap->getCapturedVar()->getName()) + ".casted");
        LValue DstLV = MakeAddrLValue(DstAddr, Ctx.getUIntPtrType());

        auto *SrcAddrVal = EmitScalarConversion(
            DstAddr.getPointer(), Ctx.getPointerType(Ctx.getUIntPtrType()),
            Ctx.getPointerType(CurField->getType()), SourceLocation());
        LValue SrcLV =
            MakeNaturalAlignAddrLValue(SrcAddrVal, CurField->getType());

        // Store the value using the source type pointer.
        EmitStoreThroughLValue(RValue::get(CV), SrcLV);

        // Load the value using the destination type pointer.
        CV = EmitLoadOfLValue(DstLV, SourceLocation()).getScalarVal();
      }
      CapturedVars.push_back(CV);
    } else {
      assert(CurCap->capturesVariable() && "Expected capture by reference.");
      CapturedVars.push_back(EmitLValue(*I).getAddress().getPointer());
    }
  }
}

static Address castValueFromUintptr(CodeGenFunction &CGF, QualType DstType,
                                    StringRef Name, LValue AddrLV,
                                    bool isReferenceType = false) {
  ASTContext &Ctx = CGF.getContext();

  auto *CastedPtr = CGF.EmitScalarConversion(
      AddrLV.getAddress().getPointer(), Ctx.getUIntPtrType(),
      Ctx.getPointerType(DstType), SourceLocation());
  auto TmpAddr =
      CGF.MakeNaturalAlignAddrLValue(CastedPtr, Ctx.getPointerType(DstType))
          .getAddress();

  // If we are dealing with references we need to return the address of the
  // reference instead of the reference of the value.
  if (isReferenceType) {
    QualType RefType = Ctx.getLValueReferenceType(DstType);
    auto *RefVal = TmpAddr.getPointer();
    TmpAddr = CGF.CreateMemTemp(RefType, Twine(Name) + ".ref");
    auto TmpLVal = CGF.MakeAddrLValue(TmpAddr, RefType);
    CGF.EmitScalarInit(RefVal, TmpLVal);
  }

  return TmpAddr;
}

llvm::Function *
CodeGenFunction::GenerateOpenMPCapturedStmtFunction(const CapturedStmt &S) {
  assert(
      CapturedStmtInfo &&
      "CapturedStmtInfo should be set when generating the captured function");
  const CapturedDecl *CD = S.getCapturedDecl();
  const RecordDecl *RD = S.getCapturedRecordDecl();
  assert(CD->hasBody() && "missing CapturedDecl body");

  // Build the argument list.
  ASTContext &Ctx = CGM.getContext();
  FunctionArgList Args;
  Args.append(CD->param_begin(),
              std::next(CD->param_begin(), CD->getContextParamPosition()));
  auto I = S.captures().begin();
  for (auto *FD : RD->fields()) {
    QualType ArgType = FD->getType();
    IdentifierInfo *II = nullptr;
    VarDecl *CapVar = nullptr;

    // If this is a capture by copy and the type is not a pointer, the outlined
    // function argument type should be uintptr and the value properly casted to
    // uintptr. This is necessary given that the runtime library is only able to
    // deal with pointers. We can pass in the same way the VLA type sizes to the
    // outlined function.
    if ((I->capturesVariableByCopy() && !ArgType->isAnyPointerType()) ||
        I->capturesVariableArrayType())
      ArgType = Ctx.getUIntPtrType();

    if (I->capturesVariable() || I->capturesVariableByCopy()) {
      CapVar = I->getCapturedVar();
      II = CapVar->getIdentifier();
    } else if (I->capturesThis())
      II = &getContext().Idents.get("this");
    else {
      assert(I->capturesVariableArrayType());
      II = &getContext().Idents.get("vla");
    }
    if (ArgType->isVariablyModifiedType())
      ArgType = getContext().getVariableArrayDecayedType(ArgType);
    Args.push_back(ImplicitParamDecl::Create(getContext(), nullptr,
                                             FD->getLocation(), II, ArgType));
    ++I;
  }
  Args.append(
      std::next(CD->param_begin(), CD->getContextParamPosition() + 1),
      CD->param_end());

  // Create the function declaration.
  FunctionType::ExtInfo ExtInfo;
  const CGFunctionInfo &FuncInfo =
      CGM.getTypes().arrangeBuiltinFunctionDeclaration(Ctx.VoidTy, Args);
  llvm::FunctionType *FuncLLVMTy = CGM.getTypes().GetFunctionType(FuncInfo);

  llvm::Function *F = llvm::Function::Create(
      FuncLLVMTy, llvm::GlobalValue::InternalLinkage,
      CapturedStmtInfo->getHelperName(), &CGM.getModule());
  CGM.SetInternalFunctionAttributes(CD, F, FuncInfo);
  if (CD->isNothrow())
    F->addFnAttr(llvm::Attribute::NoUnwind);

  // Generate the function.
  StartFunction(CD, Ctx.VoidTy, F, FuncInfo, Args, CD->getLocation(),
                CD->getBody()->getLocStart());
  unsigned Cnt = CD->getContextParamPosition();
  I = S.captures().begin();
  for (auto *FD : RD->fields()) {
    // If we are capturing a pointer by copy we don't need to do anything, just
    // use the value that we get from the arguments.
    if (I->capturesVariableByCopy() && FD->getType()->isAnyPointerType()) {
      setAddrOfLocalVar(I->getCapturedVar(), GetAddrOfLocalVar(Args[Cnt]));
      ++Cnt;
      ++I;
      continue;
    }

    LValue ArgLVal =
        MakeAddrLValue(GetAddrOfLocalVar(Args[Cnt]), Args[Cnt]->getType(),
                       AlignmentSource::Decl);
    if (FD->hasCapturedVLAType()) {
      LValue CastedArgLVal =
          MakeAddrLValue(castValueFromUintptr(*this, FD->getType(),
                                              Args[Cnt]->getName(), ArgLVal),
                         FD->getType(), AlignmentSource::Decl);
      auto *ExprArg =
          EmitLoadOfLValue(CastedArgLVal, SourceLocation()).getScalarVal();
      auto VAT = FD->getCapturedVLAType();
      VLASizeMap[VAT->getSizeExpr()] = ExprArg;
    } else if (I->capturesVariable()) {
      auto *Var = I->getCapturedVar();
      QualType VarTy = Var->getType();
      Address ArgAddr = ArgLVal.getAddress();
      if (!VarTy->isReferenceType()) {
        ArgAddr = EmitLoadOfReference(
            ArgAddr, ArgLVal.getType()->castAs<ReferenceType>());
      }
      setAddrOfLocalVar(
          Var, Address(ArgAddr.getPointer(), getContext().getDeclAlign(Var)));
    } else if (I->capturesVariableByCopy()) {
      assert(!FD->getType()->isAnyPointerType() &&
             "Not expecting a captured pointer.");
      auto *Var = I->getCapturedVar();
      QualType VarTy = Var->getType();
      setAddrOfLocalVar(Var, castValueFromUintptr(*this, FD->getType(),
                                                  Args[Cnt]->getName(), ArgLVal,
                                                  VarTy->isReferenceType()));
    } else {
      // If 'this' is captured, load it into CXXThisValue.
      assert(I->capturesThis());
      CXXThisValue =
          EmitLoadOfLValue(ArgLVal, Args[Cnt]->getLocation()).getScalarVal();
    }
    ++Cnt;
    ++I;
  }

  PGO.assignRegionCounters(GlobalDecl(CD), F);
  CapturedStmtInfo->EmitBody(*this, CD->getBody());
  FinishFunction(CD->getBodyRBrace());

  return F;
}

//===----------------------------------------------------------------------===//
//                              OpenMP Directive Emission
//===----------------------------------------------------------------------===//
void CodeGenFunction::EmitOMPAggregateAssign(
    Address DestAddr, Address SrcAddr, QualType OriginalType,
    const llvm::function_ref<void(Address, Address)> &CopyGen) {
  // Perform element-by-element initialization.
  QualType ElementTy;

  // Drill down to the base element type on both arrays.
  auto ArrayTy = OriginalType->getAsArrayTypeUnsafe();
  auto NumElements = emitArrayLength(ArrayTy, ElementTy, DestAddr);
  SrcAddr = Builder.CreateElementBitCast(SrcAddr, DestAddr.getElementType());

  auto SrcBegin = SrcAddr.getPointer();
  auto DestBegin = DestAddr.getPointer();
  // Cast from pointer to array type to pointer to single element.
  auto DestEnd = Builder.CreateGEP(DestBegin, NumElements);
  // The basic structure here is a while-do loop.
  auto BodyBB = createBasicBlock("omp.arraycpy.body");
  auto DoneBB = createBasicBlock("omp.arraycpy.done");
  auto IsEmpty =
      Builder.CreateICmpEQ(DestBegin, DestEnd, "omp.arraycpy.isempty");
  Builder.CreateCondBr(IsEmpty, DoneBB, BodyBB);

  // Enter the loop body, making that address the current address.
  auto EntryBB = Builder.GetInsertBlock();
  EmitBlock(BodyBB);

  CharUnits ElementSize = getContext().getTypeSizeInChars(ElementTy);

  llvm::PHINode *SrcElementPHI =
    Builder.CreatePHI(SrcBegin->getType(), 2, "omp.arraycpy.srcElementPast");
  SrcElementPHI->addIncoming(SrcBegin, EntryBB);
  Address SrcElementCurrent =
      Address(SrcElementPHI,
              SrcAddr.getAlignment().alignmentOfArrayElement(ElementSize));

  llvm::PHINode *DestElementPHI =
    Builder.CreatePHI(DestBegin->getType(), 2, "omp.arraycpy.destElementPast");
  DestElementPHI->addIncoming(DestBegin, EntryBB);
  Address DestElementCurrent =
    Address(DestElementPHI,
            DestAddr.getAlignment().alignmentOfArrayElement(ElementSize));

  // Emit copy.
  CopyGen(DestElementCurrent, SrcElementCurrent);

  // Shift the address forward by one element.
  auto DestElementNext = Builder.CreateConstGEP1_32(
      DestElementPHI, /*Idx0=*/1, "omp.arraycpy.dest.element");
  auto SrcElementNext = Builder.CreateConstGEP1_32(
      SrcElementPHI, /*Idx0=*/1, "omp.arraycpy.src.element");
  // Check whether we've reached the end.
  auto Done =
      Builder.CreateICmpEQ(DestElementNext, DestEnd, "omp.arraycpy.done");
  Builder.CreateCondBr(Done, DoneBB, BodyBB);
  DestElementPHI->addIncoming(DestElementNext, Builder.GetInsertBlock());
  SrcElementPHI->addIncoming(SrcElementNext, Builder.GetInsertBlock());

  // Done.
  EmitBlock(DoneBB, /*IsFinished=*/true);
}

/// Check if the combiner is a call to UDR combiner and if it is so return the
/// UDR decl used for reduction.
static const OMPDeclareReductionDecl *
getReductionInit(const Expr *ReductionOp) {
  if (auto *CE = dyn_cast<CallExpr>(ReductionOp))
    if (auto *OVE = dyn_cast<OpaqueValueExpr>(CE->getCallee()))
      if (auto *DRE =
              dyn_cast<DeclRefExpr>(OVE->getSourceExpr()->IgnoreImpCasts()))
        if (auto *DRD = dyn_cast<OMPDeclareReductionDecl>(DRE->getDecl()))
          return DRD;
  return nullptr;
}

static void emitInitWithReductionInitializer(CodeGenFunction &CGF,
                                             const OMPDeclareReductionDecl *DRD,
                                             const Expr *InitOp,
                                             Address Private, Address Original,
                                             QualType Ty) {
  if (DRD->getInitializer()) {
    std::pair<llvm::Function *, llvm::Function *> Reduction =
        CGF.CGM.getOpenMPRuntime().getUserDefinedReduction(DRD);
    auto *CE = cast<CallExpr>(InitOp);
    auto *OVE = cast<OpaqueValueExpr>(CE->getCallee());
    const Expr *LHS = CE->getArg(/*Arg=*/0)->IgnoreParenImpCasts();
    const Expr *RHS = CE->getArg(/*Arg=*/1)->IgnoreParenImpCasts();
    auto *LHSDRE = cast<DeclRefExpr>(cast<UnaryOperator>(LHS)->getSubExpr());
    auto *RHSDRE = cast<DeclRefExpr>(cast<UnaryOperator>(RHS)->getSubExpr());
    CodeGenFunction::OMPPrivateScope PrivateScope(CGF);
    PrivateScope.addPrivate(cast<VarDecl>(LHSDRE->getDecl()),
                            [=]() -> Address { return Private; });
    PrivateScope.addPrivate(cast<VarDecl>(RHSDRE->getDecl()),
                            [=]() -> Address { return Original; });
    (void)PrivateScope.Privatize();
    RValue Func = RValue::get(Reduction.second);
    CodeGenFunction::OpaqueValueMapping Map(CGF, OVE, Func);
    CGF.EmitIgnoredExpr(InitOp);
  } else {
    llvm::Constant *Init = CGF.CGM.EmitNullConstant(Ty);
    auto *GV = new llvm::GlobalVariable(
        CGF.CGM.getModule(), Init->getType(), /*isConstant=*/true,
        llvm::GlobalValue::PrivateLinkage, Init, ".init");
    LValue LV = CGF.MakeNaturalAlignAddrLValue(GV, Ty);
    RValue InitRVal;
    switch (CGF.getEvaluationKind(Ty)) {
    case TEK_Scalar:
      InitRVal = CGF.EmitLoadOfLValue(LV, SourceLocation());
      break;
    case TEK_Complex:
      InitRVal =
          RValue::getComplex(CGF.EmitLoadOfComplex(LV, SourceLocation()));
      break;
    case TEK_Aggregate:
      InitRVal = RValue::getAggregate(LV.getAddress());
      break;
    }
    OpaqueValueExpr OVE(SourceLocation(), Ty, VK_RValue);
    CodeGenFunction::OpaqueValueMapping OpaqueMap(CGF, &OVE, InitRVal);
    CGF.EmitAnyExprToMem(&OVE, Private, Ty.getQualifiers(),
                         /*IsInitializer=*/false);
  }
}

/// \brief Emit initialization of arrays of complex types.
/// \param DestAddr Address of the array.
/// \param Type Type of array.
/// \param Init Initial expression of array.
/// \param SrcAddr Address of the original array.
static void EmitOMPAggregateInit(CodeGenFunction &CGF, Address DestAddr,
                                 QualType Type, const Expr *Init,
                                 Address SrcAddr = Address::invalid()) {
  auto *DRD = getReductionInit(Init);
  // Perform element-by-element initialization.
  QualType ElementTy;

  // Drill down to the base element type on both arrays.
  auto ArrayTy = Type->getAsArrayTypeUnsafe();
  auto NumElements = CGF.emitArrayLength(ArrayTy, ElementTy, DestAddr);
  DestAddr =
      CGF.Builder.CreateElementBitCast(DestAddr, DestAddr.getElementType());
  if (DRD)
    SrcAddr =
        CGF.Builder.CreateElementBitCast(SrcAddr, DestAddr.getElementType());

  llvm::Value *SrcBegin = nullptr;
  if (DRD)
    SrcBegin = SrcAddr.getPointer();
  auto DestBegin = DestAddr.getPointer();
  // Cast from pointer to array type to pointer to single element.
  auto DestEnd = CGF.Builder.CreateGEP(DestBegin, NumElements);
  // The basic structure here is a while-do loop.
  auto BodyBB = CGF.createBasicBlock("omp.arrayinit.body");
  auto DoneBB = CGF.createBasicBlock("omp.arrayinit.done");
  auto IsEmpty =
      CGF.Builder.CreateICmpEQ(DestBegin, DestEnd, "omp.arrayinit.isempty");
  CGF.Builder.CreateCondBr(IsEmpty, DoneBB, BodyBB);

  // Enter the loop body, making that address the current address.
  auto EntryBB = CGF.Builder.GetInsertBlock();
  CGF.EmitBlock(BodyBB);

  CharUnits ElementSize = CGF.getContext().getTypeSizeInChars(ElementTy);

  llvm::PHINode *SrcElementPHI = nullptr;
  Address SrcElementCurrent = Address::invalid();
  if (DRD) {
    SrcElementPHI = CGF.Builder.CreatePHI(SrcBegin->getType(), 2,
                                          "omp.arraycpy.srcElementPast");
    SrcElementPHI->addIncoming(SrcBegin, EntryBB);
    SrcElementCurrent =
        Address(SrcElementPHI,
                SrcAddr.getAlignment().alignmentOfArrayElement(ElementSize));
  }
  llvm::PHINode *DestElementPHI = CGF.Builder.CreatePHI(
      DestBegin->getType(), 2, "omp.arraycpy.destElementPast");
  DestElementPHI->addIncoming(DestBegin, EntryBB);
  Address DestElementCurrent =
      Address(DestElementPHI,
              DestAddr.getAlignment().alignmentOfArrayElement(ElementSize));

  // Emit copy.
  {
    CodeGenFunction::RunCleanupsScope InitScope(CGF);
    if (DRD && (DRD->getInitializer() || !Init)) {
      emitInitWithReductionInitializer(CGF, DRD, Init, DestElementCurrent,
                                       SrcElementCurrent, ElementTy);
    } else
      CGF.EmitAnyExprToMem(Init, DestElementCurrent, ElementTy.getQualifiers(),
                           /*IsInitializer=*/false);
  }

  if (DRD) {
    // Shift the address forward by one element.
    auto SrcElementNext = CGF.Builder.CreateConstGEP1_32(
        SrcElementPHI, /*Idx0=*/1, "omp.arraycpy.dest.element");
    SrcElementPHI->addIncoming(SrcElementNext, CGF.Builder.GetInsertBlock());
  }

  // Shift the address forward by one element.
  auto DestElementNext = CGF.Builder.CreateConstGEP1_32(
      DestElementPHI, /*Idx0=*/1, "omp.arraycpy.dest.element");
  // Check whether we've reached the end.
  auto Done =
      CGF.Builder.CreateICmpEQ(DestElementNext, DestEnd, "omp.arraycpy.done");
  CGF.Builder.CreateCondBr(Done, DoneBB, BodyBB);
  DestElementPHI->addIncoming(DestElementNext, CGF.Builder.GetInsertBlock());

  // Done.
  CGF.EmitBlock(DoneBB, /*IsFinished=*/true);
}

void CodeGenFunction::EmitOMPCopy(QualType OriginalType, Address DestAddr,
                                  Address SrcAddr, const VarDecl *DestVD,
                                  const VarDecl *SrcVD, const Expr *Copy) {
  if (OriginalType->isArrayType()) {
    auto *BO = dyn_cast<BinaryOperator>(Copy);
    if (BO && BO->getOpcode() == BO_Assign) {
      // Perform simple memcpy for simple copying.
      EmitAggregateAssign(DestAddr, SrcAddr, OriginalType);
    } else {
      // For arrays with complex element types perform element by element
      // copying.
      EmitOMPAggregateAssign(
          DestAddr, SrcAddr, OriginalType,
          [this, Copy, SrcVD, DestVD](Address DestElement, Address SrcElement) {
            // Working with the single array element, so have to remap
            // destination and source variables to corresponding array
            // elements.
            CodeGenFunction::OMPPrivateScope Remap(*this);
            Remap.addPrivate(DestVD, [DestElement]() -> Address {
              return DestElement;
            });
            Remap.addPrivate(
                SrcVD, [SrcElement]() -> Address { return SrcElement; });
            (void)Remap.Privatize();
            EmitIgnoredExpr(Copy);
          });
    }
  } else {
    // Remap pseudo source variable to private copy.
    CodeGenFunction::OMPPrivateScope Remap(*this);
    Remap.addPrivate(SrcVD, [SrcAddr]() -> Address { return SrcAddr; });
    Remap.addPrivate(DestVD, [DestAddr]() -> Address { return DestAddr; });
    (void)Remap.Privatize();
    // Emit copying of the whole variable.
    EmitIgnoredExpr(Copy);
  }
}

bool CodeGenFunction::EmitOMPFirstprivateClause(const OMPExecutableDirective &D,
                                                OMPPrivateScope &PrivateScope) {
  if (!HaveInsertPoint())
    return false;
  bool FirstprivateIsLastprivate = false;
  llvm::DenseSet<const VarDecl *> Lastprivates;
  for (const auto *C : D.getClausesOfKind<OMPLastprivateClause>()) {
    for (const auto *D : C->varlists())
      Lastprivates.insert(
          cast<VarDecl>(cast<DeclRefExpr>(D)->getDecl())->getCanonicalDecl());
  }
  llvm::DenseSet<const VarDecl *> EmittedAsFirstprivate;
  CGCapturedStmtInfo CapturesInfo(cast<CapturedStmt>(*D.getAssociatedStmt()));
  for (const auto *C : D.getClausesOfKind<OMPFirstprivateClause>()) {
    auto IRef = C->varlist_begin();
    auto InitsRef = C->inits().begin();
    for (auto IInit : C->private_copies()) {
      auto *OrigVD = cast<VarDecl>(cast<DeclRefExpr>(*IRef)->getDecl());
      bool ThisFirstprivateIsLastprivate =
          Lastprivates.count(OrigVD->getCanonicalDecl()) > 0;
      auto *CapFD = CapturesInfo.lookup(OrigVD);
      auto *FD = CapturedStmtInfo->lookup(OrigVD);
      if (!ThisFirstprivateIsLastprivate && FD && (FD == CapFD) &&
          !FD->getType()->isReferenceType()) {
        EmittedAsFirstprivate.insert(OrigVD->getCanonicalDecl());
        ++IRef;
        ++InitsRef;
        continue;
      }
      FirstprivateIsLastprivate =
          FirstprivateIsLastprivate || ThisFirstprivateIsLastprivate;
      if (EmittedAsFirstprivate.insert(OrigVD->getCanonicalDecl()).second) {
        auto *VD = cast<VarDecl>(cast<DeclRefExpr>(IInit)->getDecl());
        auto *VDInit = cast<VarDecl>(cast<DeclRefExpr>(*InitsRef)->getDecl());
        bool IsRegistered;
        DeclRefExpr DRE(const_cast<VarDecl *>(OrigVD),
                        /*RefersToEnclosingVariableOrCapture=*/FD != nullptr,
                        (*IRef)->getType(), VK_LValue, (*IRef)->getExprLoc());
        Address OriginalAddr = EmitLValue(&DRE).getAddress();
        QualType Type = VD->getType();
        if (Type->isArrayType()) {
          // Emit VarDecl with copy init for arrays.
          // Get the address of the original variable captured in current
          // captured region.
          IsRegistered = PrivateScope.addPrivate(OrigVD, [&]() -> Address {
            auto Emission = EmitAutoVarAlloca(*VD);
            auto *Init = VD->getInit();
            if (!isa<CXXConstructExpr>(Init) || isTrivialInitializer(Init)) {
              // Perform simple memcpy.
              EmitAggregateAssign(Emission.getAllocatedAddress(), OriginalAddr,
                                  Type);
            } else {
              EmitOMPAggregateAssign(
                  Emission.getAllocatedAddress(), OriginalAddr, Type,
                  [this, VDInit, Init](Address DestElement,
                                       Address SrcElement) {
                    // Clean up any temporaries needed by the initialization.
                    RunCleanupsScope InitScope(*this);
                    // Emit initialization for single element.
                    setAddrOfLocalVar(VDInit, SrcElement);
                    EmitAnyExprToMem(Init, DestElement,
                                     Init->getType().getQualifiers(),
                                     /*IsInitializer*/ false);
                    LocalDeclMap.erase(VDInit);
                  });
            }
            EmitAutoVarCleanups(Emission);
            return Emission.getAllocatedAddress();
          });
        } else {
          IsRegistered = PrivateScope.addPrivate(OrigVD, [&]() -> Address {
            // Emit private VarDecl with copy init.
            // Remap temp VDInit variable to the address of the original
            // variable
            // (for proper handling of captured global variables).
            setAddrOfLocalVar(VDInit, OriginalAddr);
            EmitDecl(*VD);
            LocalDeclMap.erase(VDInit);
            return GetAddrOfLocalVar(VD);
          });
        }
        assert(IsRegistered &&
               "firstprivate var already registered as private");
        // Silence the warning about unused variable.
        (void)IsRegistered;
      }
      ++IRef;
      ++InitsRef;
    }
  }
  return FirstprivateIsLastprivate && !EmittedAsFirstprivate.empty();
}

void CodeGenFunction::EmitOMPPrivateClause(
    const OMPExecutableDirective &D,
    CodeGenFunction::OMPPrivateScope &PrivateScope) {
  if (!HaveInsertPoint())
    return;
  llvm::DenseSet<const VarDecl *> EmittedAsPrivate;
  for (const auto *C : D.getClausesOfKind<OMPPrivateClause>()) {
    auto IRef = C->varlist_begin();
    for (auto IInit : C->private_copies()) {
      auto *OrigVD = cast<VarDecl>(cast<DeclRefExpr>(*IRef)->getDecl());
      if (EmittedAsPrivate.insert(OrigVD->getCanonicalDecl()).second) {
        auto VD = cast<VarDecl>(cast<DeclRefExpr>(IInit)->getDecl());
        bool IsRegistered =
            PrivateScope.addPrivate(OrigVD, [&]() -> Address {
              // Emit private VarDecl with copy init.
              EmitDecl(*VD);
              return GetAddrOfLocalVar(VD);
            });
        assert(IsRegistered && "private var already registered as private");
        // Silence the warning about unused variable.
        (void)IsRegistered;
      }
      ++IRef;
    }
  }
}

bool CodeGenFunction::EmitOMPCopyinClause(const OMPExecutableDirective &D) {
  if (!HaveInsertPoint())
    return false;
  // threadprivate_var1 = master_threadprivate_var1;
  // operator=(threadprivate_var2, master_threadprivate_var2);
  // ...
  // __kmpc_barrier(&loc, global_tid);
  llvm::DenseSet<const VarDecl *> CopiedVars;
  llvm::BasicBlock *CopyBegin = nullptr, *CopyEnd = nullptr;
  for (const auto *C : D.getClausesOfKind<OMPCopyinClause>()) {
    auto IRef = C->varlist_begin();
    auto ISrcRef = C->source_exprs().begin();
    auto IDestRef = C->destination_exprs().begin();
    for (auto *AssignOp : C->assignment_ops()) {
      auto *VD = cast<VarDecl>(cast<DeclRefExpr>(*IRef)->getDecl());
      QualType Type = VD->getType();
      if (CopiedVars.insert(VD->getCanonicalDecl()).second) {
        // Get the address of the master variable. If we are emitting code with
        // TLS support, the address is passed from the master as field in the
        // captured declaration.
        Address MasterAddr = Address::invalid();
        if (getLangOpts().OpenMPUseTLS &&
            getContext().getTargetInfo().isTLSSupported()) {
          assert(CapturedStmtInfo->lookup(VD) &&
                 "Copyin threadprivates should have been captured!");
          DeclRefExpr DRE(const_cast<VarDecl *>(VD), true, (*IRef)->getType(),
                          VK_LValue, (*IRef)->getExprLoc());
          MasterAddr = EmitLValue(&DRE).getAddress();
          LocalDeclMap.erase(VD);
        } else {
          MasterAddr =
            Address(VD->isStaticLocal() ? CGM.getStaticLocalDeclAddress(VD)
                                        : CGM.GetAddrOfGlobal(VD),
                    getContext().getDeclAlign(VD));
        }
        // Get the address of the threadprivate variable.
        Address PrivateAddr = EmitLValue(*IRef).getAddress();
        if (CopiedVars.size() == 1) {
          // At first check if current thread is a master thread. If it is, no
          // need to copy data.
          CopyBegin = createBasicBlock("copyin.not.master");
          CopyEnd = createBasicBlock("copyin.not.master.end");
          Builder.CreateCondBr(
              Builder.CreateICmpNE(
                  Builder.CreatePtrToInt(MasterAddr.getPointer(), CGM.IntPtrTy),
                  Builder.CreatePtrToInt(PrivateAddr.getPointer(), CGM.IntPtrTy)),
              CopyBegin, CopyEnd);
          EmitBlock(CopyBegin);
        }
        auto *SrcVD = cast<VarDecl>(cast<DeclRefExpr>(*ISrcRef)->getDecl());
        auto *DestVD = cast<VarDecl>(cast<DeclRefExpr>(*IDestRef)->getDecl());
        EmitOMPCopy(Type, PrivateAddr, MasterAddr, DestVD, SrcVD, AssignOp);
      }
      ++IRef;
      ++ISrcRef;
      ++IDestRef;
    }
  }
  if (CopyEnd) {
    // Exit out of copying procedure for non-master thread.
    EmitBlock(CopyEnd, /*IsFinished=*/true);
    return true;
  }
  return false;
}

bool CodeGenFunction::EmitOMPLastprivateClauseInit(
    const OMPExecutableDirective &D, OMPPrivateScope &PrivateScope) {
  if (!HaveInsertPoint())
    return false;
  bool HasAtLeastOneLastprivate = false;
  llvm::DenseSet<const VarDecl *> SIMDLCVs;
  if (isOpenMPSimdDirective(D.getDirectiveKind())) {
    auto *LoopDirective = cast<OMPLoopDirective>(&D);
    for (auto *C : LoopDirective->counters()) {
      SIMDLCVs.insert(
          cast<VarDecl>(cast<DeclRefExpr>(C)->getDecl())->getCanonicalDecl());
    }
  }
  llvm::DenseSet<const VarDecl *> AlreadyEmittedVars;
  for (const auto *C : D.getClausesOfKind<OMPLastprivateClause>()) {
    HasAtLeastOneLastprivate = true;
    if (isOpenMPTaskLoopDirective(D.getDirectiveKind()))
      break;
    auto IRef = C->varlist_begin();
    auto IDestRef = C->destination_exprs().begin();
    for (auto *IInit : C->private_copies()) {
      // Keep the address of the original variable for future update at the end
      // of the loop.
      auto *OrigVD = cast<VarDecl>(cast<DeclRefExpr>(*IRef)->getDecl());
      // Taskloops do not require additional initialization, it is done in
      // runtime support library.
      if (AlreadyEmittedVars.insert(OrigVD->getCanonicalDecl()).second) {
        auto *DestVD = cast<VarDecl>(cast<DeclRefExpr>(*IDestRef)->getDecl());
        PrivateScope.addPrivate(DestVD, [this, OrigVD, IRef]() -> Address {
          DeclRefExpr DRE(
              const_cast<VarDecl *>(OrigVD),
              /*RefersToEnclosingVariableOrCapture=*/CapturedStmtInfo->lookup(
                  OrigVD) != nullptr,
              (*IRef)->getType(), VK_LValue, (*IRef)->getExprLoc());
          return EmitLValue(&DRE).getAddress();
        });
        // Check if the variable is also a firstprivate: in this case IInit is
        // not generated. Initialization of this variable will happen in codegen
        // for 'firstprivate' clause.
        if (IInit && !SIMDLCVs.count(OrigVD->getCanonicalDecl())) {
          auto *VD = cast<VarDecl>(cast<DeclRefExpr>(IInit)->getDecl());
          bool IsRegistered = PrivateScope.addPrivate(OrigVD, [&]() -> Address {
            // Emit private VarDecl with copy init.
            EmitDecl(*VD);
            return GetAddrOfLocalVar(VD);
          });
          assert(IsRegistered &&
                 "lastprivate var already registered as private");
          (void)IsRegistered;
        }
      }
      ++IRef;
      ++IDestRef;
    }
  }
  return HasAtLeastOneLastprivate;
}

void CodeGenFunction::EmitOMPLastprivateClauseFinal(
    const OMPExecutableDirective &D, bool NoFinals,
    llvm::Value *IsLastIterCond) {
  if (!HaveInsertPoint())
    return;
  // Emit following code:
  // if (<IsLastIterCond>) {
  //   orig_var1 = private_orig_var1;
  //   ...
  //   orig_varn = private_orig_varn;
  // }
  llvm::BasicBlock *ThenBB = nullptr;
  llvm::BasicBlock *DoneBB = nullptr;
  if (IsLastIterCond) {
    ThenBB = createBasicBlock(".omp.lastprivate.then");
    DoneBB = createBasicBlock(".omp.lastprivate.done");
    Builder.CreateCondBr(IsLastIterCond, ThenBB, DoneBB);
    EmitBlock(ThenBB);
  }
  llvm::DenseSet<const VarDecl *> AlreadyEmittedVars;
  llvm::DenseMap<const VarDecl *, const Expr *> LoopCountersAndUpdates;
  if (auto *LoopDirective = dyn_cast<OMPLoopDirective>(&D)) {
    auto IC = LoopDirective->counters().begin();
    for (auto F : LoopDirective->finals()) {
      auto *D =
          cast<VarDecl>(cast<DeclRefExpr>(*IC)->getDecl())->getCanonicalDecl();
      if (NoFinals)
        AlreadyEmittedVars.insert(D);
      else
        LoopCountersAndUpdates[D] = F;
      ++IC;
    }
  }
  for (const auto *C : D.getClausesOfKind<OMPLastprivateClause>()) {
    auto IRef = C->varlist_begin();
    auto ISrcRef = C->source_exprs().begin();
    auto IDestRef = C->destination_exprs().begin();
    for (auto *AssignOp : C->assignment_ops()) {
      auto *PrivateVD = cast<VarDecl>(cast<DeclRefExpr>(*IRef)->getDecl());
      QualType Type = PrivateVD->getType();
      auto *CanonicalVD = PrivateVD->getCanonicalDecl();
      if (AlreadyEmittedVars.insert(CanonicalVD).second) {
        // If lastprivate variable is a loop control variable for loop-based
        // directive, update its value before copyin back to original
        // variable.
        if (auto *FinalExpr = LoopCountersAndUpdates.lookup(CanonicalVD))
          EmitIgnoredExpr(FinalExpr);
        auto *SrcVD = cast<VarDecl>(cast<DeclRefExpr>(*ISrcRef)->getDecl());
        auto *DestVD = cast<VarDecl>(cast<DeclRefExpr>(*IDestRef)->getDecl());
        // Get the address of the original variable.
        Address OriginalAddr = GetAddrOfLocalVar(DestVD);
        // Get the address of the private variable.
        Address PrivateAddr = GetAddrOfLocalVar(PrivateVD);
        if (auto RefTy = PrivateVD->getType()->getAs<ReferenceType>())
          PrivateAddr =
              Address(Builder.CreateLoad(PrivateAddr),
                      getNaturalTypeAlignment(RefTy->getPointeeType()));
        EmitOMPCopy(Type, OriginalAddr, PrivateAddr, DestVD, SrcVD, AssignOp);
      }
      ++IRef;
      ++ISrcRef;
      ++IDestRef;
    }
    if (auto *PostUpdate = C->getPostUpdateExpr())
      EmitIgnoredExpr(PostUpdate);
  }
  if (IsLastIterCond)
    EmitBlock(DoneBB, /*IsFinished=*/true);
}

static Address castToBase(CodeGenFunction &CGF, QualType BaseTy, QualType ElTy,
                          LValue BaseLV, llvm::Value *Addr) {
  Address Tmp = Address::invalid();
  Address TopTmp = Address::invalid();
  Address MostTopTmp = Address::invalid();
  BaseTy = BaseTy.getNonReferenceType();
  while ((BaseTy->isPointerType() || BaseTy->isReferenceType()) &&
         !CGF.getContext().hasSameType(BaseTy, ElTy)) {
    Tmp = CGF.CreateMemTemp(BaseTy);
    if (TopTmp.isValid())
      CGF.Builder.CreateStore(Tmp.getPointer(), TopTmp);
    else
      MostTopTmp = Tmp;
    TopTmp = Tmp;
    BaseTy = BaseTy->getPointeeType();
  }
  llvm::Type *Ty = BaseLV.getPointer()->getType();
  if (Tmp.isValid())
    Ty = Tmp.getElementType();
  Addr = CGF.Builder.CreatePointerBitCastOrAddrSpaceCast(Addr, Ty);
  if (Tmp.isValid()) {
    CGF.Builder.CreateStore(Addr, Tmp);
    return MostTopTmp;
  }
  return Address(Addr, BaseLV.getAlignment());
}

static LValue loadToBegin(CodeGenFunction &CGF, QualType BaseTy, QualType ElTy,
                          LValue BaseLV) {
  BaseTy = BaseTy.getNonReferenceType();
  while ((BaseTy->isPointerType() || BaseTy->isReferenceType()) &&
         !CGF.getContext().hasSameType(BaseTy, ElTy)) {
    if (auto *PtrTy = BaseTy->getAs<PointerType>())
      BaseLV = CGF.EmitLoadOfPointerLValue(BaseLV.getAddress(), PtrTy);
    else {
      BaseLV = CGF.EmitLoadOfReferenceLValue(BaseLV.getAddress(),
                                             BaseTy->castAs<ReferenceType>());
    }
    BaseTy = BaseTy->getPointeeType();
  }
  return CGF.MakeAddrLValue(
      Address(
          CGF.Builder.CreatePointerBitCastOrAddrSpaceCast(
              BaseLV.getPointer(), CGF.ConvertTypeForMem(ElTy)->getPointerTo()),
          BaseLV.getAlignment()),
      BaseLV.getType(), BaseLV.getAlignmentSource());
}

void CodeGenFunction::EmitOMPReductionClauseInit(
    const OMPExecutableDirective &D,
    CodeGenFunction::OMPPrivateScope &PrivateScope) {
  if (!HaveInsertPoint())
    return;
  for (const auto *C : D.getClausesOfKind<OMPReductionClause>()) {
    auto ILHS = C->lhs_exprs().begin();
    auto IRHS = C->rhs_exprs().begin();
    auto IPriv = C->privates().begin();
    auto IRed = C->reduction_ops().begin();
    for (auto IRef : C->varlists()) {
      auto *LHSVD = cast<VarDecl>(cast<DeclRefExpr>(*ILHS)->getDecl());
      auto *RHSVD = cast<VarDecl>(cast<DeclRefExpr>(*IRHS)->getDecl());
      auto *PrivateVD = cast<VarDecl>(cast<DeclRefExpr>(*IPriv)->getDecl());
      auto *DRD = getReductionInit(*IRed);
      if (auto *OASE = dyn_cast<OMPArraySectionExpr>(IRef)) {
        auto *Base = OASE->getBase()->IgnoreParenImpCasts();
        while (auto *TempOASE = dyn_cast<OMPArraySectionExpr>(Base))
          Base = TempOASE->getBase()->IgnoreParenImpCasts();
        while (auto *TempASE = dyn_cast<ArraySubscriptExpr>(Base))
          Base = TempASE->getBase()->IgnoreParenImpCasts();
        auto *DE = cast<DeclRefExpr>(Base);
        auto *OrigVD = cast<VarDecl>(DE->getDecl());
        auto OASELValueLB = EmitOMPArraySectionExpr(OASE);
        auto OASELValueUB =
            EmitOMPArraySectionExpr(OASE, /*IsLowerBound=*/false);
        auto OriginalBaseLValue = EmitLValue(DE);
        LValue BaseLValue =
            loadToBegin(*this, OrigVD->getType(), OASELValueLB.getType(),
                        OriginalBaseLValue);
        // Store the address of the original variable associated with the LHS
        // implicit variable.
        PrivateScope.addPrivate(LHSVD, [this, OASELValueLB]() -> Address {
          return OASELValueLB.getAddress();
        });
        // Emit reduction copy.
        bool IsRegistered = PrivateScope.addPrivate(
            OrigVD, [this, OrigVD, PrivateVD, BaseLValue, OASELValueLB,
                     OASELValueUB, OriginalBaseLValue, DRD, IRed]() -> Address {
              // Emit VarDecl with copy init for arrays.
              // Get the address of the original variable captured in current
              // captured region.
              auto *Size = Builder.CreatePtrDiff(OASELValueUB.getPointer(),
                                                 OASELValueLB.getPointer());
              Size = Builder.CreateNUWAdd(
                  Size, llvm::ConstantInt::get(Size->getType(), /*V=*/1));
              CodeGenFunction::OpaqueValueMapping OpaqueMap(
                  *this, cast<OpaqueValueExpr>(
                             getContext()
                                 .getAsVariableArrayType(PrivateVD->getType())
                                 ->getSizeExpr()),
                  RValue::get(Size));
              EmitVariablyModifiedType(PrivateVD->getType());
              auto Emission = EmitAutoVarAlloca(*PrivateVD);
              auto Addr = Emission.getAllocatedAddress();
              auto *Init = PrivateVD->getInit();
              EmitOMPAggregateInit(*this, Addr, PrivateVD->getType(),
                                   DRD ? *IRed : Init,
                                   OASELValueLB.getAddress());
              EmitAutoVarCleanups(Emission);
              // Emit private VarDecl with reduction init.
              auto *Offset = Builder.CreatePtrDiff(BaseLValue.getPointer(),
                                                   OASELValueLB.getPointer());
              auto *Ptr = Builder.CreateGEP(Addr.getPointer(), Offset);
              return castToBase(*this, OrigVD->getType(),
                                OASELValueLB.getType(), OriginalBaseLValue,
                                Ptr);
            });
        assert(IsRegistered && "private var already registered as private");
        // Silence the warning about unused variable.
        (void)IsRegistered;
        PrivateScope.addPrivate(RHSVD, [this, PrivateVD]() -> Address {
          return GetAddrOfLocalVar(PrivateVD);
        });
      } else if (auto *ASE = dyn_cast<ArraySubscriptExpr>(IRef)) {
        auto *Base = ASE->getBase()->IgnoreParenImpCasts();
        while (auto *TempASE = dyn_cast<ArraySubscriptExpr>(Base))
          Base = TempASE->getBase()->IgnoreParenImpCasts();
        auto *DE = cast<DeclRefExpr>(Base);
        auto *OrigVD = cast<VarDecl>(DE->getDecl());
        auto ASELValue = EmitLValue(ASE);
        auto OriginalBaseLValue = EmitLValue(DE);
        LValue BaseLValue = loadToBegin(
            *this, OrigVD->getType(), ASELValue.getType(), OriginalBaseLValue);
        // Store the address of the original variable associated with the LHS
        // implicit variable.
        PrivateScope.addPrivate(LHSVD, [this, ASELValue]() -> Address {
          return ASELValue.getAddress();
        });
        // Emit reduction copy.
        bool IsRegistered = PrivateScope.addPrivate(
            OrigVD, [this, OrigVD, PrivateVD, BaseLValue, ASELValue,
                     OriginalBaseLValue, DRD, IRed]() -> Address {
              // Emit private VarDecl with reduction init.
              AutoVarEmission Emission = EmitAutoVarAlloca(*PrivateVD);
              auto Addr = Emission.getAllocatedAddress();
              if (DRD && (DRD->getInitializer() || !PrivateVD->hasInit())) {
                emitInitWithReductionInitializer(*this, DRD, *IRed, Addr,
                                                 ASELValue.getAddress(),
                                                 ASELValue.getType());
              } else
                EmitAutoVarInit(Emission);
              EmitAutoVarCleanups(Emission);
              auto *Offset = Builder.CreatePtrDiff(BaseLValue.getPointer(),
                                                   ASELValue.getPointer());
              auto *Ptr = Builder.CreateGEP(Addr.getPointer(), Offset);
              return castToBase(*this, OrigVD->getType(), ASELValue.getType(),
                                OriginalBaseLValue, Ptr);
            });
        assert(IsRegistered && "private var already registered as private");
        // Silence the warning about unused variable.
        (void)IsRegistered;
        PrivateScope.addPrivate(RHSVD, [this, PrivateVD, RHSVD]() -> Address {
          return Builder.CreateElementBitCast(
              GetAddrOfLocalVar(PrivateVD), ConvertTypeForMem(RHSVD->getType()),
              "rhs.begin");
        });
      } else {
        auto *OrigVD = cast<VarDecl>(cast<DeclRefExpr>(IRef)->getDecl());
        QualType Type = PrivateVD->getType();
        if (getContext().getAsArrayType(Type)) {
          // Store the address of the original variable associated with the LHS
          // implicit variable.
          DeclRefExpr DRE(const_cast<VarDecl *>(OrigVD),
                          CapturedStmtInfo->lookup(OrigVD) != nullptr,
                          IRef->getType(), VK_LValue, IRef->getExprLoc());
          Address OriginalAddr = EmitLValue(&DRE).getAddress();
          PrivateScope.addPrivate(LHSVD, [this, &OriginalAddr,
                                          LHSVD]() -> Address {
            OriginalAddr = Builder.CreateElementBitCast(
                OriginalAddr, ConvertTypeForMem(LHSVD->getType()), "lhs.begin");
            return OriginalAddr;
          });
          bool IsRegistered = PrivateScope.addPrivate(OrigVD, [&]() -> Address {
            if (Type->isVariablyModifiedType()) {
              CodeGenFunction::OpaqueValueMapping OpaqueMap(
                  *this, cast<OpaqueValueExpr>(
                             getContext()
                                 .getAsVariableArrayType(PrivateVD->getType())
                                 ->getSizeExpr()),
                  RValue::get(
                      getTypeSize(OrigVD->getType().getNonReferenceType())));
              EmitVariablyModifiedType(Type);
            }
            auto Emission = EmitAutoVarAlloca(*PrivateVD);
            auto Addr = Emission.getAllocatedAddress();
            auto *Init = PrivateVD->getInit();
            EmitOMPAggregateInit(*this, Addr, PrivateVD->getType(),
                                 DRD ? *IRed : Init, OriginalAddr);
            EmitAutoVarCleanups(Emission);
            return Emission.getAllocatedAddress();
          });
          assert(IsRegistered && "private var already registered as private");
          // Silence the warning about unused variable.
          (void)IsRegistered;
          PrivateScope.addPrivate(RHSVD, [this, PrivateVD, RHSVD]() -> Address {
            return Builder.CreateElementBitCast(
                GetAddrOfLocalVar(PrivateVD),
                ConvertTypeForMem(RHSVD->getType()), "rhs.begin");
          });
        } else {
          // Store the address of the original variable associated with the LHS
          // implicit variable.
          Address OriginalAddr = Address::invalid();
          PrivateScope.addPrivate(LHSVD, [this, OrigVD, IRef,
                                          &OriginalAddr]() -> Address {
            DeclRefExpr DRE(const_cast<VarDecl *>(OrigVD),
                            CapturedStmtInfo->lookup(OrigVD) != nullptr,
                            IRef->getType(), VK_LValue, IRef->getExprLoc());
            OriginalAddr = EmitLValue(&DRE).getAddress();
            return OriginalAddr;
          });
          // Emit reduction copy.
          bool IsRegistered = PrivateScope.addPrivate(
              OrigVD, [this, PrivateVD, OriginalAddr, DRD, IRed]() -> Address {
                // Emit private VarDecl with reduction init.
                AutoVarEmission Emission = EmitAutoVarAlloca(*PrivateVD);
                auto Addr = Emission.getAllocatedAddress();
                if (DRD && (DRD->getInitializer() || !PrivateVD->hasInit())) {
                  emitInitWithReductionInitializer(*this, DRD, *IRed, Addr,
                                                   OriginalAddr,
                                                   PrivateVD->getType());
                } else
                  EmitAutoVarInit(Emission);
                EmitAutoVarCleanups(Emission);
                return Addr;
              });
          assert(IsRegistered && "private var already registered as private");
          // Silence the warning about unused variable.
          (void)IsRegistered;
          PrivateScope.addPrivate(RHSVD, [this, PrivateVD]() -> Address {
            return GetAddrOfLocalVar(PrivateVD);
          });
        }
      }
      ++ILHS;
      ++IRHS;
      ++IPriv;
      ++IRed;
    }
  }
}

void CodeGenFunction::EmitOMPReductionClauseFinal(
    const OMPExecutableDirective &D) {
  if (!HaveInsertPoint())
    return;
  llvm::SmallVector<const Expr *, 8> Privates;
  llvm::SmallVector<const Expr *, 8> LHSExprs;
  llvm::SmallVector<const Expr *, 8> RHSExprs;
  llvm::SmallVector<const Expr *, 8> ReductionOps;
  bool HasAtLeastOneReduction = false;
  for (const auto *C : D.getClausesOfKind<OMPReductionClause>()) {
    HasAtLeastOneReduction = true;
    Privates.append(C->privates().begin(), C->privates().end());
    LHSExprs.append(C->lhs_exprs().begin(), C->lhs_exprs().end());
    RHSExprs.append(C->rhs_exprs().begin(), C->rhs_exprs().end());
    ReductionOps.append(C->reduction_ops().begin(), C->reduction_ops().end());
  }
  if (HasAtLeastOneReduction) {
    // Emit nowait reduction if nowait clause is present or directive is a
    // parallel directive (it always has implicit barrier).
    CGM.getOpenMPRuntime().emitReduction(
        *this, D.getLocEnd(), Privates, LHSExprs, RHSExprs, ReductionOps,
        D.getSingleClause<OMPNowaitClause>() ||
            isOpenMPParallelDirective(D.getDirectiveKind()) ||
            D.getDirectiveKind() == OMPD_simd,
        D.getDirectiveKind() == OMPD_simd);
  }
}

static void emitPostUpdateForReductionClause(
    CodeGenFunction &CGF, const OMPExecutableDirective &D,
    const llvm::function_ref<llvm::Value *(CodeGenFunction &)> &CondGen) {
  if (!CGF.HaveInsertPoint())
    return;
  llvm::BasicBlock *DoneBB = nullptr;
  for (const auto *C : D.getClausesOfKind<OMPReductionClause>()) {
    if (auto *PostUpdate = C->getPostUpdateExpr()) {
      if (!DoneBB) {
        if (auto *Cond = CondGen(CGF)) {
          // If the first post-update expression is found, emit conditional
          // block if it was requested.
          auto *ThenBB = CGF.createBasicBlock(".omp.reduction.pu");
          DoneBB = CGF.createBasicBlock(".omp.reduction.pu.done");
          CGF.Builder.CreateCondBr(Cond, ThenBB, DoneBB);
          CGF.EmitBlock(ThenBB);
        }
      }
      CGF.EmitIgnoredExpr(PostUpdate);
    }
  }
  if (DoneBB)
    CGF.EmitBlock(DoneBB, /*IsFinished=*/true);
}

static void emitCommonOMPParallelDirective(CodeGenFunction &CGF,
                                           const OMPExecutableDirective &S,
                                           OpenMPDirectiveKind InnermostKind,
                                           const RegionCodeGenTy &CodeGen) {
  auto CS = cast<CapturedStmt>(S.getAssociatedStmt());
  auto OutlinedFn = CGF.CGM.getOpenMPRuntime().
      emitParallelOrTeamsOutlinedFunction(S,
          *CS->getCapturedDecl()->param_begin(), InnermostKind, CodeGen);
  if (const auto *NumThreadsClause = S.getSingleClause<OMPNumThreadsClause>()) {
    CodeGenFunction::RunCleanupsScope NumThreadsScope(CGF);
    auto NumThreads = CGF.EmitScalarExpr(NumThreadsClause->getNumThreads(),
                                         /*IgnoreResultAssign*/ true);
    CGF.CGM.getOpenMPRuntime().emitNumThreadsClause(
        CGF, NumThreads, NumThreadsClause->getLocStart());
  }
  if (const auto *ProcBindClause = S.getSingleClause<OMPProcBindClause>()) {
    CodeGenFunction::RunCleanupsScope ProcBindScope(CGF);
    CGF.CGM.getOpenMPRuntime().emitProcBindClause(
        CGF, ProcBindClause->getProcBindKind(), ProcBindClause->getLocStart());
  }
  const Expr *IfCond = nullptr;
  for (const auto *C : S.getClausesOfKind<OMPIfClause>()) {
    if (C->getNameModifier() == OMPD_unknown ||
        C->getNameModifier() == OMPD_parallel) {
      IfCond = C->getCondition();
      break;
    }
  }

  OMPLexicalScope Scope(CGF, S);
  llvm::SmallVector<llvm::Value *, 16> CapturedVars;
  CGF.GenerateOpenMPCapturedVars(*CS, CapturedVars);
  CGF.CGM.getOpenMPRuntime().emitParallelCall(CGF, S.getLocStart(), OutlinedFn,
                                              CapturedVars, IfCond);
}

void CodeGenFunction::EmitOMPParallelDirective(const OMPParallelDirective &S) {
  // Emit parallel region as a standalone region.
  auto &&CodeGen = [&S](CodeGenFunction &CGF, PrePostActionTy &) {
    OMPPrivateScope PrivateScope(CGF);
    bool Copyins = CGF.EmitOMPCopyinClause(S);
    (void)CGF.EmitOMPFirstprivateClause(S, PrivateScope);
    if (Copyins) {
      // Emit implicit barrier to synchronize threads and avoid data races on
      // propagation master's thread values of threadprivate variables to local
      // instances of that variables of all other implicit threads.
      CGF.CGM.getOpenMPRuntime().emitBarrierCall(
          CGF, S.getLocStart(), OMPD_unknown, /*EmitChecks=*/false,
          /*ForceSimpleCall=*/true);
    }
    CGF.EmitOMPPrivateClause(S, PrivateScope);
    CGF.EmitOMPReductionClauseInit(S, PrivateScope);
    (void)PrivateScope.Privatize();
    CGF.EmitStmt(cast<CapturedStmt>(S.getAssociatedStmt())->getCapturedStmt());
    CGF.EmitOMPReductionClauseFinal(S);
  };
  emitCommonOMPParallelDirective(*this, S, OMPD_parallel, CodeGen);
  emitPostUpdateForReductionClause(
      *this, S, [](CodeGenFunction &) -> llvm::Value * { return nullptr; });
}

void CodeGenFunction::EmitOMPLoopBody(const OMPLoopDirective &D,
                                      JumpDest LoopExit) {
  RunCleanupsScope BodyScope(*this);
  // Update counters values on current iteration.
  for (auto I : D.updates()) {
    EmitIgnoredExpr(I);
  }
  // Update the linear variables.
  for (const auto *C : D.getClausesOfKind<OMPLinearClause>()) {
    for (auto *U : C->updates())
      EmitIgnoredExpr(U);
  }

  // On a continue in the body, jump to the end.
  auto Continue = getJumpDestInCurrentScope("omp.body.continue");
  BreakContinueStack.push_back(BreakContinue(LoopExit, Continue));
  // Emit loop body.
  EmitStmt(D.getBody());
  // The end (updates/cleanups).
  EmitBlock(Continue.getBlock());
  BreakContinueStack.pop_back();
}

void CodeGenFunction::EmitOMPInnerLoop(
    const Stmt &S, bool RequiresCleanup, const Expr *LoopCond,
    const Expr *IncExpr,
    const llvm::function_ref<void(CodeGenFunction &)> &BodyGen,
    const llvm::function_ref<void(CodeGenFunction &)> &PostIncGen) {
  auto LoopExit = getJumpDestInCurrentScope("omp.inner.for.end");

  // Start the loop with a block that tests the condition.
  auto CondBlock = createBasicBlock("omp.inner.for.cond");
  EmitBlock(CondBlock);
  LoopStack.push(CondBlock, Builder.getCurrentDebugLocation());

  // If there are any cleanups between here and the loop-exit scope,
  // create a block to stage a loop exit along.
  auto ExitBlock = LoopExit.getBlock();
  if (RequiresCleanup)
    ExitBlock = createBasicBlock("omp.inner.for.cond.cleanup");

  auto LoopBody = createBasicBlock("omp.inner.for.body");

  // Emit condition.
  EmitBranchOnBoolExpr(LoopCond, LoopBody, ExitBlock, getProfileCount(&S));
  if (ExitBlock != LoopExit.getBlock()) {
    EmitBlock(ExitBlock);
    EmitBranchThroughCleanup(LoopExit);
  }

  EmitBlock(LoopBody);
  incrementProfileCounter(&S);

  // Create a block for the increment.
  auto Continue = getJumpDestInCurrentScope("omp.inner.for.inc");
  BreakContinueStack.push_back(BreakContinue(LoopExit, Continue));

  BodyGen(*this);

  // Emit "IV = IV + 1" and a back-edge to the condition block.
  EmitBlock(Continue.getBlock());
  EmitIgnoredExpr(IncExpr);
  PostIncGen(*this);
  BreakContinueStack.pop_back();
  EmitBranch(CondBlock);
  LoopStack.pop();
  // Emit the fall-through block.
  EmitBlock(LoopExit.getBlock());
}

void CodeGenFunction::EmitOMPLinearClauseInit(const OMPLoopDirective &D) {
  if (!HaveInsertPoint())
    return;
  // Emit inits for the linear variables.
  for (const auto *C : D.getClausesOfKind<OMPLinearClause>()) {
    for (auto *Init : C->inits()) {
      auto *VD = cast<VarDecl>(cast<DeclRefExpr>(Init)->getDecl());
      if (auto *Ref = dyn_cast<DeclRefExpr>(VD->getInit()->IgnoreImpCasts())) {
        AutoVarEmission Emission = EmitAutoVarAlloca(*VD);
        auto *OrigVD = cast<VarDecl>(Ref->getDecl());
        DeclRefExpr DRE(const_cast<VarDecl *>(OrigVD),
                        CapturedStmtInfo->lookup(OrigVD) != nullptr,
                        VD->getInit()->getType(), VK_LValue,
                        VD->getInit()->getExprLoc());
        EmitExprAsInit(&DRE, VD, MakeAddrLValue(Emission.getAllocatedAddress(),
                                                VD->getType()),
                       /*capturedByInit=*/false);
        EmitAutoVarCleanups(Emission);
      } else
        EmitVarDecl(*VD);
    }
    // Emit the linear steps for the linear clauses.
    // If a step is not constant, it is pre-calculated before the loop.
    if (auto CS = cast_or_null<BinaryOperator>(C->getCalcStep()))
      if (auto SaveRef = cast<DeclRefExpr>(CS->getLHS())) {
        EmitVarDecl(*cast<VarDecl>(SaveRef->getDecl()));
        // Emit calculation of the linear step.
        EmitIgnoredExpr(CS);
      }
  }
}

void CodeGenFunction::EmitOMPLinearClauseFinal(
    const OMPLoopDirective &D,
    const llvm::function_ref<llvm::Value *(CodeGenFunction &)> &CondGen) {
  if (!HaveInsertPoint())
    return;
  llvm::BasicBlock *DoneBB = nullptr;
  // Emit the final values of the linear variables.
  for (const auto *C : D.getClausesOfKind<OMPLinearClause>()) {
    auto IC = C->varlist_begin();
    for (auto *F : C->finals()) {
      if (!DoneBB) {
        if (auto *Cond = CondGen(*this)) {
          // If the first post-update expression is found, emit conditional
          // block if it was requested.
          auto *ThenBB = createBasicBlock(".omp.linear.pu");
          DoneBB = createBasicBlock(".omp.linear.pu.done");
          Builder.CreateCondBr(Cond, ThenBB, DoneBB);
          EmitBlock(ThenBB);
        }
      }
      auto *OrigVD = cast<VarDecl>(cast<DeclRefExpr>(*IC)->getDecl());
      DeclRefExpr DRE(const_cast<VarDecl *>(OrigVD),
                      CapturedStmtInfo->lookup(OrigVD) != nullptr,
                      (*IC)->getType(), VK_LValue, (*IC)->getExprLoc());
      Address OrigAddr = EmitLValue(&DRE).getAddress();
      CodeGenFunction::OMPPrivateScope VarScope(*this);
      VarScope.addPrivate(OrigVD, [OrigAddr]() -> Address { return OrigAddr; });
      (void)VarScope.Privatize();
      EmitIgnoredExpr(F);
      ++IC;
    }
    if (auto *PostUpdate = C->getPostUpdateExpr())
      EmitIgnoredExpr(PostUpdate);
  }
  if (DoneBB)
    EmitBlock(DoneBB, /*IsFinished=*/true);
}

static void emitAlignedClause(CodeGenFunction &CGF,
                              const OMPExecutableDirective &D) {
  if (!CGF.HaveInsertPoint())
    return;
  for (const auto *Clause : D.getClausesOfKind<OMPAlignedClause>()) {
    unsigned ClauseAlignment = 0;
    if (auto AlignmentExpr = Clause->getAlignment()) {
      auto AlignmentCI =
          cast<llvm::ConstantInt>(CGF.EmitScalarExpr(AlignmentExpr));
      ClauseAlignment = static_cast<unsigned>(AlignmentCI->getZExtValue());
    }
    for (auto E : Clause->varlists()) {
      unsigned Alignment = ClauseAlignment;
      if (Alignment == 0) {
        // OpenMP [2.8.1, Description]
        // If no optional parameter is specified, implementation-defined default
        // alignments for SIMD instructions on the target platforms are assumed.
        Alignment =
            CGF.getContext()
                .toCharUnitsFromBits(CGF.getContext().getOpenMPDefaultSimdAlign(
                    E->getType()->getPointeeType()))
                .getQuantity();
      }
      assert((Alignment == 0 || llvm::isPowerOf2_32(Alignment)) &&
             "alignment is not power of 2");
      if (Alignment != 0) {
        llvm::Value *PtrValue = CGF.EmitScalarExpr(E);
        CGF.EmitAlignmentAssumption(PtrValue, Alignment);
      }
    }
  }
}

void CodeGenFunction::EmitOMPPrivateLoopCounters(
    const OMPLoopDirective &S, CodeGenFunction::OMPPrivateScope &LoopScope) {
  if (!HaveInsertPoint())
    return;
  auto I = S.private_counters().begin();
  for (auto *E : S.counters()) {
    auto *VD = cast<VarDecl>(cast<DeclRefExpr>(E)->getDecl());
    auto *PrivateVD = cast<VarDecl>(cast<DeclRefExpr>(*I)->getDecl());
    (void)LoopScope.addPrivate(VD, [&]() -> Address {
      // Emit var without initialization.
      if (!LocalDeclMap.count(PrivateVD)) {
        auto VarEmission = EmitAutoVarAlloca(*PrivateVD);
        EmitAutoVarCleanups(VarEmission);
      }
      DeclRefExpr DRE(const_cast<VarDecl *>(PrivateVD),
                      /*RefersToEnclosingVariableOrCapture=*/false,
                      (*I)->getType(), VK_LValue, (*I)->getExprLoc());
      return EmitLValue(&DRE).getAddress();
    });
    if (LocalDeclMap.count(VD) || CapturedStmtInfo->lookup(VD) ||
        VD->hasGlobalStorage()) {
      (void)LoopScope.addPrivate(PrivateVD, [&]() -> Address {
        DeclRefExpr DRE(const_cast<VarDecl *>(VD),
                        LocalDeclMap.count(VD) || CapturedStmtInfo->lookup(VD),
                        E->getType(), VK_LValue, E->getExprLoc());
        return EmitLValue(&DRE).getAddress();
      });
    }
    ++I;
  }
}

static void emitPreCond(CodeGenFunction &CGF, const OMPLoopDirective &S,
                        const Expr *Cond, llvm::BasicBlock *TrueBlock,
                        llvm::BasicBlock *FalseBlock, uint64_t TrueCount) {
  if (!CGF.HaveInsertPoint())
    return;
  {
    CodeGenFunction::OMPPrivateScope PreCondScope(CGF);
    CGF.EmitOMPPrivateLoopCounters(S, PreCondScope);
    (void)PreCondScope.Privatize();
    // Get initial values of real counters.
    for (auto I : S.inits()) {
      CGF.EmitIgnoredExpr(I);
    }
  }
  // Check that loop is executed at least one time.
  CGF.EmitBranchOnBoolExpr(Cond, TrueBlock, FalseBlock, TrueCount);
}

void CodeGenFunction::EmitOMPLinearClause(
    const OMPLoopDirective &D, CodeGenFunction::OMPPrivateScope &PrivateScope) {
  if (!HaveInsertPoint())
    return;
  llvm::DenseSet<const VarDecl *> SIMDLCVs;
  if (isOpenMPSimdDirective(D.getDirectiveKind())) {
    auto *LoopDirective = cast<OMPLoopDirective>(&D);
    for (auto *C : LoopDirective->counters()) {
      SIMDLCVs.insert(
          cast<VarDecl>(cast<DeclRefExpr>(C)->getDecl())->getCanonicalDecl());
    }
  }
  for (const auto *C : D.getClausesOfKind<OMPLinearClause>()) {
    auto CurPrivate = C->privates().begin();
    for (auto *E : C->varlists()) {
      auto *VD = cast<VarDecl>(cast<DeclRefExpr>(E)->getDecl());
      auto *PrivateVD =
          cast<VarDecl>(cast<DeclRefExpr>(*CurPrivate)->getDecl());
      if (!SIMDLCVs.count(VD->getCanonicalDecl())) {
        bool IsRegistered = PrivateScope.addPrivate(VD, [&]() -> Address {
          // Emit private VarDecl with copy init.
          EmitVarDecl(*PrivateVD);
          return GetAddrOfLocalVar(PrivateVD);
        });
        assert(IsRegistered && "linear var already registered as private");
        // Silence the warning about unused variable.
        (void)IsRegistered;
      } else
        EmitVarDecl(*PrivateVD);
      ++CurPrivate;
    }
  }
}

static void emitSimdlenSafelenClause(CodeGenFunction &CGF,
                                     const OMPExecutableDirective &D,
                                     bool IsMonotonic) {
  if (!CGF.HaveInsertPoint())
    return;
  if (const auto *C = D.getSingleClause<OMPSimdlenClause>()) {
    RValue Len = CGF.EmitAnyExpr(C->getSimdlen(), AggValueSlot::ignored(),
                                 /*ignoreResult=*/true);
    llvm::ConstantInt *Val = cast<llvm::ConstantInt>(Len.getScalarVal());
    CGF.LoopStack.setVectorizeWidth(Val->getZExtValue());
    // In presence of finite 'safelen', it may be unsafe to mark all
    // the memory instructions parallel, because loop-carried
    // dependences of 'safelen' iterations are possible.
    if (!IsMonotonic)
      CGF.LoopStack.setParallel(!D.getSingleClause<OMPSafelenClause>());
  } else if (const auto *C = D.getSingleClause<OMPSafelenClause>()) {
    RValue Len = CGF.EmitAnyExpr(C->getSafelen(), AggValueSlot::ignored(),
                                 /*ignoreResult=*/true);
    llvm::ConstantInt *Val = cast<llvm::ConstantInt>(Len.getScalarVal());
    CGF.LoopStack.setVectorizeWidth(Val->getZExtValue());
    // In presence of finite 'safelen', it may be unsafe to mark all
    // the memory instructions parallel, because loop-carried
    // dependences of 'safelen' iterations are possible.
    CGF.LoopStack.setParallel(false);
  }
}

void CodeGenFunction::EmitOMPSimdInit(const OMPLoopDirective &D,
                                      bool IsMonotonic) {
  // Walk clauses and process safelen/lastprivate.
  LoopStack.setParallel(!IsMonotonic);
  LoopStack.setVectorizeEnable(true);
  emitSimdlenSafelenClause(*this, D, IsMonotonic);
}

void CodeGenFunction::EmitOMPSimdFinal(
    const OMPLoopDirective &D,
    const llvm::function_ref<llvm::Value *(CodeGenFunction &)> &CondGen) {
  if (!HaveInsertPoint())
    return;
  llvm::BasicBlock *DoneBB = nullptr;
  auto IC = D.counters().begin();
  auto IPC = D.private_counters().begin();
  for (auto F : D.finals()) {
    auto *OrigVD = cast<VarDecl>(cast<DeclRefExpr>((*IC))->getDecl());
    auto *PrivateVD = cast<VarDecl>(cast<DeclRefExpr>((*IPC))->getDecl());
    auto *CED = dyn_cast<OMPCapturedExprDecl>(OrigVD);
    if (LocalDeclMap.count(OrigVD) || CapturedStmtInfo->lookup(OrigVD) ||
        OrigVD->hasGlobalStorage() || CED) {
      if (!DoneBB) {
        if (auto *Cond = CondGen(*this)) {
          // If the first post-update expression is found, emit conditional
          // block if it was requested.
          auto *ThenBB = createBasicBlock(".omp.final.then");
          DoneBB = createBasicBlock(".omp.final.done");
          Builder.CreateCondBr(Cond, ThenBB, DoneBB);
          EmitBlock(ThenBB);
        }
      }
      Address OrigAddr = Address::invalid();
      if (CED)
        OrigAddr = EmitLValue(CED->getInit()->IgnoreImpCasts()).getAddress();
      else {
        DeclRefExpr DRE(const_cast<VarDecl *>(PrivateVD),
                        /*RefersToEnclosingVariableOrCapture=*/false,
                        (*IPC)->getType(), VK_LValue, (*IPC)->getExprLoc());
        OrigAddr = EmitLValue(&DRE).getAddress();
      }
      OMPPrivateScope VarScope(*this);
      VarScope.addPrivate(OrigVD,
                          [OrigAddr]() -> Address { return OrigAddr; });
      (void)VarScope.Privatize();
      EmitIgnoredExpr(F);
    }
    ++IC;
    ++IPC;
  }
  if (DoneBB)
    EmitBlock(DoneBB, /*IsFinished=*/true);
}

void CodeGenFunction::EmitOMPSimdDirective(const OMPSimdDirective &S) {
  auto &&CodeGen = [&S](CodeGenFunction &CGF, PrePostActionTy &) {
    OMPLoopScope PreInitScope(CGF, S);
    // if (PreCond) {
    //   for (IV in 0..LastIteration) BODY;
    //   <Final counter/linear vars updates>;
    // }
    //

    // Emit: if (PreCond) - begin.
    // If the condition constant folds and can be elided, avoid emitting the
    // whole loop.
    bool CondConstant;
    llvm::BasicBlock *ContBlock = nullptr;
    if (CGF.ConstantFoldsToSimpleInteger(S.getPreCond(), CondConstant)) {
      if (!CondConstant)
        return;
    } else {
      auto *ThenBlock = CGF.createBasicBlock("simd.if.then");
      ContBlock = CGF.createBasicBlock("simd.if.end");
      emitPreCond(CGF, S, S.getPreCond(), ThenBlock, ContBlock,
                  CGF.getProfileCount(&S));
      CGF.EmitBlock(ThenBlock);
      CGF.incrementProfileCounter(&S);
    }

    // Emit the loop iteration variable.
    const Expr *IVExpr = S.getIterationVariable();
    const VarDecl *IVDecl = cast<VarDecl>(cast<DeclRefExpr>(IVExpr)->getDecl());
    CGF.EmitVarDecl(*IVDecl);
    CGF.EmitIgnoredExpr(S.getInit());

    // Emit the iterations count variable.
    // If it is not a variable, Sema decided to calculate iterations count on
    // each iteration (e.g., it is foldable into a constant).
    if (auto LIExpr = dyn_cast<DeclRefExpr>(S.getLastIteration())) {
      CGF.EmitVarDecl(*cast<VarDecl>(LIExpr->getDecl()));
      // Emit calculation of the iterations count.
      CGF.EmitIgnoredExpr(S.getCalcLastIteration());
    }

    CGF.EmitOMPSimdInit(S);

    emitAlignedClause(CGF, S);
    CGF.EmitOMPLinearClauseInit(S);
    {
      OMPPrivateScope LoopScope(CGF);
      CGF.EmitOMPPrivateLoopCounters(S, LoopScope);
      CGF.EmitOMPLinearClause(S, LoopScope);
      CGF.EmitOMPPrivateClause(S, LoopScope);
      CGF.EmitOMPReductionClauseInit(S, LoopScope);
      bool HasLastprivateClause =
          CGF.EmitOMPLastprivateClauseInit(S, LoopScope);
      (void)LoopScope.Privatize();
      CGF.EmitOMPInnerLoop(S, LoopScope.requiresCleanups(), S.getCond(),
                           S.getInc(),
                           [&S](CodeGenFunction &CGF) {
                             CGF.EmitOMPLoopBody(S, JumpDest());
                             CGF.EmitStopPoint(&S);
                           },
                           [](CodeGenFunction &) {});
      CGF.EmitOMPSimdFinal(
          S, [](CodeGenFunction &) -> llvm::Value * { return nullptr; });
      // Emit final copy of the lastprivate variables at the end of loops.
      if (HasLastprivateClause)
        CGF.EmitOMPLastprivateClauseFinal(S, /*NoFinals=*/true);
      CGF.EmitOMPReductionClauseFinal(S);
      emitPostUpdateForReductionClause(
          CGF, S, [](CodeGenFunction &) -> llvm::Value * { return nullptr; });
    }
    CGF.EmitOMPLinearClauseFinal(
        S, [](CodeGenFunction &) -> llvm::Value * { return nullptr; });
    // Emit: if (PreCond) - end.
    if (ContBlock) {
      CGF.EmitBranch(ContBlock);
      CGF.EmitBlock(ContBlock, true);
    }
  };
  OMPLexicalScope Scope(*this, S, /*AsInlined=*/true);
  CGM.getOpenMPRuntime().emitInlinedDirective(*this, OMPD_simd, CodeGen);
}

void CodeGenFunction::EmitOMPOuterLoop(bool DynamicOrOrdered, bool IsMonotonic,
    const OMPLoopDirective &S, OMPPrivateScope &LoopScope, bool Ordered,
    Address LB, Address UB, Address ST, Address IL, llvm::Value *Chunk) {
  auto &RT = CGM.getOpenMPRuntime();

  const Expr *IVExpr = S.getIterationVariable();
  const unsigned IVSize = getContext().getTypeSize(IVExpr->getType());
  const bool IVSigned = IVExpr->getType()->hasSignedIntegerRepresentation();

  auto LoopExit = getJumpDestInCurrentScope("omp.dispatch.end");

  // Start the loop with a block that tests the condition.
  auto CondBlock = createBasicBlock("omp.dispatch.cond");
  EmitBlock(CondBlock);
  LoopStack.push(CondBlock, Builder.getCurrentDebugLocation());

  llvm::Value *BoolCondVal = nullptr;
  if (!DynamicOrOrdered) {
    // UB = min(UB, GlobalUB)
    EmitIgnoredExpr(S.getEnsureUpperBound());
    // IV = LB
    EmitIgnoredExpr(S.getInit());
    // IV < UB
    BoolCondVal = EvaluateExprAsBool(S.getCond());
  } else {
    BoolCondVal = RT.emitForNext(*this, S.getLocStart(), IVSize, IVSigned, IL,
                                 LB, UB, ST);
  }

  // If there are any cleanups between here and the loop-exit scope,
  // create a block to stage a loop exit along.
  auto ExitBlock = LoopExit.getBlock();
  if (LoopScope.requiresCleanups())
    ExitBlock = createBasicBlock("omp.dispatch.cleanup");

  auto LoopBody = createBasicBlock("omp.dispatch.body");
  Builder.CreateCondBr(BoolCondVal, LoopBody, ExitBlock);
  if (ExitBlock != LoopExit.getBlock()) {
    EmitBlock(ExitBlock);
    EmitBranchThroughCleanup(LoopExit);
  }
  EmitBlock(LoopBody);

  // Emit "IV = LB" (in case of static schedule, we have already calculated new
  // LB for loop condition and emitted it above).
  if (DynamicOrOrdered)
    EmitIgnoredExpr(S.getInit());

  // Create a block for the increment.
  auto Continue = getJumpDestInCurrentScope("omp.dispatch.inc");
  BreakContinueStack.push_back(BreakContinue(LoopExit, Continue));

  // Generate !llvm.loop.parallel metadata for loads and stores for loops
  // with dynamic/guided scheduling and without ordered clause.
  if (!isOpenMPSimdDirective(S.getDirectiveKind()))
    LoopStack.setParallel(!IsMonotonic);
  else
    EmitOMPSimdInit(S, IsMonotonic);

  SourceLocation Loc = S.getLocStart();
  EmitOMPInnerLoop(S, LoopScope.requiresCleanups(), S.getCond(), S.getInc(),
                   [&S, LoopExit](CodeGenFunction &CGF) {
                     CGF.EmitOMPLoopBody(S, LoopExit);
                     CGF.EmitStopPoint(&S);
                   },
                   [Ordered, IVSize, IVSigned, Loc](CodeGenFunction &CGF) {
                     if (Ordered) {
                       CGF.CGM.getOpenMPRuntime().emitForOrderedIterationEnd(
                           CGF, Loc, IVSize, IVSigned);
                     }
                   });

  EmitBlock(Continue.getBlock());
  BreakContinueStack.pop_back();
  if (!DynamicOrOrdered) {
    // Emit "LB = LB + Stride", "UB = UB + Stride".
    EmitIgnoredExpr(S.getNextLowerBound());
    EmitIgnoredExpr(S.getNextUpperBound());
  }

  EmitBranch(CondBlock);
  LoopStack.pop();
  // Emit the fall-through block.
  EmitBlock(LoopExit.getBlock());

  // Tell the runtime we are done.
  if (!DynamicOrOrdered)
    RT.emitForStaticFinish(*this, S.getLocEnd());
}

void CodeGenFunction::EmitOMPForOuterLoop(
    const OpenMPScheduleTy &ScheduleKind, bool IsMonotonic,
    const OMPLoopDirective &S, OMPPrivateScope &LoopScope, bool Ordered,
    Address LB, Address UB, Address ST, Address IL, llvm::Value *Chunk) {
  auto &RT = CGM.getOpenMPRuntime();

  // Dynamic scheduling of the outer loop (dynamic, guided, auto, runtime).
  const bool DynamicOrOrdered =
      Ordered || RT.isDynamic(ScheduleKind.Schedule);

  assert((Ordered ||
          !RT.isStaticNonchunked(ScheduleKind.Schedule,
                                 /*Chunked=*/Chunk != nullptr)) &&
         "static non-chunked schedule does not need outer loop");

  // Emit outer loop.
  //
  // OpenMP [2.7.1, Loop Construct, Description, table 2-1]
  // When schedule(dynamic,chunk_size) is specified, the iterations are
  // distributed to threads in the team in chunks as the threads request them.
  // Each thread executes a chunk of iterations, then requests another chunk,
  // until no chunks remain to be distributed. Each chunk contains chunk_size
  // iterations, except for the last chunk to be distributed, which may have
  // fewer iterations. When no chunk_size is specified, it defaults to 1.
  //
  // When schedule(guided,chunk_size) is specified, the iterations are assigned
  // to threads in the team in chunks as the executing threads request them.
  // Each thread executes a chunk of iterations, then requests another chunk,
  // until no chunks remain to be assigned. For a chunk_size of 1, the size of
  // each chunk is proportional to the number of unassigned iterations divided
  // by the number of threads in the team, decreasing to 1. For a chunk_size
  // with value k (greater than 1), the size of each chunk is determined in the
  // same way, with the restriction that the chunks do not contain fewer than k
  // iterations (except for the last chunk to be assigned, which may have fewer
  // than k iterations).
  //
  // When schedule(auto) is specified, the decision regarding scheduling is
  // delegated to the compiler and/or runtime system. The programmer gives the
  // implementation the freedom to choose any possible mapping of iterations to
  // threads in the team.
  //
  // When schedule(runtime) is specified, the decision regarding scheduling is
  // deferred until run time, and the schedule and chunk size are taken from the
  // run-sched-var ICV. If the ICV is set to auto, the schedule is
  // implementation defined
  //
  // while(__kmpc_dispatch_next(&LB, &UB)) {
  //   idx = LB;
  //   while (idx <= UB) { BODY; ++idx;
  //   __kmpc_dispatch_fini_(4|8)[u](); // For ordered loops only.
  //   } // inner loop
  // }
  //
  // OpenMP [2.7.1, Loop Construct, Description, table 2-1]
  // When schedule(static, chunk_size) is specified, iterations are divided into
  // chunks of size chunk_size, and the chunks are assigned to the threads in
  // the team in a round-robin fashion in the order of the thread number.
  //
  // while(UB = min(UB, GlobalUB), idx = LB, idx < UB) {
  //   while (idx <= UB) { BODY; ++idx; } // inner loop
  //   LB = LB + ST;
  //   UB = UB + ST;
  // }
  //

  const Expr *IVExpr = S.getIterationVariable();
  const unsigned IVSize = getContext().getTypeSize(IVExpr->getType());
  const bool IVSigned = IVExpr->getType()->hasSignedIntegerRepresentation();

  if (DynamicOrOrdered) {
    llvm::Value *UBVal = EmitScalarExpr(S.getLastIteration());
    RT.emitForDispatchInit(*this, S.getLocStart(), ScheduleKind, IVSize,
                           IVSigned, Ordered, UBVal, Chunk);
  } else {
    RT.emitForStaticInit(*this, S.getLocStart(), ScheduleKind, IVSize, IVSigned,
                         Ordered, IL, LB, UB, ST, Chunk);
  }

  EmitOMPOuterLoop(DynamicOrOrdered, IsMonotonic, S, LoopScope, Ordered, LB, UB,
                   ST, IL, Chunk);
}

void CodeGenFunction::EmitOMPDistributeOuterLoop(
    OpenMPDistScheduleClauseKind ScheduleKind,
    const OMPDistributeDirective &S, OMPPrivateScope &LoopScope,
    Address LB, Address UB, Address ST, Address IL, llvm::Value *Chunk) {

  auto &RT = CGM.getOpenMPRuntime();

  // Emit outer loop.
  // Same behavior as a OMPForOuterLoop, except that schedule cannot be
  // dynamic
  //

  const Expr *IVExpr = S.getIterationVariable();
  const unsigned IVSize = getContext().getTypeSize(IVExpr->getType());
  const bool IVSigned = IVExpr->getType()->hasSignedIntegerRepresentation();

  RT.emitDistributeStaticInit(*this, S.getLocStart(), ScheduleKind,
                              IVSize, IVSigned, /* Ordered = */ false,
                              IL, LB, UB, ST, Chunk);

  EmitOMPOuterLoop(/* DynamicOrOrdered = */ false, /* IsMonotonic = */ false,
                   S, LoopScope, /* Ordered = */ false, LB, UB, ST, IL, Chunk);
}

void CodeGenFunction::EmitOMPDistributeParallelForDirective(
    const OMPDistributeParallelForDirective &S) {
  OMPLexicalScope Scope(*this, S, /*AsInlined=*/true);
  CGM.getOpenMPRuntime().emitInlinedDirective(
      *this, OMPD_distribute_parallel_for,
      [&S](CodeGenFunction &CGF, PrePostActionTy &) {
        OMPLoopScope PreInitScope(CGF, S);
        CGF.EmitStmt(
            cast<CapturedStmt>(S.getAssociatedStmt())->getCapturedStmt());
      });
}

void CodeGenFunction::EmitOMPDistributeParallelForSimdDirective(
    const OMPDistributeParallelForSimdDirective &S) {
  OMPLexicalScope Scope(*this, S, /*AsInlined=*/true);
  CGM.getOpenMPRuntime().emitInlinedDirective(
      *this, OMPD_distribute_parallel_for_simd,
      [&S](CodeGenFunction &CGF, PrePostActionTy &) {
        OMPLoopScope PreInitScope(CGF, S);
        CGF.EmitStmt(
            cast<CapturedStmt>(S.getAssociatedStmt())->getCapturedStmt());
      });
}

void CodeGenFunction::EmitOMPDistributeSimdDirective(
    const OMPDistributeSimdDirective &S) {
  OMPLexicalScope Scope(*this, S, /*AsInlined=*/true);
  CGM.getOpenMPRuntime().emitInlinedDirective(
      *this, OMPD_distribute_simd,
      [&S](CodeGenFunction &CGF, PrePostActionTy &) {
        OMPLoopScope PreInitScope(CGF, S);
        CGF.EmitStmt(
            cast<CapturedStmt>(S.getAssociatedStmt())->getCapturedStmt());
      });
}

void CodeGenFunction::EmitOMPTargetParallelForSimdDirective(
    const OMPTargetParallelForSimdDirective &S) {
  OMPLexicalScope Scope(*this, S, /*AsInlined=*/true);
  CGM.getOpenMPRuntime().emitInlinedDirective(
      *this, OMPD_target_parallel_for_simd,
      [&S](CodeGenFunction &CGF, PrePostActionTy &) {
        OMPLoopScope PreInitScope(CGF, S);
        CGF.EmitStmt(
            cast<CapturedStmt>(S.getAssociatedStmt())->getCapturedStmt());
      });
}

/// \brief Emit a helper variable and return corresponding lvalue.
static LValue EmitOMPHelperVar(CodeGenFunction &CGF,
                               const DeclRefExpr *Helper) {
  auto VDecl = cast<VarDecl>(Helper->getDecl());
  CGF.EmitVarDecl(*VDecl);
  return CGF.EmitLValue(Helper);
}

namespace {
  struct ScheduleKindModifiersTy {
    OpenMPScheduleClauseKind Kind;
    OpenMPScheduleClauseModifier M1;
    OpenMPScheduleClauseModifier M2;
    ScheduleKindModifiersTy(OpenMPScheduleClauseKind Kind,
                            OpenMPScheduleClauseModifier M1,
                            OpenMPScheduleClauseModifier M2)
        : Kind(Kind), M1(M1), M2(M2) {}
  };
} // namespace

bool CodeGenFunction::EmitOMPWorksharingLoop(const OMPLoopDirective &S) {
  // Emit the loop iteration variable.
  auto IVExpr = cast<DeclRefExpr>(S.getIterationVariable());
  auto IVDecl = cast<VarDecl>(IVExpr->getDecl());
  EmitVarDecl(*IVDecl);

  // Emit the iterations count variable.
  // If it is not a variable, Sema decided to calculate iterations count on each
  // iteration (e.g., it is foldable into a constant).
  if (auto LIExpr = dyn_cast<DeclRefExpr>(S.getLastIteration())) {
    EmitVarDecl(*cast<VarDecl>(LIExpr->getDecl()));
    // Emit calculation of the iterations count.
    EmitIgnoredExpr(S.getCalcLastIteration());
  }

  auto &RT = CGM.getOpenMPRuntime();

  bool HasLastprivateClause;
  // Check pre-condition.
  {
    OMPLoopScope PreInitScope(*this, S);
    // Skip the entire loop if we don't meet the precondition.
    // If the condition constant folds and can be elided, avoid emitting the
    // whole loop.
    bool CondConstant;
    llvm::BasicBlock *ContBlock = nullptr;
    if (ConstantFoldsToSimpleInteger(S.getPreCond(), CondConstant)) {
      if (!CondConstant)
        return false;
    } else {
      auto *ThenBlock = createBasicBlock("omp.precond.then");
      ContBlock = createBasicBlock("omp.precond.end");
      emitPreCond(*this, S, S.getPreCond(), ThenBlock, ContBlock,
                  getProfileCount(&S));
      EmitBlock(ThenBlock);
      incrementProfileCounter(&S);
    }

    bool Ordered = false;
    if (auto *OrderedClause = S.getSingleClause<OMPOrderedClause>()) {
      if (OrderedClause->getNumForLoops())
        RT.emitDoacrossInit(*this, S);
      else
        Ordered = true;
    }

    llvm::DenseSet<const Expr *> EmittedFinals;
    emitAlignedClause(*this, S);
    EmitOMPLinearClauseInit(S);
    // Emit helper vars inits.
    LValue LB =
        EmitOMPHelperVar(*this, cast<DeclRefExpr>(S.getLowerBoundVariable()));
    LValue UB =
        EmitOMPHelperVar(*this, cast<DeclRefExpr>(S.getUpperBoundVariable()));
    LValue ST =
        EmitOMPHelperVar(*this, cast<DeclRefExpr>(S.getStrideVariable()));
    LValue IL =
        EmitOMPHelperVar(*this, cast<DeclRefExpr>(S.getIsLastIterVariable()));

    // Emit 'then' code.
    {
      OMPPrivateScope LoopScope(*this);
      if (EmitOMPFirstprivateClause(S, LoopScope)) {
        // Emit implicit barrier to synchronize threads and avoid data races on
        // initialization of firstprivate variables and post-update of
        // lastprivate variables.
        CGM.getOpenMPRuntime().emitBarrierCall(
            *this, S.getLocStart(), OMPD_unknown, /*EmitChecks=*/false,
            /*ForceSimpleCall=*/true);
      }
      EmitOMPPrivateClause(S, LoopScope);
      HasLastprivateClause = EmitOMPLastprivateClauseInit(S, LoopScope);
      EmitOMPReductionClauseInit(S, LoopScope);
      EmitOMPPrivateLoopCounters(S, LoopScope);
      EmitOMPLinearClause(S, LoopScope);
      (void)LoopScope.Privatize();

      // Detect the loop schedule kind and chunk.
      llvm::Value *Chunk = nullptr;
      OpenMPScheduleTy ScheduleKind;
      if (auto *C = S.getSingleClause<OMPScheduleClause>()) {
        ScheduleKind.Schedule = C->getScheduleKind();
        ScheduleKind.M1 = C->getFirstScheduleModifier();
        ScheduleKind.M2 = C->getSecondScheduleModifier();
        if (const auto *Ch = C->getChunkSize()) {
          Chunk = EmitScalarExpr(Ch);
          Chunk = EmitScalarConversion(Chunk, Ch->getType(),
                                       S.getIterationVariable()->getType(),
                                       S.getLocStart());
        }
      }
      const unsigned IVSize = getContext().getTypeSize(IVExpr->getType());
      const bool IVSigned = IVExpr->getType()->hasSignedIntegerRepresentation();
      // OpenMP 4.5, 2.7.1 Loop Construct, Description.
      // If the static schedule kind is specified or if the ordered clause is
      // specified, and if no monotonic modifier is specified, the effect will
      // be as if the monotonic modifier was specified.
      if (RT.isStaticNonchunked(ScheduleKind.Schedule,
                                /* Chunked */ Chunk != nullptr) &&
          !Ordered) {
        if (isOpenMPSimdDirective(S.getDirectiveKind()))
          EmitOMPSimdInit(S, /*IsMonotonic=*/true);
        // OpenMP [2.7.1, Loop Construct, Description, table 2-1]
        // When no chunk_size is specified, the iteration space is divided into
        // chunks that are approximately equal in size, and at most one chunk is
        // distributed to each thread. Note that the size of the chunks is
        // unspecified in this case.
        RT.emitForStaticInit(*this, S.getLocStart(), ScheduleKind,
                             IVSize, IVSigned, Ordered,
                             IL.getAddress(), LB.getAddress(),
                             UB.getAddress(), ST.getAddress());
        auto LoopExit =
            getJumpDestInCurrentScope(createBasicBlock("omp.loop.exit"));
        // UB = min(UB, GlobalUB);
        EmitIgnoredExpr(S.getEnsureUpperBound());
        // IV = LB;
        EmitIgnoredExpr(S.getInit());
        // while (idx <= UB) { BODY; ++idx; }
        EmitOMPInnerLoop(S, LoopScope.requiresCleanups(), S.getCond(),
                         S.getInc(),
                         [&S, LoopExit](CodeGenFunction &CGF) {
                           CGF.EmitOMPLoopBody(S, LoopExit);
                           CGF.EmitStopPoint(&S);
                         },
                         [](CodeGenFunction &) {});
        EmitBlock(LoopExit.getBlock());
        // Tell the runtime we are done.
        RT.emitForStaticFinish(*this, S.getLocStart());
      } else {
        const bool IsMonotonic =
            Ordered || ScheduleKind.Schedule == OMPC_SCHEDULE_static ||
            ScheduleKind.Schedule == OMPC_SCHEDULE_unknown ||
            ScheduleKind.M1 == OMPC_SCHEDULE_MODIFIER_monotonic ||
            ScheduleKind.M2 == OMPC_SCHEDULE_MODIFIER_monotonic;
        // Emit the outer loop, which requests its work chunk [LB..UB] from
        // runtime and runs the inner loop to process it.
        EmitOMPForOuterLoop(ScheduleKind, IsMonotonic, S, LoopScope, Ordered,
                            LB.getAddress(), UB.getAddress(), ST.getAddress(),
                            IL.getAddress(), Chunk);
      }
      if (isOpenMPSimdDirective(S.getDirectiveKind())) {
        EmitOMPSimdFinal(S,
                         [&](CodeGenFunction &CGF) -> llvm::Value * {
                           return CGF.Builder.CreateIsNotNull(
                               CGF.EmitLoadOfScalar(IL, S.getLocStart()));
                         });
      }
      EmitOMPReductionClauseFinal(S);
      // Emit post-update of the reduction variables if IsLastIter != 0.
      emitPostUpdateForReductionClause(
          *this, S, [&](CodeGenFunction &CGF) -> llvm::Value * {
            return CGF.Builder.CreateIsNotNull(
                CGF.EmitLoadOfScalar(IL, S.getLocStart()));
          });
      // Emit final copy of the lastprivate variables if IsLastIter != 0.
      if (HasLastprivateClause)
        EmitOMPLastprivateClauseFinal(
            S, isOpenMPSimdDirective(S.getDirectiveKind()),
            Builder.CreateIsNotNull(EmitLoadOfScalar(IL, S.getLocStart())));
    }
    EmitOMPLinearClauseFinal(S, [&](CodeGenFunction &CGF) -> llvm::Value * {
      return CGF.Builder.CreateIsNotNull(
          CGF.EmitLoadOfScalar(IL, S.getLocStart()));
    });
    // We're now done with the loop, so jump to the continuation block.
    if (ContBlock) {
      EmitBranch(ContBlock);
      EmitBlock(ContBlock, true);
    }
  }

  return HasLastprivateClause;
}

bool isParallel = true;
bool prevState = true;
std::vector<std::vector<Stmt *> > blocos = {}, def = {}, use = {};
std::vector<bool> blocosTy = {};
std::set<VarDecl *> varset = {};
std::vector<std::vector<VarDecl *> > blocosVars = {};


std::vector<std::vector<Stmt *> >::iterator iDef;
std::vector<std::vector<Stmt *> >::iterator iUse;

bool isRHS = false;
bool isArray = false;
bool isIndex = false;
int state = 0;
/*
bool compareStmt(Stmt *A, Stmt*B) {
	if(A->getStmtClass() != B->getStmtClass()) return false;
	if(isa<DeclRefExpr>(A)) {
		DeclRefExpr *declexprA = cast<DeclRefExpr>(A);
		DeclRefExpr *declexprB = cast<DeclRefExpr>(B);
		if((declexprA->getDecl())->getKind() == Decl::Var && (declexprB->getDecl())->getKind() == Decl::Var) {
			VarDecl *declA = cast<VarDecl>(declexprA->getDecl());
			VarDecl *declB = cast<VarDecl>(declexprB->getDecl());
			if(declA != declB) return false;
			else return true;
		}
		else return false;
	}
	auto cdB = B->child_begin();
	for(auto cdA = A->child_begin(); cdA != A->child_end(); cdA++, cdB++) {
		return compareStmt(*cdA, *cdB);
	}
}
*/

/*void handleDeclRefExpr2(CodeGenFunction &CGF, Stmt **cd) {
//	llvm::errs() << "DECLREFEXPR!\n";
//	(*cd)->dumpColor();
//	llvm::errs() << "---\n";
	DeclRefExpr * declexpr = cast<DeclRefExpr>(*cd);
	if((declexpr->getDecl())->getKind() == Decl::Var) {
		VarDecl *vd = cast<VarDecl>(declexpr->getDecl());
		if(!varset.count(vd)) {
			varset.insert(vd);
			if(!isArray) {
				if(isRHS) {
					bool member = false;
					for(auto it = (*iDef).begin(); it != (*iDef).end(); it++) {
						if(isa<DeclRefExpr>(*it)) {
							DeclRefExpr *declexprA = cast<DeclRefExpr>(*it);
							VarDecl *declA = cast<VarDecl>(declexprA->getDecl());
							if(declA == vd) {
								member = true;
								break;
							}
						}
					}
					if(!member)
						(*iUse).push_back(*cd);
					int dep = 0;
					for(auto it = def.begin(); it != iDef; it++) {
						for(auto it2 = (*it).begin(); it2 != (*it).end(); it2++) {
							if(isa<DeclRefExpr>(*it2)) {
								DeclRefExpr *declexprA = cast<DeclRefExpr>(*it2);
								VarDecl *declA = cast<VarDecl>(declexprA->getDecl());
								if(declA == vd) {
									(*it2)->dumpPretty(CGF.getContext());
									llvm::errs() << ": Dependence with component " << std::to_string(dep) << "\n";
								}
							}
						}
						dep++;
					}
				}
				else (*iDef).push_back(*cd);
			}
			else if(isIndex && !isRHS) {
				llvm::errs() << "ENTROU!\n";
				(*cd)->dumpColor();
			}
		}
	}
}*/

void checkDeclRefExpr(CodeGenFunction &CGF, Stmt *body);

void handleDeclRefExpr(CodeGenFunction &CGF, Stmt **cd) {
	DeclRefExpr * declexpr = cast<DeclRefExpr>((*cd)->IgnoreImplicit());
	if((declexpr->getDecl())->getKind() == Decl::Var) {
		VarDecl *vd = cast<VarDecl>(declexpr->getDecl());
		if(!varset.count(vd)) {
			varset.insert(vd);
			if(isRHS || isIndex) {
				bool member = false;
				for(auto it = (*iDef).begin(); it != (*iDef).end(); it++) {
					if(isa<DeclRefExpr>(*it)) {
						DeclRefExpr *declexprA = cast<DeclRefExpr>(*it);
						VarDecl *declA = cast<VarDecl>(declexprA->getDecl());
						if(declA == vd) {
							member = true;
							break;
						}
					}
				}
				if(!member) {
					(*iUse).push_back(*cd);
				}
				int dep = 0;
				for(auto it = def.begin(); it != iDef; it++) {
					for(auto it2 = (*it).begin(); it2 != (*it).end(); it2++) {
						if(isa<DeclRefExpr>(*it2)) {
							DeclRefExpr *declexprA = cast<DeclRefExpr>(*it2);
							VarDecl *declA = cast<VarDecl>(declexprA->getDecl());
							if(declA == vd) {
								(*it2)->dumpPretty(CGF.getContext());
								llvm::errs() << ": Dependence with component " << std::to_string(dep) << "\n";
							}
						}
					}
					dep++;
				}

			}
			else {
				(*iDef).push_back(*cd);
			}
		}
	}
}

void handleArraySubscriptExpr(CodeGenFunction &CGF, Stmt **cd) {
	ArraySubscriptExpr *array = cast<ArraySubscriptExpr>((*cd)->IgnoreImplicit());
	Stmt *base = (array->getBase())->IgnoreImplicit();
	Stmt *idx = array->getIdx();
	DeclRefExpr *declexpr = cast<DeclRefExpr>(base);
	VarDecl *vd = cast<VarDecl>(declexpr->getDecl());
	if(!varset.count(vd)) varset.insert(vd);
	if(isRHS) {
		int dep = 0;
		for(auto it = def.begin(); it != def.end(); it++) {
			for(auto it2 = (*it).begin(); it2 != (*it).end(); it2++) {
				if(isa<ArraySubscriptExpr>(*it2)) {
					ArraySubscriptExpr *array2 = cast<ArraySubscriptExpr>(*it2);
					Stmt *base2 = (array2->getBase())->IgnoreImplicit();
					DeclRefExpr *declexprA = cast<DeclRefExpr>(base2);
					VarDecl *declA = cast<VarDecl>(declexprA->getDecl());
					if(declA == vd) {
						(*it2)->dumpPretty(CGF.getContext());
						if(it == iDef) {
							llvm::errs() << ", ";
							(array)->dumpPretty(CGF.getContext());
							llvm::errs() << ": Loop-Carried Dependence\n";
						}
						else
							llvm::errs() << ": Dependence with component " << std::to_string(dep) << "\n";
					}
				}
			}
			dep++;
		}
		(*iUse).push_back(*cd);
	}
	else (*iDef).push_back(*cd);

	bool tempisIndex = isIndex;
	isIndex = true;
	checkDeclRefExpr(CGF, idx);
	isIndex = tempisIndex;
}

void checkDeclRefExpr(CodeGenFunction &CGF, Stmt *body) {
	bool checkChild = true;

	if(isa<DeclRefExpr>(body->IgnoreImplicit())) {
		handleDeclRefExpr(CGF, &body);
		checkChild = false;
	}
	else if(isa<ArraySubscriptExpr>(body->IgnoreImplicit())) {
		handleArraySubscriptExpr(CGF, &body);
		checkChild = false;
	}
	else if(isa<BinaryOperator>(body->IgnoreImplicit())) {
		BinaryOperator *BinOp = cast<BinaryOperator>(body);

		if(BinOp->isAssignmentOp()) {
			Stmt *lhs, *rhs;
			lhs = BinOp->getLHS();
			rhs = BinOp->getRHS();
			isRHS = false;
			checkDeclRefExpr(CGF, lhs);
			isRHS = true;
			checkDeclRefExpr(CGF, rhs);
			isRHS = false;
			checkChild = false;
		}
	}

	if(checkChild) {
		for(auto cd = body->child_begin();cd != body->child_end();cd++) {
			if(*cd != NULL) {
				checkDeclRefExpr(CGF, *cd);
			}
		}
	}

	return;
}

DeclContext * declCTX = nullptr;
bool finishCheck = false;

// Recursively try to find the declaration context for the first declaration on AST
void checkDeclRefExpr2(Stmt *body) {
	bool checkChild = true;
  if(!finishCheck) {
    if(isa<DeclRefExpr>(body->IgnoreImplicit()) && declCTX == nullptr) {
      DeclRefExpr *declexprA = cast<DeclRefExpr>(body->IgnoreImplicit());
      if(isa<VarDecl>(declexprA->getDecl())) {
        Decl *declA = cast<Decl>(declexprA->getDecl());
        llvm::errs() << "Getting DeclContext from\n";
        declexprA->dumpColor();
        declCTX = declA->getDeclContext();
        if(declCTX != nullptr) {
          finishCheck = true;
          return;
        }
      }
    }
    if(checkChild) {
      for(auto cd = (body->IgnoreImplicit())->child_begin();cd !=
          (body->IgnoreImplicit())->child_end();cd++) {
        if(*cd != NULL) {
          checkDeclRefExpr2(*cd);
        }
      }
    }
  }
	return;
}

bool checkOrdered(Stmt *body) {
	bool ret = false;
  for(auto cd = body->child_begin();cd != body->child_end();cd++) {
    if(*cd != NULL) {
      if((*cd)->getStmtClass() == Stmt::OMPOrderedDirectiveClass) {
        ret = true;
      }
      else {
        ret |= checkOrdered(*cd);
      }
    }
  }
	return ret;
}

// Recursively visit AST nodes to find loop components separated by 'ordered' directive
void visitBody(Stmt *body) {
  for(auto cd = body->child_begin();cd != body->child_end();cd++) {
    if((*cd)->getStmtClass() == Stmt::OMPOrderedDirectiveClass) {
      const OMPOrderedDirective *ord = cast<OMPOrderedDirective>(*cd);
      for(auto cl = ord->clauses().begin(); cl != ord->clauses().end(); cl++) {
        if(isa<OMPDependClause>(*cl)) {
          OMPDependClause *depend = cast<OMPDependClause>(*cl);
          if (depend) {
            if (depend->getDependencyKind() == OMPC_DEPEND_source) {
              isParallel = true;
              prevState = false;
            }
            else if (depend->getDependencyKind() == OMPC_DEPEND_sink) {
              isParallel = false;
              prevState = true;
            } else if (depend->getDependencyKind() == OMPC_DEPEND_var) {
              clang::OMPVarListClause<OMPDependClause> *vars =
                cast<clang::OMPVarListClause<OMPDependClause> >(depend);

              std::vector<VarDecl*> tmp = {};
              for (auto vr = vars->varlist_begin(); vr != vars->varlist_end(); vr++) {
                DeclRefExpr *dre = cast<DeclRefExpr>(*vr);
                VarDecl *vd = cast<VarDecl>(dre->getDecl());
                tmp.push_back(vd);
              }
              blocosVars.push_back(tmp);
            }
          }
        }
      }
    }
    else if((*cd)->getStmtClass() == Stmt::CompoundStmtClass) {
      visitBody(*cd);
    }
    else {

      if((*cd)->getStmtClass() == Stmt::ForStmtClass) {
        Stmt *subbody = (cast<ForStmt>(*cd))->getBody();
        if(checkOrdered(subbody)) {
          blocos.clear();
          blocosTy.clear();
          isParallel = true;
          prevState = true;
          visitBody(subbody);
          break;
        }
      }
      if(prevState == isParallel && !blocos.empty()) {
        blocos.back().push_back(*cd);
      }
      else {
        std::vector<Stmt*> vct = {*cd};
        std::vector<Stmt*> tempDef = {};
        std::vector<Stmt*> tempUse = {};
        blocos.push_back(vct);
        def.push_back(tempDef);
        use.push_back(tempUse);
        blocosTy.push_back(isParallel);
      }
      prevState = isParallel;
    }
  }
}


void visitBody2(Stmt *body) {
	for(auto cd = body->child_begin();cd != body->child_end();cd++) {
		if((*cd)->getStmtClass() == Stmt::OMPOrderedDirectiveClass) {
			const OMPOrderedDirective *ord = cast<OMPOrderedDirective>(*cd);
            for(auto cl = ord->clauses().begin(); cl != ord->clauses().end(); cl++) {
                if(isa<OMPDependClause>(*cl)) {
//                    OMPDependClause *depend = cast<OMPDependClause>(*cl);
                    //if (depend) {
                        //if (depend->getDependencyKind() == OMPC_DEPEND_source) { isParallel = true; prevState = false;}
                        //else if (depend->getDependencyKind() == OMPC_DEPEND_sink) { isParallel = false; prevState = true;}
                        /* not yet
                        else if (depend->getDependencyKind() == OMPC_DEPEND_var) {


                           clang::OMPVarListClause<OMPDependClause> *vars = cast<clang::OMPVarListClause<OMPDependClause> >(depend);

                            std::vector<VarDecl*> tmp = {};
                            for (auto vr = vars->varlist_begin(); vr != vars->varlist_end(); vr++) {
                                DeclRefExpr *dre = cast<DeclRefExpr>(*vr);
                                VarDecl *vd = cast<VarDecl>(dre->getDecl());
                                tmp.push_back(vd);
                            }
                            blocosVars.push_back(tmp);

                        }
                        */
                    //}
                }
            }
		}
		else if((*cd)->getStmtClass() == Stmt::CompoundStmtClass) {
			visitBody(*cd);
		}
		else {

			if((*cd)->getStmtClass() == Stmt::ForStmtClass) {
				Stmt *subbody = (cast<ForStmt>(*cd))->getBody();
				if(checkOrdered(subbody)) {
					blocos.clear();
					//blocosTy.clear();
					//isParallel = true;
					//prevState = true;
					visitBody(subbody);
					break;
				}
			}
				if(prevState == isParallel && !blocos.empty()) {
					blocos.back().push_back(*cd);
				}
				else {
					std::vector<Stmt*> vct = {*cd};
					//std::vector<Stmt*> tempDef = {};
					//std::vector<Stmt*> tempUse = {};
					blocos.push_back(vct);
					//def.push_back(tempDef);
					//use.push_back(tempUse);
					//blocosTy.push_back(isParallel);
				}
				//prevState = isParallel;
		}
	}
}

// Recursively try to find the declaration context for the first declaration on AST
void ReplaceVarDecl(Stmt *body, VarDecl *A, VarDecl *B) {

//llvm::errs () << body << " - " << A << " - " << B << "\n";

	if(isa<DeclRefExpr>(body->IgnoreImplicit())) {
		DeclRefExpr *declexprA = dyn_cast<DeclRefExpr>(body->IgnoreImplicit());
		VarDecl *declA = dyn_cast<VarDecl>(declexprA->getDecl());
		if(declA == A) {
			declexprA->setDecl(B);
		}
		return;
	}

	for(auto cd = body->child_begin();cd != body->child_end();cd++) {
		if(*cd != NULL) {
			ReplaceVarDecl(*cd, A, B);
		}
	}

	return;
}

void findDeclRefExpr(Stmt *st, std::vector<VarDecl *> &list) {
	if(st == NULL) return;
	if(isa<DeclRefExpr>(st)) {
		DeclRefExpr *dre = cast<DeclRefExpr>(st->IgnoreImplicit());
		if(isa<VarDecl>(dre->getDecl())) {
			VarDecl *declA = cast<VarDecl>(dre->getDecl());
			list.push_back(declA);
		}
	}
	else {
		for(auto cd = st->child_begin(); cd != st->child_end(); cd++) {
			findDeclRefExpr(*cd, list);
		}
	}
}

void CodeGenFunction::EmitOMPForDirective(const OMPForDirective &S) {
  bool HasLastprivates = false;
  auto &&CodeGen = [&S, &HasLastprivates](CodeGenFunction &CGF,
                                          PrePostActionTy &) {
    HasLastprivates = CGF.EmitOMPWorksharingLoop(S);
  };

  OMPLexicalScope Scope(*this, S, /*AsInlined=*/true);
	CGM.getOpenMPRuntime().emitInlinedDirective(*this, OMPD_for, CodeGen,
		                                            S.hasCancel());

	// Emit an implicit barrier at the end.
	if (!S.getSingleClause<OMPNowaitClause>() || HasLastprivates) {
    CGM.getOpenMPRuntime().emitBarrierCall(*this, S.getLocStart(), OMPD_for);
	}
#if 0
// Luis Felipe - Choose the Use clause type to handle
	const OMPUseClause *C = S.getSingleClause<OMPUseClause>();
	const OMPOrderedClause *O = S.getSingleClause<OMPOrderedClause>();

	if(C) {
	    llvm::errs() << "entre al if\n";
		if(!O) {
			llvm::errs() << "The 'use' clause must be used with the 'ordered' clause!\n";
			return;
		}

		int num_loops = (dyn_cast<IntegerLiteral>(O->getNumForLoops())->getValue()).getLimitedValue();

		if(C->getUseKind() == OMPC_USE_dswp) llvm::errs() << "dswp\n";
		else if(C->getUseKind() == OMPC_USE_psbdx || C->getUseKind() == OMPC_USE_bdx) {
            IdentifierTable *idt2 = new IdentifierTable(CGM.getLangOpts());
            //CodeGenFunction CGF(CGM);


			llvm::errs() << "bdx ("<< num_loops <<")\n";

			FunctionArgList TemplateArgsCond, TemplateArgs, TemplateArgsBegin;

			// Get the original loop body, associated with the 'parallel for' directive
			Stmt *Body = S.getAssociatedStmt()->IgnoreContainers(true);

			Body = dyn_cast<ForStmt>(Body)->getBody();
	
			// Call the function to separate the body into loop components, based on the 'ordered' directive
			visitBody(Body);
			checkDeclRefExpr2(Body);

            S.dumpPretty(getContext());
			//S.dumpColor();

			int num_components = blocos.size();
			
			llvm::errs() << "numero componentes "<< num_components << "\n";

			if(num_components > 0) {

				//findDeps();

//				auto *GV = new llvm::GlobalVariable(CGM.getModule(), llvm::ArrayType::get(llvm::ArrayType::getInt32Ty(CGM.getLLVMContext()), num_components), false, llvm::GlobalValue::LinkageTypes::CommonLinkage, initArray, "__bdx_flags");

//				declCTX->dumpDeclContext();


/*				VarDecl *vdPost;
				VarDecl *vdBufferSize;
				NestedNameSpecifierLoc *nnsl = new NestedNameSpecifierLoc();

				QualType qt = getContext().getConstantArrayType(getContext().IntTy, llvm::APInt(getContext().getTypeSize(getContext().IntTy), (uint64_t) num_components), ArrayType::ArraySizeModifier::Normal, 0);

//				NestedNameSpecifierLoc *nnsl2 = new NestedNameSpecifierLoc();
				IdentifierInfo* iiFlags = &(idt2->get("__bdx_flags"));
				DeclarationName *dnFlags = new DeclarationName(iiFlags);
				DeclarationNameInfo *dniFlags = new DeclarationNameInfo(*dnFlags, SourceLocation());
				vdPost = VarDecl::Create(getContext(), declCTX, SourceLocation(), SourceLocation(), iiFlags, qt, nullptr, StorageClass::SC_None);*/



				std::vector<VarDecl *> bdx_flags;
				VarDecl *vdPost;
				VarDecl *vdBufferSize;
				NestedNameSpecifierLoc *nnsl = new NestedNameSpecifierLoc();

				QualType qt = getContext().getVolatileType(getContext().IntTy);

				IdentifierInfo* iiFlags = &(idt2->get("__bdx_flags"));
				DeclarationName *dnFlags = new DeclarationName(iiFlags);
				DeclarationNameInfo *dniFlags = new DeclarationNameInfo(*dnFlags, SourceLocation());

				auto cTy = blocosTy.begin();
				for(int i=0;i<num_components;i++, cTy++) {
					vdPost = VarDecl::Create(getContext(), declCTX, SourceLocation(), SourceLocation(), iiFlags, qt, nullptr, StorageClass::SC_None);
					vdPost->setInit(IntegerLiteral::Create(getContext(), llvm::APInt(getContext().getTypeSize(getContext().IntTy), 0), getContext().IntTy, SourceLocation()));
					bdx_flags.push_back(vdPost);
				}

				// Create the identifier for using the condition function call
				IdentifierInfo* iiParam = &(idt2->get("bdx_stage_param"));

//				ImplicitParamDecl stageFlag(getContext(), declCTX, SourceLocation(), nullptr, getContext().IntTy);
				ParmVarDecl *stageFlag = ParmVarDecl::Create(getContext(), declCTX, SourceLocation(), SourceLocation(), iiParam, getContext().IntTy, nullptr, StorageClass::SC_None, nullptr);
	 			TemplateArgsCond.push_back(stageFlag);

				// Create the function declaration.
				const CGFunctionInfo &FuncInfoCond = CGM.getTypes().arrangeBuiltinFunctionDeclaration(getContext().IntTy, TemplateArgsCond);
				llvm::FunctionType *FuncLLVMTyCond = CGM.getTypes().GetFunctionType(FuncInfoCond);

				const CGFunctionInfo &FuncInfo = CGM.getTypes().arrangeBuiltinFunctionDeclaration(getContext().VoidTy, TemplateArgs);
				llvm::FunctionType *FuncLLVMTy = CGM.getTypes().GetFunctionType(FuncInfo);

				const CGFunctionInfo &FuncInfoNumThreads = CGM.getTypes().arrangeBuiltinFunctionDeclaration(getContext().IntTy, TemplateArgs);
				llvm::FunctionType *FuncLLVMTyNumThreads = CGM.getTypes().GetFunctionType(FuncInfoNumThreads);

				CGM.CreateRuntimeFunction(FuncLLVMTyCond, "__bdx_cond__");
				CGM.CreateRuntimeFunction(FuncLLVMTy, "__bdx_stage_end__");

				CGM.CreateRuntimeFunction(FuncLLVMTyNumThreads, "omp_get_thread_num");

				// Create a new identifiers table based on the current context
				IdentifierTable *idt = new IdentifierTable(CGM.getLangOpts());
				//NestedNameSpecifierLoc *nnsl = new NestedNameSpecifierLoc();

				// Create the identifier for using the condition function call
				IdentifierInfo* iiCond = &(idt->get("__bdx_cond__"));
				DeclarationName *dnCond = new DeclarationName(iiCond);
				DeclarationNameInfo *dniCond = new DeclarationNameInfo(*dnCond, SourceLocation());

				// Create the identifier for finding the start of the component
				IdentifierInfo* iiBegin = &(idt->get("__bdx_stage_begin__"));
				DeclarationName *dnBegin = new DeclarationName(iiBegin);
				DeclarationNameInfo *dniBegin = new DeclarationNameInfo(*dnBegin, SourceLocation());

				// Create the identifier for finding the end of the component
				IdentifierInfo* iiEnd = &(idt->get("__bdx_stage_end__"));
				DeclarationName *dnEnd = new DeclarationName(iiEnd);
				DeclarationNameInfo *dniEnd = new DeclarationNameInfo(*dnEnd, SourceLocation());

				// Create the identifier for using the condition function call
				IdentifierInfo* iiNumThreads = &(idt->get("omp_get_thread_num"));
				DeclarationName *dnNumThreads = new DeclarationName(iiNumThreads);
				DeclarationNameInfo *dniNumThreads = new DeclarationNameInfo(*dnNumThreads, SourceLocation());

				// Create the identifier for using the condition function call
				IdentifierInfo* iiNumThreads0 = &(idt->get("omp_get_num_threads"));
				DeclarationName *dnNumThreads0 = new DeclarationName(iiNumThreads0);
				DeclarationNameInfo *dniNumThreads0 = new DeclarationNameInfo(*dnNumThreads0, SourceLocation());

				// Create the 'FunctionDecl' nodes for inserting on the AST
				std::vector<QualType> argsCond, args;
				FunctionProtoType::ExtProtoInfo fpi;
				fpi.Variadic = false;

				argsCond.push_back(getContext().IntTy);

				QualType funcTypeCond = getContext().getFunctionType(getContext().IntTy, argsCond, fpi);
				QualType funcTypeNumThreads = getContext().getFunctionType(getContext().IntTy, None, fpi);
				QualType funcType = getContext().getFunctionType(getContext().VoidTy, None, fpi);

				FunctionDecl *FDCond = FunctionDecl::Create(getContext(), declCTX, SourceLocation(), SourceLocation(), *dnCond, funcTypeCond, nullptr, StorageClass::SC_None);
//				FunctionDecl *FDBegin = FunctionDecl::Create(getContext(), declCTX, SourceLocation(), SourceLocation(), *dnBegin, funcType, nullptr, StorageClass::SC_None);
				FunctionDecl *FDEnd = FunctionDecl::Create(getContext(), declCTX, SourceLocation(), SourceLocation(), *dnEnd, funcType, nullptr, StorageClass::SC_None);
				FunctionDecl *FDNumThreads = FunctionDecl::Create(getContext(), declCTX, SourceLocation(), SourceLocation(), *dnNumThreads, funcTypeNumThreads, nullptr, StorageClass::SC_Extern, false);
				FunctionDecl *FDNumThreads0 = FunctionDecl::Create(getContext(), declCTX, SourceLocation(), SourceLocation(), *dnNumThreads0, funcTypeNumThreads, nullptr, StorageClass::SC_Extern, false);

				std::vector<ParmVarDecl *> params;
				params.push_back(stageFlag);

				FDCond->setParams(params);

				// Auxiliary vector
				std::vector<Stmt *> ifStmts;

				std::vector<Expr *> ArgsCond, Args;

                Stmt *forInit = dyn_cast<ForStmt>(dyn_cast<CapturedStmt>(S.getAssociatedStmt())->getCapturedStmt())->getInit();
//                Expr *forCond = cast<ForStmt>(cast<CapturedStmt>(S.getAssociatedStmt())->getCapturedStmt())->getCond();
//                Expr *forInc = cast<ForStmt>(cast<CapturedStmt>(S.getAssociatedStmt())->getCapturedStmt())->getInc();
//				VarDecl *forCondVar = cast<ForStmt>(cast<CapturedStmt>(S.getAssociatedStmt())->getCapturedStmt())->getConditionVariable();

                BinaryOperator *boInit = dyn_cast<BinaryOperator>(forInit);
                VarDecl *forCondVar = dyn_cast<VarDecl>(dyn_cast<DeclRefExpr>(boInit->getLHS())->getDecl());
                //Expr *forCondVal = boInit->getRHS();




				IdentifierInfo* iiParam2 = &(idt2->get("bdx_stage_param"));
//				ImplicitParamDecl stageFlag(getContext(), declCTX, SourceLocation(), nullptr, getContext().IntTy);
				ParmVarDecl *stageFlag2 = ParmVarDecl::Create(getContext(), declCTX, SourceLocation(), SourceLocation(), iiParam2, forCondVar->getType(), nullptr, StorageClass::SC_None, nullptr);
	 			TemplateArgsBegin.push_back(stageFlag2);

				// Create the function declaration.
				const CGFunctionInfo &FuncInfo2 = CGM.getTypes().arrangeBuiltinFunctionDeclaration(getContext().VoidTy, TemplateArgsBegin);
				llvm::FunctionType *FuncLLVMTy2 = CGM.getTypes().GetFunctionType(FuncInfo2);

				CGM.CreateRuntimeFunction(FuncLLVMTy2, "__bdx_stage_begin__");

				argsCond.clear();
				argsCond.push_back(forCondVar->getType());

				QualType funcTypeBegin = getContext().getFunctionType(getContext().VoidTy, argsCond, fpi);

				FunctionDecl *FDBegin = FunctionDecl::Create(getContext(), declCTX, SourceLocation(), SourceLocation(), *dnBegin, funcTypeBegin, nullptr, StorageClass::SC_None);

				params.clear();
				params.push_back(stageFlag2);

				FDBegin->setParams(params);

				IdentifierInfo* iiBS = &(idt2->get("__bdx_buffer_size"));
				DeclarationName *dnBS = new DeclarationName(iiBS);
				DeclarationNameInfo *dniBS = new DeclarationNameInfo(*dnBS, SourceLocation());
				vdBufferSize = VarDecl::Create(getContext(), declCTX, SourceLocation(), SourceLocation(), iiBS, getContext().getVolatileType(forCondVar->getType()), nullptr, StorageClass::SC_None);

				Expr *chunk;
				ImplicitCastExpr *iceChunkVar;

				DeclRefExpr *dreInitBS = DeclRefExpr::Create (getContext(), *nnsl , SourceLocation(), vdBufferSize, false, *dniBS, vdBufferSize->getType(), ExprValueKind::VK_RValue);
				if(C->getChunkSize()) {
					chunk = const_cast<Expr *>(C->getChunkSize());
					//chunk->setType(forCondVar->getType());
					if(chunk->getType() != forCondVar->getType())
						chunk = ImplicitCastExpr::Create(getContext(), forCondVar->getType(), CastKind::CK_IntegralCast, chunk, nullptr, ExprValueKind::VK_RValue);
					//chunk = CStyleCastExpr::Create(getContext(), forCondVar->getType(), ExprValueKind::VK_RValue, CastKind::CK_IntegralCast, chunk, nullptr, nullptr, SourceLocation(), SourceLocation());
				}
				else chunk = IntegerLiteral::Create(getContext(), llvm::APInt(getContext().getTypeSize(forCondVar->getType()), (uint64_t) 1), forCondVar->getType(), SourceLocation());
				//DeclRefExpr *dreInitBS2 = DeclRefExpr::Create (getContext(), *nnsl , SourceLocation(), *var, false, *dniVar2, (*var)->getType(), ExprValueKind::VK_RValue);
				//ImplicitCastExpr *iceBS = ImplicitCastExpr::Create(getContext(), (*var)->getType(), CastKind::CK_LValueToRValue, dreInitVar2, nullptr, ExprValueKind::VK_RValue);
				//chunk->dumpColor();
				/*if(chunk->getType() != forCondVar->getType()) {
					CStyleCastExpr *castE = CStyleCastExpr::Create(getContext(), forCondVar->getType(), clang::ExprValueKind::VK_RValue, clang::CastKind::CK_IntegralCast, chunk, nullptr, nullptr, SourceLocation(), SourceLocation());
					chunk = castE;
				}*/
				BinaryOperator *boInitBS = new (getContext()) BinaryOperator(dreInitBS, chunk, BinaryOperatorKind::BO_Assign, forCondVar->getType(), clang::ExprValueKind::VK_RValue, ExprObjectKind::OK_Ordinary, SourceLocation(), false);
				//boInitBS->dumpColor();
				//EmitBinaryOperatorLValue(boInitBS);
				LValue lv = EmitLValue(dreInitBS);
				RValue rv = EmitAnyExpr(chunk);
				EmitStoreThroughLValue(rv, lv);

				for(int i=0;i<num_components;i++, cTy++) {
					Expr *lhs4 = DeclRefExpr::Create(getContext(), NestedNameSpecifierLoc(), SourceLocation(), bdx_flags[i], false, *dniFlags, qt, clang::ExprValueKind::VK_RValue, dyn_cast<NamedDecl>(vdPost->getCanonicalDecl()));
					ImplicitCastExpr *iceArray2 = ImplicitCastExpr::Create(getContext(), qt, CastKind::CK_LValueToRValue, lhs4, nullptr, ExprValueKind::VK_RValue);

					BinaryOperator *boAssign2 = new (getContext()) BinaryOperator(iceArray2, IntegerLiteral::Create(getContext(), llvm::APInt(getContext().getTypeSize(getContext().IntTy), (uint64_t) 0), getContext().IntTy, SourceLocation()) , BinaryOperatorKind::BO_Assign, getContext().IntTy, clang::ExprValueKind::VK_RValue, ExprObjectKind::OK_Ordinary, SourceLocation(), false);
					EmitBinaryOperatorLValue(boAssign2);
				}


                IdentifierInfo* iiforCondVar = &(idt->get(forCondVar->getNameAsString()));
                DeclarationName *dnforCondVar = new DeclarationName(iiforCondVar);
                DeclarationNameInfo *dniforCondVar = new DeclarationNameInfo(*dnforCondVar, SourceLocation());

                Expr *dreCondVar = DeclRefExpr::Create(getContext(), NestedNameSpecifierLoc(), SourceLocation(), forCondVar, false, *dniforCondVar, forCondVar->getType(), clang::ExprValueKind::VK_RValue);
                ImplicitCastExpr *iceCondVar = ImplicitCastExpr::Create(getContext(), forCondVar->getType(), CastKind::CK_LValueToRValue, dreCondVar, nullptr, ExprValueKind::VK_RValue);

                std::vector<Stmt *> InitArray;
                std::vector<VarDecl *> tempVars;

/*                FunctionArgList ArgsInit;
                ImplicitParamDecl DummyPtr(getContext(), DC=nullptr, SourceLocation(), Id=nullptr, forCondVar->getType());
                ArgsInit.push_back(&DummyPtr);

                for(auto comp = blocosVars.begin(); comp != blocosVars.end(); comp++) {
                    for(auto var = (*comp).begin(); var != (*comp).end(); var++) {
                        ImplicitParamDecl DummyPtr2(getContext(), DC=nullptr, SourceLocation(), Id=nullptr, (*var)->getType());
                        ArgsInit.push_back(&DummyPtr2);
                    }
                }

                const CGFunctionInfo &FuncInfoInit = CGM.getTypes().arrangeBuiltinFunctionDeclaration(getContext().IntTy, ArgsInit);
                llvm::FunctionType *FuncLLVMTyInit = CGM.getTypes().GetFunctionType(FuncInfoInit);
                llvm::Function *F = llvm::Function::Create(FuncLLVMTyInit, llvm::GlobalValue::InternalLinkage, "bdx_init_var", &CGM.getModule());

                CGF.StartFunction(GlobalDecl(), forCondVar->getType(), F, FuncInfoInit, ArgsInit);

                auto arg = ArgsInit.begin();
                VarDecl *retVar = const_cast<VarDecl *>(*arg);
                arg++;
                for(auto comp = blocosVars.begin(); comp != blocosVars.end(); comp++) {
                    for (auto var = (*comp).begin(); var != (*comp).end(); var++) {
                        //LValue DummyPtrParam = CGF.EmitLoadOfPointerLValue(CGF.GetAddrOfLocalVar(*arg), (getContext().VoidPtrTy).castAs<PointerType>());
                        VarDecl *tmpVar = const_cast<VarDecl *>(*arg);

                        IdentifierInfo *iiVar = &(idt->get("__bdx_loop_carried_" + (*var)->getNameAsString()));
                        //DeclarationName *dnVar = new DeclarationName(iiVar);
                        //DeclarationNameInfo *dniVar = new DeclarationNameInfo(*dnVar, SourceLocation());

                        //DeclarationName dnVar2 = tmpVar->getDeclName();
                        //DeclarationNameInfo *dniVar2 = new DeclarationNameInfo(tmpVar->getDeclName(), SourceLocation());

                        VarDecl *bdxVar = VarDecl::Create(getContext(), declCTX, SourceLocation(), SourceLocation(), iiVar, getContext().getVolatileType((*var)->getType()), nullptr, StorageClass::SC_None);
                        tempVars.push_back(bdxVar);

                        llvm::Value *loadVar = CGF.EmitLoadOfScalar(CGF.GetAddrOfLocalVar (tmpVar), false, tmpVar->getType(), SourceLocation());
                        //Address loadVar = CGF.GetAddrOfLocalVar(tmpVar);
                        llvm::Value *loadGlobal = CGM.GetAddrOfGlobalVar(bdxVar);
                        //llvm::Value *loadGlobal = CGM.GetAddrOfGlobal(GlobalDecl(bdxVar));
                        //llvm::errs() << *(loadVar) << "\n";
                        CGF.EmitStoreOfScalar(loadVar, Address(loadGlobal, CharUnits()), true, bdxVar->getType());
                        //EmitStoreOfScalar(load, Address(loadGlobal, CharUnits()), true, bdxVar->getType());

                        //DeclRefExpr *dreVar = DeclRefExpr::Create (getContext(), *nnsl , SourceLocation(), bdxVar, false, *dniVar, getContext().getVolatileType(bdxVar->getType()), ExprValueKind::VK_RValue);
                        //ImplicitCastExpr *iceVar = ImplicitCastExpr::Create(getContext(), getContext().getPointerType(bdxVar->getType()), CastKind::CK_LValueToRValue, dreVar, nullptr, ExprValueKind::VK_RValue);
                        //DeclRefExpr *dreVar2 = DeclRefExpr::Create (getContext(), *nnsl , SourceLocation(), tmpVar, false, *dniVar2, tmpVar->getType(), ExprValueKind::VK_RValue);
                        //ImplicitCastExpr *iceVar2 = ImplicitCastExpr::Create(getContext(), getContext().getPointerType(tmpVar->getType()), CastKind::CK_LValueToRValue, dreVar2, nullptr, ExprValueKind::VK_RValue);
                        //BinaryOperator *boforVar = new (getContext()) BinaryOperator(dreVar, IntegerLiteral::Create(getContext(), llvm::APInt(getContext().getTypeSize(getContext().IntTy), (uint64_t) 5), getContext().IntTy, SourceLocation()), BinaryOperatorKind::BO_Assign, getContext().IntTy, clang::ExprValueKind::VK_RValue, ExprObjectKind::OK_Ordinary, SourceLocation(), false);
                        //EmitBinaryOperatorLValue(boforVar);
                        //InitArray.push_back(boforVar);
                        arg++;
                    }
                }
                llvm::IRBuilder<> Builder(CGM.getLLVMContext());
                llvm::Value *retVal = CGF.EmitLoadOfScalar(CGF.GetAddrOfLocalVar(retVar), false, retVar->getType(), SourceLocation());
                //ReturnStmt *rsInitVars = new (getContext()) ReturnStmt(SourceLocation(), forCondVal, nullptr);
                //InitArray.push_back(rsInitVars);

                Builder.CreateRet(retVal);
                //CompoundStmt *InitStmt = new (getContext()) CompoundStmt(getContext(), InitArray, SourceLocation(), SourceLocation());
                //CGF.EmitCompoundStmt(*InitStmt);
                //CGF.EmitReturnStmt(*rsInitVars);
                CGF.FinishFunction();*/

				Expr *lhs, *rhs, *lhs2, *rhs2, *lhs3, *rhs3;
				int i_component = 0;
                int i_component_serial = 0;
				// Iterate over the components
                auto itTy = blocosTy.begin();
				for(auto it = blocos.begin();it != blocos.end();it++, itTy++) {


					IdentifierInfo* iiVar = &(idt->get("bdx_inner_iterator"));
					VarDecl *condVar = VarDecl::Create(getContext(), declCTX, SourceLocation(), SourceLocation(), iiVar, getContext().getVolatileType(forCondVar->getType()), nullptr, StorageClass::SC_None);

					Expr *dreBeginVar = DeclRefExpr::Create(getContext(), NestedNameSpecifierLoc(), SourceLocation(), condVar, false, *dniforCondVar, getContext().getVolatileType(condVar->getType()), clang::ExprValueKind::VK_LValue);
					ImplicitCastExpr *iceBeginVar = ImplicitCastExpr::Create(getContext(), getContext().getVolatileType(condVar->getType()), CastKind::CK_LValueToRValue, dreBeginVar, nullptr, ExprValueKind::VK_RValue);

					ArgsCond.clear();
					ArgsCond.push_back(iceBeginVar);

					// Create the 'CallExpr' nodes for inserting on the AST
					DeclRefExpr *dreBegin = DeclRefExpr::Create (getContext(), *nnsl , SourceLocation(), FDBegin, false, *dniBegin, funcTypeBegin, ExprValueKind::VK_RValue);
					ImplicitCastExpr *iceBegin = ImplicitCastExpr::Create(getContext(), getContext().getPointerType(funcTypeBegin), CastKind::CK_FunctionToPointerDecay, dreBegin, nullptr, ExprValueKind::VK_RValue);
					clang::CallExpr *ceBegin = new (getContext()) CallExpr(getContext(), iceBegin, ArgsCond, getContext().VoidTy, clang::ExprValueKind::VK_RValue, SourceLocation());

					DeclRefExpr *dreEnd = DeclRefExpr::Create (getContext(), *nnsl , SourceLocation(), FDEnd, false, *dniEnd, funcType, ExprValueKind::VK_RValue);
					ImplicitCastExpr *iceEnd = ImplicitCastExpr::Create(getContext(), getContext().getPointerType(funcType), CastKind::CK_FunctionToPointerDecay, dreEnd, nullptr, ExprValueKind::VK_RValue);
					clang::CallExpr *ceEnd = new (getContext()) CallExpr(getContext(), iceEnd, None, getContext().VoidTy, clang::ExprValueKind::VK_RValue, SourceLocation());

					std::vector<Stmt *> forStmts;
					forStmts.push_back(ceBegin);
					for(auto it2 = (*it).begin(); it2 != (*it).end(); it2++) forStmts.push_back(*it2);
					forStmts.push_back(ceEnd);

					Stmt *forCompS = new (getContext()) CompoundStmt(getContext(), forStmts, SourceLocation(), SourceLocation());



					condVar->setInit(iceCondVar);

					DeclStmt *declStmt = new (getContext()) DeclStmt(DeclGroupRef(condVar), SourceLocation(), SourceLocation());

					Expr *dreCondVar2 = DeclRefExpr::Create(getContext(), NestedNameSpecifierLoc(), SourceLocation(), condVar, false, *dniforCondVar, getContext().getVolatileType(condVar->getType()), clang::ExprValueKind::VK_RValue);
					Expr *dreCondVar3 = DeclRefExpr::Create(getContext(), NestedNameSpecifierLoc(), SourceLocation(), forCondVar, false, *dniforCondVar, forCondVar->getType(), clang::ExprValueKind::VK_RValue);
					//BinaryOperator *boforCondAdd = new (getContext()) BinaryOperator(dreCondVar3, IntegerLiteral::Create(getContext(), llvm::APInt(getContext().getTypeSize(getContext().IntTy), (uint64_t) 5), getContext().IntTy, SourceLocation()), BinaryOperatorKind::BO_Add, getContext().IntTy, clang::ExprValueKind::VK_RValue, ExprObjectKind::OK_Ordinary, SourceLocation(), false);
					Expr *cz;
					if(C->getChunkSize()) {
						cz = const_cast<Expr *>(C->getChunkSize());
						//cz->setType(forCondVar->getType());
						if(cz->getType() != forCondVar->getType())
							cz = ImplicitCastExpr::Create(getContext(), forCondVar->getType(), CastKind::CK_IntegralCast, cz, nullptr, ExprValueKind::VK_RValue);
					}
					else cz = IntegerLiteral::Create(getContext(), llvm::APInt(getContext().getTypeSize(forCondVar->getType()), (uint64_t) 1), forCondVar->getType(), SourceLocation());
					BinaryOperator *boforCondAdd = new (getContext()) BinaryOperator(dreCondVar3, cz, BinaryOperatorKind::BO_Add, forCondVar->getType(), clang::ExprValueKind::VK_RValue, ExprObjectKind::OK_Ordinary, SourceLocation(), false);
					BinaryOperator *boforCond = new (getContext()) BinaryOperator(dreCondVar2, boforCondAdd, BinaryOperatorKind::BO_LT, forCondVar->getType(), clang::ExprValueKind::VK_RValue, ExprObjectKind::OK_Ordinary, SourceLocation(), false);

					Expr *dreCondVar4 = DeclRefExpr::Create(getContext(), NestedNameSpecifierLoc(), SourceLocation(), condVar, false, *dniforCondVar, getContext().getVolatileType(forCondVar->getType()), clang::ExprValueKind::VK_RValue);
					UnaryOperator *uoforInc = new (getContext()) UnaryOperator(dreCondVar4, UnaryOperatorKind::UO_PostInc, forCondVar->getType(), clang::ExprValueKind::VK_RValue, ExprObjectKind::OK_Ordinary, SourceLocation());

					ForStmt *forStmt = new (getContext()) ForStmt (getContext(), declStmt, boforCond, condVar, uoforInc, forCompS, SourceLocation(), SourceLocation(), SourceLocation());



					ReplaceVarDecl(forCompS, forCondVar, condVar);

	
					// Build __bdx_flags[0]
					/*lhs = DeclRefExpr::Create(getContext(), NestedNameSpecifierLoc(), SourceLocation(), vdPost, false, *dniFlags, qt, clang::ExprValueKind::VK_RValue, dyn_cast<NamedDecl>(vdPost->getCanonicalDecl()));
					rhs = IntegerLiteral::Create(getContext(), llvm::APInt(getContext().getTypeSize(getContext().IntTy), (uint64_t) i_component), getContext().IntTy, SourceLocation());
					ImplicitCastExpr *iceFlags = ImplicitCastExpr::Create(getContext(), getContext().getPointerType(qt), CastKind::CK_ArrayToPointerDecay, lhs, nullptr, ExprValueKind::VK_RValue);
					ArraySubscriptExpr *asexpr = new (getContext()) ArraySubscriptExpr(iceFlags, rhs, getContext().IntTy, clang::ExprValueKind::VK_RValue, clang::ExprObjectKind::OK_Ordinary, SourceLocation());*/

					lhs = DeclRefExpr::Create(getContext(), NestedNameSpecifierLoc(), SourceLocation(), bdx_flags[i_component], false, *dniFlags, qt, clang::ExprValueKind::VK_RValue, dyn_cast<NamedDecl>(vdPost->getCanonicalDecl()));

					// Build another __bdx_flags[0]
/*					lhs2 = DeclRefExpr::Create(getContext(), NestedNameSpecifierLoc(), SourceLocation(), vdPost, false, *dniFlags, qt, clang::ExprValueKind::VK_RValue, dyn_cast<NamedDecl>(vdPost->getCanonicalDecl()));
					rhs2 = IntegerLiteral::Create(getContext(), llvm::APInt(getContext().getTypeSize(getContext().IntTy), (uint64_t) i_component), getContext().IntTy, SourceLocation());
					ImplicitCastExpr *iceFlags2 = ImplicitCastExpr::Create(getContext(), getContext().getPointerType(qt), CastKind::CK_ArrayToPointerDecay, lhs2, nullptr, ExprValueKind::VK_RValue);
					ArraySubscriptExpr *asexpr2 = new (getContext()) ArraySubscriptExpr(iceFlags2, rhs2, getContext().IntTy, clang::ExprValueKind::VK_RValue, clang::ExprObjectKind::OK_Ordinary, SourceLocation());*/

					lhs2 = DeclRefExpr::Create(getContext(), NestedNameSpecifierLoc(), SourceLocation(), bdx_flags[i_component], false, *dniFlags, qt, clang::ExprValueKind::VK_RValue, dyn_cast<NamedDecl>(vdPost->getCanonicalDecl()));

					// Build __bdx_flags[0] + 1
					BinaryOperator *boAdd = new (getContext()) BinaryOperator(lhs2, IntegerLiteral::Create(getContext(), llvm::APInt(getContext().getTypeSize(getContext().IntTy), (uint64_t) 1), getContext().IntTy, SourceLocation()), BinaryOperatorKind::BO_Add, getContext().IntTy, clang::ExprValueKind::VK_RValue, ExprObjectKind::OK_Ordinary, SourceLocation(), false);
					ParenExpr *peAdd = new (getContext()) ParenExpr(SourceLocation(), SourceLocation(), boAdd);

					DeclRefExpr *dreNumThreads0 = DeclRefExpr::Create (getContext(), *nnsl , SourceLocation(), FDNumThreads0, false, *dniNumThreads0, funcTypeNumThreads, ExprValueKind::VK_RValue);
					ImplicitCastExpr *iceNumThreads0 = ImplicitCastExpr::Create(getContext(), getContext().getPointerType(funcTypeNumThreads), CastKind::CK_FunctionToPointerDecay, dreNumThreads0, nullptr, ExprValueKind::VK_RValue);
					clang::CallExpr *ceNumThreads0 = new (getContext()) CallExpr(getContext(), iceNumThreads0, None, getContext().IntTy, clang::ExprValueKind::VK_RValue, SourceLocation());

					// Build (__bdx_flags + 1) % num_components
					//BinaryOperator *boMod = new (getContext()) BinaryOperator(peAdd, IntegerLiteral::Create(getContext(), llvm::APInt(getContext().getTypeSize(getContext().IntTy), (uint64_t) num_components), getContext().IntTy, SourceLocation()), BinaryOperatorKind::BO_Rem, getContext().IntTy, clang::ExprValueKind::VK_RValue, ExprObjectKind::OK_Ordinary, SourceLocation(), false);
					BinaryOperator *boMod = new (getContext()) BinaryOperator(peAdd, ceNumThreads0, BinaryOperatorKind::BO_Rem, getContext().IntTy, clang::ExprValueKind::VK_RValue, ExprObjectKind::OK_Ordinary, SourceLocation(), false);

					// Build __bdx_flags[0] = (__bdx_flags[0] + 1) % num_components
					BinaryOperator *boAssign = new (getContext()) BinaryOperator(lhs, boMod, BinaryOperatorKind::BO_Assign, getContext().IntTy, clang::ExprValueKind::VK_RValue, ExprObjectKind::OK_Ordinary, SourceLocation(), false);

					// Build another __bdx_flags[0]
/*					lhs3 = DeclRefExpr::Create(getContext(), NestedNameSpecifierLoc(), SourceLocation(), vdPost, false, *dniFlags, qt, clang::ExprValueKind::VK_RValue, dyn_cast<NamedDecl>(vdPost->getCanonicalDecl()));
					rhs3 = IntegerLiteral::Create(getContext(), llvm::APInt(getContext().getTypeSize(getContext().IntTy), (uint64_t) i_component), getContext().IntTy, SourceLocation());
					ImplicitCastExpr *iceFlags3 = ImplicitCastExpr::Create(getContext(), getContext().getPointerType(qt), CastKind::CK_ArrayToPointerDecay, lhs3, nullptr, ExprValueKind::VK_RValue);
					ArraySubscriptExpr *asexpr3 = new (getContext()) ArraySubscriptExpr(iceFlags3, rhs3, getContext().IntTy, clang::ExprValueKind::VK_RValue, clang::ExprObjectKind::OK_Ordinary, SourceLocation());
					ImplicitCastExpr *iceArray = ImplicitCastExpr::Create(getContext(), getContext().IntTy, CastKind::CK_LValueToRValue, asexpr3, nullptr, ExprValueKind::VK_RValue);*/

					lhs3 = DeclRefExpr::Create(getContext(), NestedNameSpecifierLoc(), SourceLocation(), bdx_flags[i_component], false, *dniFlags, qt, clang::ExprValueKind::VK_RValue, dyn_cast<NamedDecl>(vdPost->getCanonicalDecl()));
					ImplicitCastExpr *iceArray = ImplicitCastExpr::Create(getContext(), qt, CastKind::CK_LValueToRValue, lhs3, nullptr, ExprValueKind::VK_RValue);

					// Build an 'omp_get_thread_num' call node
					DeclRefExpr *dreNumThreads = DeclRefExpr::Create (getContext(), *nnsl , SourceLocation(), FDNumThreads, false, *dniNumThreads, funcTypeNumThreads, ExprValueKind::VK_RValue);
					ImplicitCastExpr *iceNumThreads = ImplicitCastExpr::Create(getContext(), getContext().getPointerType(funcTypeNumThreads), CastKind::CK_FunctionToPointerDecay, dreNumThreads, nullptr, ExprValueKind::VK_RValue);
					clang::CallExpr *ceNumThreads = new (getContext()) CallExpr(getContext(), iceNumThreads, None, getContext().IntTy, clang::ExprValueKind::VK_RValue, SourceLocation());

					// Build __bdx_flags[0] != omp_get_thread_num()
					BinaryOperator *boNE = new (getContext()) BinaryOperator(iceArray, ceNumThreads, BinaryOperatorKind::BO_NE, getContext().IntTy, clang::ExprValueKind::VK_RValue, ExprObjectKind::OK_Ordinary, SourceLocation(), false);

					// Build an empty body
					Stmt *nullBody = new (getContext()) NullStmt(SourceLocation());

					// Build while(__bdx_flags[0] != omp_get_thread_num());
					WhileStmt *whileStmt = new (getContext()) WhileStmt(getContext(), nullptr, boNE, nullBody, SourceLocation());

					(*it).clear();

                    if(C->getUseKind() == OMPC_USE_bdx && (*itTy) == true) (*it).push_back(whileStmt);

                    if((*itTy) == false) {
                        (*it).push_back(whileStmt);
                        //int iVar = 0;
						if(blocosVars.size() > 0) {
		                    for(auto var = (blocosVars[i_component_serial]).begin(); var != (blocosVars[i_component_serial]).end(); var++) {
		                        IdentifierInfo *iiVar = &(idt->get("__bdx_loop_carried_" + (*var)->getNameAsString()));
		                        DeclarationName *dnVar = new DeclarationName(iiVar);
		                        DeclarationNameInfo *dniVar = new DeclarationNameInfo(*dnVar, SourceLocation());

		                        IdentifierInfo *iiVar2 = &(idt->get((*var)->getNameAsString()));
		                        DeclarationName *dnVar2 = new DeclarationName(iiVar2);
		                        DeclarationNameInfo *dniVar2 = new DeclarationNameInfo(*dnVar2, SourceLocation());

		                        VarDecl *bdxVar = VarDecl::Create(getContext(), declCTX, SourceLocation(), SourceLocation(), iiVar, getContext().getVolatileType((*var)->getType()), nullptr, StorageClass::SC_None);
		                        //bdxVar->setInit((*var)->getInit());
		                        tempVars.push_back(bdxVar);
		                        //VarDecl *bdxVar = tempVars[iVar++];

		                        DeclRefExpr *dreVar = DeclRefExpr::Create (getContext(), *nnsl , SourceLocation(), bdxVar, false, *dniVar, getContext().getVolatileType(bdxVar->getType()), ExprValueKind::VK_RValue);
		                        DeclRefExpr *dreVar2 = DeclRefExpr::Create (getContext(), *nnsl , SourceLocation(), *var, false, *dniVar2, (*var)->getType(), ExprValueKind::VK_RValue);
		                        BinaryOperator *boforVar = new (getContext()) BinaryOperator(dreVar2, dreVar, BinaryOperatorKind::BO_Assign, (*var)->getType(), clang::ExprValueKind::VK_RValue, ExprObjectKind::OK_Ordinary, SourceLocation(), false);
		                        (*it).push_back(boforVar);
		                    }
						}
                    }
                    

                    
					(*it).push_back(forStmt);
                    if(C->getUseKind() == OMPC_USE_bdx && (*itTy) == true) (*it).push_back(boAssign);
                    if((*itTy) == false) {
                        int iVar = 0;
						if(blocosVars.size() > 0) {
		                    for(auto var = (blocosVars[i_component_serial]).begin(); var != (blocosVars[i_component_serial]).end(); var++) {

		                        IdentifierInfo *iiVar = &(idt->get("__bdx_loop_carried_" + (*var)->getNameAsString()));

		                        DeclarationName *dnVar = new DeclarationName(iiVar);
		                        DeclarationNameInfo *dniVar = new DeclarationNameInfo(*dnVar, SourceLocation());

		                        IdentifierInfo *iiVar2 = &(idt->get((*var)->getNameAsString()));
		                        DeclarationName *dnVar2 = new DeclarationName(iiVar2);
		                        DeclarationNameInfo *dniVar2 = new DeclarationNameInfo(*dnVar2, SourceLocation());

		                        VarDecl *bdxVar = tempVars[iVar++];

		                        DeclRefExpr *dreVar = DeclRefExpr::Create (getContext(), *nnsl , SourceLocation(), bdxVar, false, *dniVar, getContext().getVolatileType(getContext().IntTy), ExprValueKind::VK_RValue);
		                        DeclRefExpr *dreVar2 = DeclRefExpr::Create (getContext(), *nnsl , SourceLocation(), *var, false, *dniVar2, getContext().IntTy, ExprValueKind::VK_RValue);
		                        BinaryOperator *boforVar = new (getContext()) BinaryOperator(dreVar, dreVar2, BinaryOperatorKind::BO_Assign, getContext().IntTy, clang::ExprValueKind::VK_RValue, ExprObjectKind::OK_Ordinary, SourceLocation(), false);
		                        (*it).push_back(boforVar);
		                        //llvm::Value *loadVar = EmitLoadOfScalar(GetAddrOfLocalVar (*var), false, (*var)->getType(), SourceLocation());
		                        //llvm::Value *loadGlobal = CGM.GetAddrOfGlobalVar(bdxVar);
		                        //EmitStoreOfScalar(loadVar, Address(loadGlobal, CharUnits()), true, bdxVar->getType());

		                        DeclRefExpr *dreInitVar = DeclRefExpr::Create (getContext(), *nnsl , SourceLocation(), bdxVar, false, *dniVar, getContext().getVolatileType(bdxVar->getType()), ExprValueKind::VK_RValue);
		                        //ImplicitCastExpr *iceVar = ImplicitCastExpr::Create(getContext(), getContext().getPointerType(bdxVar->getType()), CastKind::CK_LValueToRValue, dreVar, nullptr, ExprValueKind::VK_RValue);
		                        DeclRefExpr *dreInitVar2 = DeclRefExpr::Create (getContext(), *nnsl , SourceLocation(), *var, false, *dniVar2, (*var)->getType(), ExprValueKind::VK_RValue);
		                        ImplicitCastExpr *iceVar2 = ImplicitCastExpr::Create(getContext(), (*var)->getType(), CastKind::CK_LValueToRValue, dreInitVar2, nullptr, ExprValueKind::VK_RValue);


		                        BinaryOperator *boInitVar = new (getContext()) BinaryOperator(dreInitVar, iceVar2, BinaryOperatorKind::BO_Assign, getContext().IntTy, clang::ExprValueKind::VK_RValue, ExprObjectKind::OK_Ordinary, SourceLocation(), false);

		                        
		                        EmitBinaryOperatorLValue(boInitVar);

		                        
		                    }
						}
                        i_component_serial++;
                        (*it).push_back(boAssign);
                    }
                    tempVars.clear();
					i_component++;
				}

				CompoundStmt *CompS;
				itTy = blocosTy.begin();
				IntegerLiteral *stageFlagArg;

				// Iterate over the new components, with the already inserted function call for begin and end
				for(auto it = blocos.begin();it != blocos.end();it++, itTy++) {
					ArgsCond.clear();

					if((*itTy) == true)
						stageFlagArg = IntegerLiteral::Create(getContext(), llvm::APInt(getContext().getTypeSize(getContext().IntTy), (uint64_t) 1), getContext().IntTy, SourceLocation());
					else
						stageFlagArg = IntegerLiteral::Create(getContext(), llvm::APInt(getContext().getTypeSize(getContext().IntTy), (uint64_t) 0), getContext().IntTy, SourceLocation());
					ArgsCond.push_back(stageFlagArg);

					// Create the 'CallExpr' node for the IF condition
					DeclRefExpr *dreCond = DeclRefExpr::Create (getContext(), *nnsl , SourceLocation(), FDCond, false, *dniCond, funcTypeCond, ExprValueKind::VK_RValue);
					ImplicitCastExpr *iceCond = ImplicitCastExpr::Create(getContext(), getContext().getPointerType(funcTypeCond), CastKind::CK_FunctionToPointerDecay, dreCond, nullptr, ExprValueKind::VK_RValue);
					clang::CallExpr *ceCond = new (getContext()) CallExpr(getContext(), iceCond, ArgsCond, getContext().IntTy, clang::ExprValueKind::VK_RValue, SourceLocation());

					// Create a new 'CompoundStmt' containing the component statements
					CompS = new (getContext()) CompoundStmt(getContext(), *it, SourceLocation(), SourceLocation());
					Stmt *st = CompS;
					// Create a new IF and use the recently created 'CompoundStmt' as 'then' condition
					IfStmt *ifs = new (getContext()) IfStmt(getContext(), SourceLocation(), false, nullptr, nullptr, ceCond, st);
					// Push each IF into a vector
					ifStmts.push_back(ifs);
				}

				// Create the 'CompoundStmt' containing all created IFs
				CompS = new (getContext()) CompoundStmt(getContext(), ifStmts, SourceLocation(), SourceLocation());
				Stmt *st = CompS;

				// Set the 'CompoundStmt' as the For body in the AST
				dyn_cast<ForStmt>(dyn_cast<CapturedStmt>(S.getAssociatedStmt())->getCapturedStmt())->setBody(st);

				// Create a new 'OMPNumThreadsClause' node to insert the 'num_threads' clause in the pragma
				IntegerLiteral *numThreads = IntegerLiteral::Create(getContext(), llvm::APInt(getContext().getTypeSize(getContext().IntTy), (uint64_t) blocos.size()), getContext().IntTy, SourceLocation());
			 	OMPNumThreadsClause *numThreadsClause =  new (getContext()) OMPNumThreadsClause(numThreads, S.getLocStart(), S.getLocEnd(), S.getLocEnd());

				IntegerLiteral *chunkSize = IntegerLiteral::Create(getContext(), llvm::APInt(getContext().getTypeSize(getContext().IntTy), (uint64_t) 1), getContext().IntTy, SourceLocation());
				OMPScheduleClause *scheduleClause = new (getContext()) OMPScheduleClause(S.getLocStart(), S.getLocEnd(), S.getLocEnd(), S.getLocEnd(), S.getLocEnd(), OpenMPScheduleClauseKind::OMPC_SCHEDULE_static, chunkSize, nullptr, OpenMPScheduleClauseModifier::OMPC_SCHEDULE_MODIFIER_unknown, SourceLocation(), OpenMPScheduleClauseModifier::OMPC_SCHEDULE_MODIFIER_unknown, SourceLocation());

//				Expr *name = DeclRefExpr::Create(getContext(), NestedNameSpecifierLoc(), SourceLocation(), vdPost, false, *dniFlags, qt, clang::ExprValueKind::VK_RValue, cast<NamedDecl>(vdPost->getCanonicalDecl()));
//				std::vector<Expr *> shareds;
//				shareds.push_back(name);
//				OMPSharedClause *sharedClause = OMPSharedClause::Create(getContext(), S.getLocStart(), S.getLocEnd(), S.getLocEnd(), shareds);

                std::vector<OMPClause *> clauses = S.clauses().vec();

                OMPPrivateClause *P = nullptr;
                for(auto cl = clauses.begin(); cl != clauses.end(); cl++) {
                  if(isa<OMPPrivateClause>(*cl)) {
                    P = dyn_cast<OMPPrivateClause>(*cl);
                    break;
                  }
                }

                std::vector<Expr *> privates;
                if(P != nullptr) {
                  clang::OMPVarListClause<OMPPrivateClause> *Pvars = dyn_cast<clang::OMPVarListClause<OMPPrivateClause> >(P);

                  for (auto vr = Pvars->varlist_begin(); vr != Pvars->varlist_end(); vr++) {
                    DeclRefExpr *dre = dyn_cast<DeclRefExpr>(*vr);
                    //VarDecl *vd = cast<VarDecl>(dre->getDecl());
                    privates.push_back(dre);
                  }
                }

				for(auto comp = blocosVars.begin(); comp != blocosVars.end(); comp++) {
                    for(auto var = (*comp).begin(); var != (*comp).end(); var++) {
                        //DeclarationName dnPvtVar = (*var)->getDeclName();
                        DeclarationNameInfo *dniPvtVar = new DeclarationNameInfo((*var)->getDeclName(), SourceLocation());

                        Expr *name = DeclRefExpr::Create(getContext(), NestedNameSpecifierLoc(), SourceLocation(), *var, false, *dniPvtVar, (*var)->getType(), clang::ExprValueKind::VK_RValue, dyn_cast<NamedDecl>(vdPost->getCanonicalDecl()));
                        privates.push_back(name);
                    }
                }

				OMPPrivateClause *privateClause = OMPPrivateClause::Create(getContext(), S.getLocStart(), S.getLocEnd(), S.getLocEnd(), privates, privates);

				// Get all original clauses from pragma


				if(C->getUseKind() == OMPC_USE_bdx) {
					for(auto cl = clauses.begin(); cl != clauses.end(); cl++) {
						if(isa<OMPNumThreadsClause>(*cl)) {
							clauses.erase(cl);
							break;
						}
					}
				}

				for(auto cl = clauses.begin(); cl != clauses.end(); cl++) {
					if(isa<OMPPrivateClause>(*cl)) {
						clauses.erase(cl);
						break;
					}
				}

				for(auto cl = clauses.begin(); cl != clauses.end(); cl++) {
					if(isa<OMPOrderedClause>(*cl)) {
						clauses.erase(cl);
						break;
					}
				}
				
			for(auto cl = clauses.begin(); cl != clauses.end(); cl++) {
                  if(isa<OMPPrivateClause>(*cl)) {
                    clauses.erase(cl);                    
                    break;
                  }
                }


				// Insert 'num_threads' and 'schedule' clause
				if(C->getUseKind() == OMPC_USE_bdx) clauses.push_back(numThreadsClause);
				clauses.push_back(scheduleClause);
				clauses.push_back(privateClause);

				OMPLoopDirective::HelperExprs B;

				// Copy the struct 'HelperExprs', which is responsible for controlling loop bounds

				B.IterationVarRef = S.getIterationVariable();
				B.LastIteration = S.getLastIteration();
				B.CalcLastIteration = S.getCalcLastIteration();
				B.PreCond = S.getPreCond();
				B.Cond = S.getCond();
				B.Init = S.getInit();
				B.Inc = S.getInc();
				B.IL = S.getIsLastIterVariable();
				B.LB = S.getLowerBoundVariable();
				B.UB = S.getUpperBoundVariable();
				B.ST = S.getStrideVariable();
				B.EUB = S.getEnsureUpperBound();
				B.NLB = S.getNextLowerBound();
				B.NUB = S.getNextUpperBound();
				B.NumIterations = S.getNumIterations();
				B.PrevLB = S.getPrevLowerBoundVariable();
				B.PrevUB = S.getPrevUpperBoundVariable();
				for(auto it=S.counters().begin();it!=S.counters().end();it++) {(B.Counters).push_back(*it);}
				for(auto it=S.private_counters().begin();it!=S.private_counters().end();it++) {(B.PrivateCounters).push_back(*it);}
				for(auto it=S.inits().begin();it!=S.inits().end();it++) {(B.Inits).push_back(*it);}
				for(auto it=S.updates().begin();it!=S.updates().end();it++) {(B.Updates).push_back(*it);}
				for(auto it=S.finals().begin();it!=S.finals().end();it++) {(B.Finals).push_back(*it);}
				Stmt* temp = const_cast<Stmt *>(S.getPreInits());
				B.PreInits = temp;

				// Create a new 'OMPParallelForDirective' node for replacing the original pragma
				OMPParallelForDirective *newS = OMPParallelForDirective::Create(getContext(), S.getLocStart(), S.getLocEnd(), S.getCollapsedNumber(), clauses, S.getAssociatedStmt(), B, false);

				newS->dumpPretty(getContext());
				//newS->dumpColor();

				// Generate a new CodeGen structure, based on the new 'parallel for' pragma
/*				auto &&CodeGen = [newS](CodeGenFunction &CGF, PrePostActionTy &) {
			 		CGF.EmitOMPWorksharingLoop(*newS);
			 	  };*/

				  auto &&CodeGen = [newS, &HasLastprivates](CodeGenFunction &CGF,
                                          PrePostActionTy &) {
					HasLastprivates = CGF.EmitOMPWorksharingLoop(*newS);
				  };

				// Emit the OpenMP using the modified AST
				OMPLexicalScope Scope(*this, *newS, /*AsInlined=*/true);
				CGM.getOpenMPRuntime().emitInlinedDirective(*this, OMPD_for, CodeGen,
						                                    newS->hasCancel());

			  // Emit an implicit barrier at the end.
			  if (!newS->getSingleClause<OMPNowaitClause>() || HasLastprivates) {
				CGM.getOpenMPRuntime().emitBarrierCall(*this, newS->getLocStart(), OMPD_for);
			  }
			}
			else {
				OMPLexicalScope Scope(*this, S, /*AsInlined=*/true);
				CGM.getOpenMPRuntime().emitInlinedDirective(*this, OMPD_for, CodeGen,
						                                    S.hasCancel());

			  // Emit an implicit barrier at the end.
			  if (!S.getSingleClause<OMPNowaitClause>() || HasLastprivates) {
				CGM.getOpenMPRuntime().emitBarrierCall(*this, S.getLocStart(), OMPD_for);
			  }
			}
		}
		else if(C->getUseKind() == OMPC_USE_doacross) {
			llvm::errs() << "doacross\n";
				OMPLexicalScope Scope(*this, S, /*AsInlined=*/true);
				CGM.getOpenMPRuntime().emitInlinedDirective(*this, OMPD_for, CodeGen,
						                                    S.hasCancel());

			  // Emit an implicit barrier at the end.
			  if (!S.getSingleClause<OMPNowaitClause>() || HasLastprivates) {
				CGM.getOpenMPRuntime().emitBarrierCall(*this, S.getLocStart(), OMPD_for);
			  }
		}
		else if(C->getUseKind() == OMPC_USE_tls){
		    //llvm::errs() << "tls\n";
		    
		    IdentifierTable *idt2 = new IdentifierTable(CGM.getLangOpts());
            //CodeGenFunction CGF(CGM);


			llvm::errs() << "tls ("<< num_loops <<")\n";

			FunctionArgList TemplateArgsCond, TemplateArgs1, TemplateArgs2,TemplateArgsCreate, TemplateArgsSetM;

			// Get the original loop body, associated with the 'parallel for' directive
			Stmt *Body = S.getAssociatedStmt()->IgnoreContainers(true);

			Body = dyn_cast<ForStmt>(Body)->getBody();
	
			// Call the function to separate the body into loop components, based on the 'ordered' directive
			visitBody2(Body);
			checkDeclRefExpr2(Body);

            S.dumpPretty(getContext());
			//S.dumpColor();
			
			int num_components = blocos.size();
			
			llvm::errs() << "numero componentes "<< num_components <<"\n";
			
			if (num_components>0){
			
			    VarDecl *vdPost;
				QualType qt = getContext().getConstantArrayType(getContext().IntTy, llvm::APInt(getContext().getTypeSize(getContext().IntTy), (uint64_t) num_components), ArrayType::ArraySizeModifier::Normal, 0);
//				NestedNameSpecifierLoc *nnsl2 = new NestedNameSpecifierLoc();
				IdentifierInfo* iiFlags = &(idt2->get("__tls_flags"));
				DeclarationName *dnFlags = new DeclarationName(iiFlags);
				DeclarationNameInfo *dniFlags = new DeclarationNameInfo(*dnFlags, SourceLocation());
				vdPost = VarDecl::Create(getContext(), declCTX, SourceLocation(), SourceLocation(), iiFlags, qt, nullptr, StorageClass::SC_None);


				// Create the identifier for using the condition function call
				IdentifierInfo* iiParam = &(idt2->get("tls_stage_param"));
//				ImplicitParamDecl stageFlag(getContext(), declCTX, SourceLocation(), nullptr, getContext().IntTy);
				ParmVarDecl *stageFlag = ParmVarDecl::Create(getContext(), declCTX, SourceLocation(), SourceLocation(), iiParam, getContext().IntTy, nullptr, StorageClass::SC_None, nullptr);
	 			TemplateArgsCond.push_back(stageFlag);
	 			
	 			
	 			IdentifierInfo* iiParam3 = &(idt2->get("cpu"));
//				ImplicitParamDecl stageFlag(getContext(), declCTX, SourceLocation(), nullptr, getContext().IntTy);
				ParmVarDecl *stageFlag3 = ParmVarDecl::Create(getContext(), declCTX, SourceLocation(), SourceLocation(), iiParam3, getContext().IntTy, 	nullptr, StorageClass::SC_None, nullptr);
	 			TemplateArgsSetM.push_back(stageFlag3);
	 			
	 			
	 			IdentifierInfo* iiParam2 = &(idt2->get("mask"));
//				ImplicitParamDecl stageFlag(getContext(), declCTX, SourceLocation(), nullptr, getContext().IntTy);
				ParmVarDecl *stageFlag2 = ParmVarDecl::Create(getContext(), declCTX, SourceLocation(), SourceLocation(), iiParam2, getContext().VoidPtrTy, nullptr, StorageClass::SC_None, nullptr);
	 			TemplateArgsCreate.push_back(stageFlag2);
	 			TemplateArgsSetM.push_back(stageFlag2);
	 			
	 			
	 			
	 			// Create the identifier for using the condition function call
				//IdentifierInfo*iiParam2 = &(idt2->get("execute_in_htm"));
//				ImplicitParamDecl stageFlag(getContext(), declCTX, SourceLocation(), nullptr, getContext().IntTy);
				//ParmVarDecl *stageFlag2 = ParmVarDecl::Create(getContext(), declCTX, SourceLocation(), SourceLocation(), iiParam2, getContext().IntTy, nullptr, StorageClass::SC_None, nullptr);
	 			//TemplateArgs2.push_back(stageFlag2);
	 			
	 			
	 			// Create the identifier for using the condition function call
				//IdentifierInfo* iiParam3 = &(idt2->get("next"));
//				ImplicitParamDecl stageFlag(getContext(), declCTX, SourceLocation(), nullptr, getContext().IntTy);
				//ParmVarDecl *stageFlag3 = ParmVarDecl::Create(getContext(), declCTX, SourceLocation(), SourceLocation(), iiParam3, getContext().IntTy, nullptr, StorageClass::SC_None, nullptr);
	 			//TemplateArgs1.push_back(stageFlag3);
	 			//TemplateArgs2.push_back(stageFlag3);
	 			
	 			// Create the identifier for using the condition function call
				IdentifierInfo* iiParam4 = &(idt2->get("par_abort"));
//				ImplicitParamDecl stageFlag(getContext(), declCTX, SourceLocation(), nullptr, getContext().IntTy);
				ParmVarDecl *stageFlag4 = ParmVarDecl::Create(getContext(), declCTX, SourceLocation(), SourceLocation(), iiParam4, getContext().CharTy, nullptr, StorageClass::SC_None, nullptr);
	 			//TemplateArgs1.push_back(stageFlag4);
	 			TemplateArgs2.push_back(stageFlag4);
	 				 			
	 			
			    // Create the function declaration.
				const CGFunctionInfo &FuncInfoCond = CGM.getTypes().arrangeBuiltinFunctionDeclaration(getContext().IntTy, TemplateArgsCond);
				llvm::FunctionType *FuncLLVMTyCond = CGM.getTypes().GetFunctionType(FuncInfoCond);
				//const CGFunctionInfo &FuncInfo1 = CGM.getTypes().arrangeBuiltinFunctionDeclaration(getContext().IntTy, TemplateArgs1);
				//llvm::FunctionType *FuncLLVMTy1 = CGM.getTypes().GetFunctionType(FuncInfo1);
				//const CGFunctionInfo &FuncInfo2 = CGM.getTypes().arrangeBuiltinFunctionDeclaration(getContext().VoidTy, TemplateArgs2);
				//llvm::FunctionType *FuncLLVMTy2 = CGM.getTypes().GetFunctionType(FuncInfo2);
				const CGFunctionInfo &FuncInfo3 = CGM.getTypes().arrangeBuiltinFunctionDeclaration(getContext().VoidTy, TemplateArgs2);
				llvm::FunctionType *FuncLLVMTy3 = CGM.getTypes().GetFunctionType(FuncInfo3);
				
				const CGFunctionInfo &FuncInfoCreate = CGM.getTypes().arrangeBuiltinFunctionDeclaration(getContext().VoidTy, TemplateArgsCreate);
				llvm::FunctionType *FuncLLVMTyCreate = CGM.getTypes().GetFunctionType(FuncInfoCreate);
				
				const CGFunctionInfo &FuncInfoSetM = CGM.getTypes().arrangeBuiltinFunctionDeclaration(getContext().VoidTy, TemplateArgsSetM);
				llvm::FunctionType *FuncLLVMTySetM = CGM.getTypes().GetFunctionType(FuncInfoSetM);								
								
				const CGFunctionInfo &FuncInfoSet = CGM.getTypes().arrangeBuiltinFunctionDeclaration(getContext().VoidTy, TemplateArgsCreate);
				llvm::FunctionType *FuncLLVMTySet = CGM.getTypes().GetFunctionType(FuncInfoSet);					
				
				
				CGM.CreateRuntimeFunction(FuncLLVMTyCond, "__tls_cond__");
				//CGM.CreateBuiltinFunction(FuncLLVMTy3, "__builtin_ia32_xabort");
				//CGM.CreateRuntimeFunction(FuncLLVMTy1, "__tls_begin__");
				//CGM.CreateRuntimeFunction(FuncLLVMTy2, "__tls_end__");
				
				
				CGM.CreateRuntimeFunction(FuncLLVMTyCreate, "kmp_create_affinity_mask");
				CGM.CreateRuntimeFunction(FuncLLVMTySetM, "kmp_set_affinity_mask_proc");
				CGM.CreateRuntimeFunction(FuncLLVMTySet, "kmp_set_affinity");

                // Create a new identifiers table based on the current context
				IdentifierTable *idt = new IdentifierTable(CGM.getLangOpts());
				NestedNameSpecifierLoc *nnsl = new NestedNameSpecifierLoc();

				// Create the identifier for using the condition function call
				IdentifierInfo* iiCond = &(idt->get("__tls_cond__"));
				DeclarationName *dnCond = new DeclarationName(iiCond);
				DeclarationNameInfo *dniCond = new DeclarationNameInfo(*dnCond, SourceLocation());

                // Create the identifier for finding the start of the component
				IdentifierInfo* iiBegin = &(idt->get("_xbegin"));
				DeclarationName *dnBegin = new DeclarationName(iiBegin);
				DeclarationNameInfo *dniBegin = new DeclarationNameInfo(*dnBegin, SourceLocation());

				// Create the identifier for finding the end of the component
				IdentifierInfo* iiEnd = &(idt->get("_xend"));
				DeclarationName *dnEnd = new DeclarationName(iiEnd);
				DeclarationNameInfo *dniEnd = new DeclarationNameInfo(*dnEnd, SourceLocation());
				
				//IdentifierInfo* iiAbort = &(idt->get("__builtin_ia32_xabort"));
				//DeclarationName *dnAbort = new DeclarationName(iiAbort);
				//DeclarationNameInfo *dniAbort = new DeclarationNameInfo(*dnAbort, SourceLocation());
				
				IdentifierInfo* iiCreate = &(idt->get("kmp_create_affinity_mask"));
				DeclarationName *dnCreate = new DeclarationName(iiCreate);
				DeclarationNameInfo *dniCreate = new DeclarationNameInfo(*dnCreate, SourceLocation());

				IdentifierInfo* iiSetM = &(idt->get("kmp_set_affinity_mask_proc"));
				DeclarationName *dnSetM = new DeclarationName(iiSetM);
				DeclarationNameInfo *dniSetM = new DeclarationNameInfo(*dnSetM, SourceLocation());
				
				IdentifierInfo* iiSet = &(idt->get("kmp_set_affinity"));
				DeclarationName *dnSet = new DeclarationName(iiSet);
				DeclarationNameInfo *dniSet = new DeclarationNameInfo(*dnSet, SourceLocation());
				

		
         		// Create the 'FunctionDecl' nodes for inserting on the AST
				std::vector<QualType> argsCond, args1,args2, argsCreate, argsSetM, argsSet;
				FunctionProtoType::ExtProtoInfo fpi;
				fpi.Variadic = false;

				argsCond.push_back(getContext().IntTy);
				//args1.push_back(getContext().IntTy);
				//args1.push_back(getContext().IntTy);
				args2.push_back(getContext().CharTy);
				//args2.push_back(getContext().IntTy);
				//args2.push_back(getContext().IntTy);
				
				argsCreate.push_back(getContext().VoidPtrTy);
				
				argsSetM.push_back(getContext().IntTy);
				argsSetM.push_back(getContext().VoidPtrTy);
				
				argsSet.push_back(getContext().VoidPtrTy);
				
				QualType funcTypeCond = getContext().getFunctionType(getContext().IntTy, argsCond, fpi);
				QualType funcType1 = getContext().getFunctionType(getContext().UnsignedIntTy,args1, fpi);
				QualType funcType2 = getContext().getFunctionType(getContext().VoidTy, args1, fpi); //xend
				QualType funcType3 = getContext().getFunctionType(getContext().VoidTy, args2, fpi); //xabort
				
				QualType funcTypeCreate = getContext().getFunctionType(getContext().VoidTy, argsCreate, fpi); 
				
				QualType funcTypeSetM = getContext().getFunctionType(getContext().VoidTy, argsSetM, fpi); 
				
				QualType funcTypeSet = getContext().getFunctionType(getContext().VoidTy, argsSet, fpi); 

				FunctionDecl *FDCond = FunctionDecl::Create(getContext(), declCTX, SourceLocation(), SourceLocation(), *dnCond, funcTypeCond, nullptr, StorageClass::SC_None);

        LinkageSpecDecl* lsd = LinkageSpecDecl::Create(getContext(),
            getContext().getTranslationUnitDecl(), SourceLocation(),
            SourceLocation(), LinkageSpecDecl::lang_c, false);

				FunctionDecl *FDBegin = FunctionDecl::Create(getContext(), lsd, SourceLocation(), SourceLocation(), *dnBegin, funcType1, nullptr, StorageClass::SC_None);
				FunctionDecl *FDEnd = FunctionDecl::Create(getContext(), lsd, SourceLocation(), SourceLocation(), *dnEnd, funcType2, nullptr, StorageClass::SC_None);
				FunctionDecl *FDAbort;
				
				IdentifierInfo* iiAbort = &(idt->get("_xabort2"));
				DeclarationName *dnAbort = new DeclarationName(iiAbort);
				DeclarationNameInfo *dniAbort = new DeclarationNameInfo(*dnAbort, SourceLocation());
				
				
				TranslationUnitDecl *TUDecl = getContext().getTranslationUnitDecl();
                DeclContext::lookup_result Lookup = TUDecl->lookup(*dnAbort);

                //assert(0 && "No __builtin_shufflevector?");
			    //FDAbort = cast<FunctionDecl>(Lookup.front());				
				
				
				FDAbort = FunctionDecl::Create(getContext(), declCTX, SourceLocation(), SourceLocation(), *dnAbort, funcType3, nullptr, StorageClass::SC_None);
				
				
				FunctionDecl *FDCreate = FunctionDecl::Create(getContext(), declCTX, SourceLocation(), SourceLocation(), *dnCreate, funcTypeCreate, nullptr, StorageClass::SC_None);
				
				FunctionDecl *FDSetM = FunctionDecl::Create(getContext(), declCTX, SourceLocation(), SourceLocation(), *dnSetM, funcTypeSetM, nullptr, StorageClass::SC_None);
			
                FunctionDecl *FDSet = FunctionDecl::Create(getContext(), declCTX, SourceLocation(), SourceLocation(), *dnSet, funcTypeSet, nullptr, StorageClass::SC_None);
                
                
                

			
				std::vector<ParmVarDecl *> params,params1,params2,paramsCreate,paramsSetM,paramsSet;
				params.push_back(stageFlag);				
				//params1.push_back(stageFlag3);
				//params1.push_back(stageFlag4);
				//params2.push_back(stageFlag2);
			    //params2.push_back(stageFlag3);
				params2.push_back(stageFlag4);
				
				paramsCreate.push_back(stageFlag2);
				
				paramsSetM.push_back(stageFlag3);
				paramsSetM.push_back(stageFlag2);
				
				paramsSet.push_back(stageFlag2);
								
				FDCond->setParams(params);
				//FDBegin->setParams(params1);
				FDAbort->setParams(params2);
				FDCreate->setParams(paramsCreate);
				FDSetM->setParams(paramsSetM);
				FDSet->setParams(paramsSet);
				
				// Auxiliary vector
				std::vector<Stmt *> ifStmts;

				std::vector<Expr *> ArgsCond, Args1 , Args2, ArgsCreate, ArgsSetM, ArgsSet;

                Stmt *forInit = dyn_cast<ForStmt>(dyn_cast<CapturedStmt>(S.getAssociatedStmt())->getCapturedStmt())->getInit();
				Expr *forCond = cast<ForStmt>(cast<CapturedStmt>(S.getAssociatedStmt())->getCapturedStmt())->getCond();
//                Expr *forInc = cast<ForStmt>(cast<CapturedStmt>(S.getAssociatedStmt())->getCapturedStmt())->getInc();
//				VarDecl *forCondVar = cast<ForStmt>(cast<CapturedStmt>(S.getAssociatedStmt())->getCapturedStmt())->getConditionVariable();

                Expr *iterations = dyn_cast<BinaryOperator>(forCond)->getRHS();
                       
                BinaryOperator *boInit = dyn_cast<BinaryOperator>(forInit);
                
                VarDecl *forCondVar = dyn_cast<VarDecl>(dyn_cast<DeclRefExpr>(boInit->getLHS())->getDecl());
                Expr *forCondVal = boInit->getRHS();

                IdentifierInfo* iiforCondVar = &(idt->get(forCondVar->getNameAsString()));
                DeclarationName *dnforCondVar = new DeclarationName(iiforCondVar);
                DeclarationNameInfo *dniforCondVar = new DeclarationNameInfo(*dnforCondVar, SourceLocation());

                Expr *dreCondVar = DeclRefExpr::Create(getContext(), NestedNameSpecifierLoc(), SourceLocation(), forCondVar, false, *dniforCondVar, forCondVar->getType(), clang::ExprValueKind::VK_RValue);
                ImplicitCastExpr *iceCondVar = ImplicitCastExpr::Create(getContext(), forCondVar->getType(), CastKind::CK_LValueToRValue, dreCondVar, nullptr, ExprValueKind::VK_RValue);

                std::vector<Stmt *> InitArray;
                std::vector<VarDecl *> tempVars;
                
                Expr *lhs, *rhs, *lhs2, *rhs2, *lhs3, *rhs3;
				int i_component = 0;
                //int i_component_serial = 0;
				// Iterate over the components
                //auto itTy = blocosTy.begin();
				for(auto it = blocos.begin();it != blocos.end();it++) {


					std::vector<Stmt *> forStmts;
					//forStmts.push_back(ceBegin);
					for(auto it2 = (*it).begin(); it2 != (*it).end(); it2++) forStmts.push_back(*it2);
					//forStmts.push_back(ceEnd);

					Stmt *forCompS = new (getContext()) CompoundStmt(getContext(), forStmts, SourceLocation(), SourceLocation());

					IdentifierInfo* iiVar = &(idt->get("tls_inner_iterator"));
					VarDecl *condVar = VarDecl::Create(getContext(), declCTX, SourceLocation(), SourceLocation(), iiVar, forCondVar->getType(), nullptr, StorageClass::SC_None);

					condVar->setInit(iceCondVar);

					DeclStmt *declStmt = new (getContext()) DeclStmt(DeclGroupRef(condVar), SourceLocation(), SourceLocation());

					Expr *dreCondVar2 = DeclRefExpr::Create(getContext(), NestedNameSpecifierLoc(), SourceLocation(), condVar, false, *dniforCondVar, condVar->getType(), clang::ExprValueKind::VK_RValue);
					Expr *dreCondVar3 = DeclRefExpr::Create(getContext(), NestedNameSpecifierLoc(), SourceLocation(), forCondVar, false, *dniforCondVar, forCondVar->getType(), clang::ExprValueKind::VK_RValue);
					//BinaryOperator *boforCondAdd = new (getContext()) BinaryOperator(dreCondVar3, IntegerLiteral::Create(getContext(), llvm::APInt(getContext().getTypeSize(getContext().IntTy), (uint64_t) 5), getContext().IntTy, SourceLocation()), BinaryOperatorKind::BO_Add, getContext().IntTy, clang::ExprValueKind::VK_RValue, ExprObjectKind::OK_Ordinary, SourceLocation(), false);
					BinaryOperator *boforCondAdd = new (getContext()) BinaryOperator(dreCondVar3, const_cast<Expr *>(C->getChunkSize()), BinaryOperatorKind::BO_Add, forCondVar->getType(), clang::ExprValueKind::VK_RValue, ExprObjectKind::OK_Ordinary, SourceLocation(), false);
					
					BinaryOperator *boforCondAnt = new (getContext()) BinaryOperator(dreCondVar2, iterations, BinaryOperatorKind::BO_LT,forCondVar->getType(), clang::ExprValueKind::VK_RValue, ExprObjectKind::OK_Ordinary, SourceLocation(), false);
					
					
					BinaryOperator *boforCond = new (getContext()) BinaryOperator(dreCondVar2, boforCondAdd, BinaryOperatorKind::BO_LT, forCondVar->getType(), clang::ExprValueKind::VK_RValue, ExprObjectKind::OK_Ordinary, SourceLocation(), false);
					
					BinaryOperator *boforCond2 = new (getContext()) BinaryOperator(boforCond, boforCondAnt, BinaryOperatorKind::BO_LAnd, forCondVar->getType(), clang::ExprValueKind::VK_RValue, ExprObjectKind::OK_Ordinary, SourceLocation(), false);

					

					Expr *dreCondVar4 = DeclRefExpr::Create(getContext(), NestedNameSpecifierLoc(), SourceLocation(), condVar, false, *dniforCondVar, forCondVar->getType(), clang::ExprValueKind::VK_RValue);
					UnaryOperator *uoforInc = new (getContext()) UnaryOperator(dreCondVar4, UnaryOperatorKind::UO_PostInc, forCondVar->getType(), clang::ExprValueKind::VK_RValue, ExprObjectKind::OK_Ordinary, SourceLocation());

					ForStmt *forStmt = new (getContext()) ForStmt (getContext(), declStmt, boforCond2, condVar, uoforInc, forCompS, SourceLocation(), SourceLocation(), SourceLocation());

					ReplaceVarDecl(forCompS, forCondVar, condVar);

					// Build __bdx_flags[0]
					lhs = DeclRefExpr::Create(getContext(), NestedNameSpecifierLoc(), SourceLocation(), vdPost, false, *dniFlags, qt, clang::ExprValueKind::VK_RValue, dyn_cast<NamedDecl>(vdPost->getCanonicalDecl()));
					rhs = IntegerLiteral::Create(getContext(), llvm::APInt(getContext().getTypeSize(getContext().IntTy), (uint64_t) i_component), getContext().IntTy, SourceLocation());
					ImplicitCastExpr *iceFlags = ImplicitCastExpr::Create(getContext(), getContext().getPointerType(qt), CastKind::CK_ArrayToPointerDecay, lhs, nullptr, ExprValueKind::VK_RValue);
					ArraySubscriptExpr *asexpr = new (getContext()) ArraySubscriptExpr(iceFlags, rhs, getContext().IntTy, clang::ExprValueKind::VK_RValue, clang::ExprObjectKind::OK_Ordinary, SourceLocation());
					
					IdentifierInfo*iiExecute = &(idt->get("execute_in_htm"));
					VarDecl * executeVar = VarDecl::Create(getContext(), declCTX, SourceLocation(), SourceLocation(), iiExecute, getContext().IntTy, nullptr, StorageClass::SC_None);
				
					DeclarationName *dnExecuteVar = new DeclarationName(iiExecute);
		            DeclarationNameInfo *dniExecuteVar = new DeclarationNameInfo(*dnExecuteVar, SourceLocation());

		            Expr *dreExecuteVar = DeclRefExpr::Create(getContext(), NestedNameSpecifierLoc(), SourceLocation(), executeVar, false, *dniExecuteVar, executeVar->getType(), clang::ExprValueKind::VK_RValue);
		            ImplicitCastExpr *iceExecuteVar = ImplicitCastExpr::Create(getContext(), executeVar->getType(), CastKind::CK_LValueToRValue, dreExecuteVar, nullptr, ExprValueKind::VK_RValue);

				
					DeclStmt *declStmt2 = new (getContext()) DeclStmt(DeclGroupRef(executeVar), SourceLocation(), SourceLocation());

					IdentifierInfo*iiRetry = &(idt->get("Retry"));
					LabelDecl * RetryLab = LabelDecl::Create(getContext(), declCTX, SourceLocation(),  iiRetry);
				
					DeclarationName *dnRetryLab = new DeclarationName(iiRetry);
		            DeclarationNameInfo *dniRetryLab = new DeclarationNameInfo(*dnRetryLab, SourceLocation());

		            //Expr *dreRetryLab = DeclRefExpr::Create(getContext(), NestedNameSpecifierLoc(), SourceLocation(), RetryLab, false, *dniRetryLab, nullptr, clang::ExprValueKind::VK_RValue);

					
					
					//IdentifierInfo*iiStarted = &(idt->get("_XBEGIN_STARTED"));
					//VarDecl * startedVar = VarDecl::Create(getContext(), declCTX, SourceLocation(), SourceLocation(), iiStarted, getContext().IntTy, nullptr, StorageClass::SC_None);
				
					//DeclarationName *dnStartedVar = new DeclarationName(iiStarted);
		            //DeclarationNameInfo *dniStartedVar = new DeclarationNameInfo(*dnStartedVar, SourceLocation());

		            //Expr *dreStartedVar = DeclRefExpr::Create(getContext(), NestedNameSpecifierLoc(), SourceLocation(), startedVar, false, *dniStartedVar, startedVar->getType(), clang::ExprValueKind::VK_RValue);
		            //ImplicitCastExpr *iceStartedVar = ImplicitCastExpr::Create(getContext(), startedVar->getType(), CastKind::CK_LValueToRValue, dreStartedVar, nullptr, ExprValueKind::VK_RValue);


					
					IdentifierInfo*iiStatus = &(idt->get("status"));
					VarDecl * statusVar = VarDecl::Create(getContext(), declCTX, SourceLocation(), SourceLocation(), iiStatus, getContext().UnsignedIntTy, nullptr, StorageClass::SC_None);
				
					DeclarationName *dnStatusVar = new DeclarationName(iiStatus);
		            DeclarationNameInfo *dniStatusVar = new DeclarationNameInfo(*dnStatusVar, SourceLocation());

		            Expr *dreStatusVar = DeclRefExpr::Create(getContext(), NestedNameSpecifierLoc(), SourceLocation(), statusVar, false, *dniStatusVar, statusVar->getType(), clang::ExprValueKind::VK_RValue);
		            ImplicitCastExpr *iceStatusVar = ImplicitCastExpr::Create(getContext(), statusVar->getType(), CastKind::CK_LValueToRValue, dreStatusVar, nullptr, ExprValueKind::VK_RValue);

				
					DeclStmt *declStmt3 = new (getContext()) DeclStmt(DeclGroupRef(statusVar), SourceLocation(), SourceLocation());
					
					
					IdentifierInfo*iiNext = &(idt->get("next"));
					VarDecl * nextVar = VarDecl::Create(getContext(), declCTX, SourceLocation(), SourceLocation(), iiNext, getContext().getVolatileType(forCondVar->getType()), nullptr, StorageClass::SC_None);
				
					DeclarationName *dnNextVar = new DeclarationName(iiNext);
		            DeclarationNameInfo *dniNextVar = new DeclarationNameInfo(*dnNextVar, SourceLocation());

		            Expr *dreNextVar = DeclRefExpr::Create(getContext(), NestedNameSpecifierLoc(), SourceLocation(), nextVar, false, *dniNextVar, nextVar->getType(), clang::ExprValueKind::VK_RValue);
		            ImplicitCastExpr *iceNextVar = ImplicitCastExpr::Create(getContext(), nextVar->getType(), CastKind::CK_LValueToRValue, dreNextVar, nullptr, ExprValueKind::VK_RValue);
					
					BinaryOperator *boInitVar = new (getContext()) BinaryOperator(dreNextVar,forCondVal , BinaryOperatorKind::BO_Assign, forCondVar->getType(), clang::ExprValueKind::VK_RValue, ExprObjectKind::OK_Ordinary, SourceLocation(), false);

		                        
		            EmitBinaryOperatorLValue(boInitVar);
		            
		            					
					BinaryOperator *boAssign1 = new (getContext()) BinaryOperator(dreExecuteVar, IntegerLiteral::Create(getContext(), llvm::APInt(getContext().getTypeSize(getContext().IntTy), (uint64_t) 0), getContext().IntTy, SourceLocation()) , BinaryOperatorKind::BO_Assign, getContext().IntTy, clang::ExprValueKind::VK_RValue, ExprObjectKind::OK_Ordinary, SourceLocation(), false);
					
					
							            
//		       		Stmt *nullBody = new (getContext()) NullStmt(SourceLocation());
					LabelStmt *labelStmt = new (getContext()) LabelStmt(SourceLocation(), RetryLab, boAssign1);
					
					BinaryOperator *boNE = new (getContext()) BinaryOperator(dreNextVar, dreCondVar, BinaryOperatorKind::BO_NE, getContext().IntTy, clang::ExprValueKind::VK_RValue, ExprObjectKind::OK_Ordinary, SourceLocation(), false);
					
					
					IdentifierInfo*iiMask = &(idt->get("mask"));
					VarDecl * maskVar = VarDecl::Create(getContext(), declCTX, SourceLocation(), SourceLocation(), iiMask, getContext().VoidPtrTy, nullptr, StorageClass::SC_None);
				
					DeclarationName *dnMaskVar = new DeclarationName(iiMask);
		            DeclarationNameInfo *dniMaskVar = new DeclarationNameInfo(*dnMaskVar, SourceLocation());

		            Expr *dreMaskVar = DeclRefExpr::Create(getContext(), NestedNameSpecifierLoc(), SourceLocation(), maskVar, false, *dniMaskVar, maskVar->getType(), clang::ExprValueKind::VK_RValue);
		            ImplicitCastExpr *iceMaskVar = ImplicitCastExpr::Create(getContext(), nextVar->getType(), CastKind::CK_LValueToRValue, dreMaskVar, nullptr, ExprValueKind::VK_RValue);
		            
		            DeclStmt *declStmt4 = new (getContext()) DeclStmt(DeclGroupRef(maskVar), SourceLocation(), SourceLocation());
					
					//Args1.push_back(dreNextVar);
					//Args1.push_back(dreCondVar);
					
					
					UnaryOperator *uoAd = new (getContext()) UnaryOperator(dreMaskVar, UnaryOperatorKind::UO_AddrOf, getContext().VoidPtrTy, clang::ExprValueKind::VK_RValue, ExprObjectKind::OK_Ordinary, SourceLocation());
					
					ArgsCreate.push_back(uoAd);
					
					DeclRefExpr *dreCreate = DeclRefExpr::Create (getContext(), *nnsl , SourceLocation(), FDCreate, false, *dniCreate, funcTypeCreate, ExprValueKind::VK_RValue);
					ImplicitCastExpr *iceCreate = ImplicitCastExpr::Create(getContext(), getContext().getPointerType(funcTypeCreate), CastKind::CK_FunctionToPointerDecay, dreCreate, nullptr, ExprValueKind::VK_RValue);
					clang::CallExpr *ceCreate = new (getContext()) CallExpr(getContext(), iceCreate, ArgsCreate, getContext().VoidPtrTy, clang::ExprValueKind::VK_RValue, SourceLocation());

					BinaryOperator *boiSub = new (getContext()) BinaryOperator(dreCondVar, forCondVal , BinaryOperatorKind::BO_Sub, getContext().IntTy, clang::ExprValueKind::VK_RValue, ExprObjectKind::OK_Ordinary, SourceLocation(), false);
					
					ParenExpr * peiArg= new (getContext()) ParenExpr(SourceLocation(),SourceLocation(),boiSub);	
					
					BinaryOperator *boiDiv = new (getContext()) BinaryOperator(peiArg, const_cast<Expr *>(C->getChunkSize()) , BinaryOperatorKind::BO_Div, getContext().IntTy, clang::ExprValueKind::VK_RValue, ExprObjectKind::OK_Ordinary, SourceLocation(), false);
					
					ParenExpr * peiArg2= new (getContext()) ParenExpr(SourceLocation(),SourceLocation(),boiDiv);	
					
					BinaryOperator *boiMod = new (getContext()) BinaryOperator(peiArg2, IntegerLiteral::Create(getContext(), llvm::APInt(getContext().getTypeSize(getContext().IntTy), (uint64_t) llvm::sys::getHostNumPhysicalCores()), getContext().IntTy, SourceLocation()) , BinaryOperatorKind::BO_Rem, getContext().IntTy, clang::ExprValueKind::VK_RValue, ExprObjectKind::OK_Ordinary, SourceLocation(), false);

//					BinaryOperator *boiMod = new (getContext()) BinaryOperator(peiArg2, IntegerLiteral::Create(getContext(), llvm::APInt(getContext().getTypeSize(getContext().IntTy), (uint64_t) 4), getContext().IntTy, SourceLocation()) , BinaryOperatorKind::BO_Rem, getContext().IntTy, clang::ExprValueKind::VK_RValue, ExprObjectKind::OK_Ordinary, SourceLocation(), false);
					
					ArgsSetM.push_back(boiMod);
					ArgsSetM.push_back(uoAd);
					


					DeclRefExpr *dreSetM = DeclRefExpr::Create (getContext(), *nnsl , SourceLocation(), FDSetM, false, *dniSetM, funcTypeSetM, ExprValueKind::VK_RValue);
					ImplicitCastExpr *iceSetM = ImplicitCastExpr::Create(getContext(), getContext().getPointerType(funcTypeSetM), CastKind::CK_FunctionToPointerDecay, dreSetM, nullptr, ExprValueKind::VK_RValue);
					clang::CallExpr *ceSetM = new (getContext()) CallExpr(getContext(), iceSetM, ArgsSetM, getContext().VoidPtrTy, clang::ExprValueKind::VK_RValue, SourceLocation());
					
					ArgsSet.push_back(uoAd);
					
					DeclRefExpr *dreSet = DeclRefExpr::Create (getContext(), *nnsl , SourceLocation(), FDSet, false, *dniSet, funcTypeSet, ExprValueKind::VK_RValue);
					ImplicitCastExpr *iceSet = ImplicitCastExpr::Create(getContext(), getContext().getPointerType(funcTypeSet), CastKind::CK_FunctionToPointerDecay, dreSet, nullptr, ExprValueKind::VK_RValue);
					clang::CallExpr *ceSet = new (getContext()) CallExpr(getContext(), iceSet, ArgsSet, getContext().VoidPtrTy, clang::ExprValueKind::VK_RValue, SourceLocation());

					

					// Build an 'begin' call node
					DeclRefExpr *dreBegin = DeclRefExpr::Create (getContext(), *nnsl , SourceLocation(), FDBegin, false, *dniBegin, funcType1, ExprValueKind::VK_RValue);
					ImplicitCastExpr *iceBegin = ImplicitCastExpr::Create(getContext(), getContext().getPointerType(funcType1), CastKind::CK_FunctionToPointerDecay, dreBegin, nullptr, ExprValueKind::VK_RValue);
					clang::CallExpr *ceBegin = new (getContext()) CallExpr(getContext(), iceBegin, Args1, getContext().UnsignedIntTy, clang::ExprValueKind::VK_RValue, SourceLocation());

					
					BinaryOperator *boAssign2 = new (getContext()) BinaryOperator(dreExecuteVar, IntegerLiteral::Create(getContext(), llvm::APInt(getContext().getTypeSize(getContext().IntTy), (uint64_t) 1), getContext().IntTy, SourceLocation()) , BinaryOperatorKind::BO_Assign, getContext().IntTy, clang::ExprValueKind::VK_RValue, ExprObjectKind::OK_Ordinary, SourceLocation(), false);
							 				
					BinaryOperator *boAssign = new (getContext()) BinaryOperator(dreStatusVar, ceBegin , BinaryOperatorKind::BO_Assign, getContext().UnsignedIntTy, clang::ExprValueKind::VK_RValue, ExprObjectKind::OK_Ordinary, SourceLocation(), false);
					


					
					std::vector<Stmt *> stmts,stmts1,stmts2;
					
					stmts.push_back(boAssign2);
					stmts.push_back(boAssign);
					
					
					IntegerLiteral * startedArg = IntegerLiteral::Create(getContext(), llvm::APInt(getContext().getTypeSize(getContext().UnsignedIntTy), (uint64_t) 0), getContext().UnsignedIntTy, SourceLocation());
					
					
					
					//ImplicitCastExpr *iceStartedArg = ImplicitCastExpr::Create(getContext(), getContext().UnsignedIntTy, CastKind::CK_IntegralCast, peAbortArg, nullptr, ExprValueKind::VK_RValue);
					

					
					UnaryOperator *uoNeg = new (getContext()) UnaryOperator(startedArg, UnaryOperatorKind::UO_Not, getContext().UnsignedIntTy, clang::ExprValueKind::VK_RValue, ExprObjectKind::OK_Ordinary, SourceLocation());
					
					ParenExpr * peStartedArg= new (getContext()) ParenExpr(SourceLocation(),SourceLocation(),uoNeg);	
					
					BinaryOperator *boNE1 = new (getContext()) BinaryOperator(dreStatusVar, peStartedArg, BinaryOperatorKind::BO_NE, getContext().UnsignedIntTy, clang::ExprValueKind::VK_RValue, ExprObjectKind::OK_Ordinary, SourceLocation(), false);
					
					GotoStmt *gotos = new (getContext()) GotoStmt(RetryLab, SourceLocation(),SourceLocation());
					
					stmts1.push_back(gotos);

					CompoundStmt *CompSt1 = new (getContext()) CompoundStmt(getContext(), stmts1, SourceLocation(), SourceLocation());
					Stmt *stm1 = CompSt1;

					
					IfStmt *ifs1 = new (getContext()) IfStmt(getContext(), SourceLocation(), false, nullptr, nullptr, boNE1, stm1);
					stmts.push_back(ifs1);
					
					CompoundStmt *CompSt = new (getContext()) CompoundStmt(getContext(), stmts, SourceLocation(), SourceLocation());
					Stmt *stm = CompSt;
					
					IfStmt *ifs = new (getContext()) IfStmt(getContext(), SourceLocation(), false, nullptr, nullptr, boNE, stm);
					
					
					
					//DeclStmt *declStmt3 = new (getContext()) DeclStmt(DeclGroupRef(nextVar), SourceLocation(), SourceLocation());

								
					//Args2.push_back(dreExecuteVar);
					//Args2.push_back(dreNextVar);
					
					IntegerLiteral * abortArg = IntegerLiteral::Create(getContext(), llvm::APInt(getContext().getTypeSize(getContext().IntTy), 255), getContext().IntTy, SourceLocation());
					
					clang::ParenExpr * peAbortArg= new (getContext()) ParenExpr(SourceLocation(),SourceLocation(),abortArg);
					
					ImplicitCastExpr *iceAbortArg = ImplicitCastExpr::Create(getContext(), getContext().CharTy, CastKind::CK_IntegralCast, peAbortArg, nullptr, ExprValueKind::VK_RValue);
					
					Args2.push_back(iceAbortArg);
										

					DeclRefExpr *dreEnd = DeclRefExpr::Create (getContext(), *nnsl , SourceLocation(), FDEnd, false, *dniEnd, funcType2, ExprValueKind::VK_RValue);
					ImplicitCastExpr *iceEnd = ImplicitCastExpr::Create(getContext(), getContext().getPointerType(funcType2), CastKind::CK_FunctionToPointerDecay, dreEnd, nullptr, ExprValueKind::VK_RValue);
					clang::CallExpr *ceEnd = new (getContext()) CallExpr(getContext(), iceEnd, Args1, getContext().VoidTy, clang::ExprValueKind::VK_RValue, SourceLocation());
					
					DeclRefExpr *dreAbort = DeclRefExpr::Create (getContext(), *nnsl , SourceLocation(), FDAbort, false, *dniAbort, funcType3, ExprValueKind::VK_RValue);
					//DeclRefExpr *dreAbort = DeclRefExpr::Create (getContext(), *nnsl , SourceLocation(), FDAbort, false, *dniAbort, getContext().BuiltinFnTy, ExprValueKind::VK_RValue);
					
					
					
    				//DeclRefExpr *dreAbort = new (getContext()) DeclRefExpr(FDAbort, false,getContext().BuiltinFnTy,ExprValueKind::VK_RValue, SourceLocation());
										
					
					ImplicitCastExpr *iceAbort = ImplicitCastExpr::Create(getContext(), getContext().getPointerType(funcType3), CastKind::CK_FunctionToPointerDecay, dreAbort, nullptr, ExprValueKind::VK_RValue);
					//ImplicitCastExpr *iceAbort = ImplicitCastExpr::Create(getContext(), getContext().getPointerType(FDAbort->getType()), CastKind::CK_BuiltinFnToFnPtr, dreAbort, nullptr, ExprValueKind::VK_RValue);
					
					clang::CallExpr *ceAbort = new (getContext()) CallExpr(getContext(), iceAbort, Args2, FDAbort->getCallResultType(), Expr::getValueKindForType(FDAbort->getReturnType()), SourceLocation());


					IfStmt *ifs2 = new (getContext()) IfStmt(getContext(), SourceLocation(), false, nullptr, nullptr, boNE, ceAbort);

					stmts2.push_back(ifs2);
					stmts2.push_back(ceEnd);


					BinaryOperator *nextAdd = new (getContext()) BinaryOperator(dreNextVar, const_cast<Expr *>(C->getChunkSize()), BinaryOperatorKind::BO_Add, forCondVar->getType(), clang::ExprValueKind::VK_RValue, ExprObjectKind::OK_Ordinary, SourceLocation(), false);
					
					BinaryOperator *boAssign3 = new (getContext()) BinaryOperator(dreNextVar, nextAdd , BinaryOperatorKind::BO_Assign, forCondVar->getType(), clang::ExprValueKind::VK_RValue, ExprObjectKind::OK_Ordinary, SourceLocation(), false);
					

					

					CompoundStmt *CompSt2 = new (getContext()) CompoundStmt(getContext(), stmts2, SourceLocation(), SourceLocation());
					Stmt *stm2 = CompSt2;





					IfStmt *ifs3 = new (getContext()) IfStmt(getContext(), SourceLocation(), false, nullptr, nullptr, dreExecuteVar, stm2);


					(*it).clear();
					
					
					//(*it).push_back(declStmt4);
					
					//(*it).push_back(ceCreate);
					
					//(*it).push_back(ceSetM);
					
					//(*it).push_back(ceSet);
					
					(*it).push_back(declStmt2);
					
					(*it).push_back(declStmt3);
					
					
					
					(*it).push_back(labelStmt);
					
					//(*it).push_back(boAssign1);
					
					(*it).push_back(ifs);
					
					(*it).push_back(forStmt);
					(*it).push_back(ifs3);
					
					(*it).push_back(boAssign3);
					
					i_component++;
				}
				
				CompoundStmt *CompS;
				Stmt *st;
				IntegerLiteral *stageFlagArg;
				
				// Iterate over the new components, with the already inserted function 
				for(auto it = blocos.begin();it != blocos.end();it++) {
					ArgsCond.clear();


					stageFlagArg = IntegerLiteral::Create(getContext(), llvm::APInt(getContext().getTypeSize(getContext().IntTy), (uint64_t) 1), getContext().IntTy, SourceLocation());

					ArgsCond.push_back(stageFlagArg);

					// Create the 'CallExpr' node for the IF condition
					DeclRefExpr *dreCond = DeclRefExpr::Create (getContext(), *nnsl , SourceLocation(), FDCond, false, *dniCond, funcTypeCond, ExprValueKind::VK_RValue);
					ImplicitCastExpr *iceCond = ImplicitCastExpr::Create(getContext(), getContext().getPointerType(funcTypeCond), CastKind::CK_FunctionToPointerDecay, dreCond, nullptr, ExprValueKind::VK_RValue);
					clang::CallExpr *ceCond = new (getContext()) CallExpr(getContext(), iceCond, ArgsCond, getContext().IntTy, clang::ExprValueKind::VK_RValue, SourceLocation());

					// Create a new 'CompoundStmt' containing the component statements
					CompS = new (getContext()) CompoundStmt(getContext(), *it, SourceLocation(), SourceLocation());
					st = CompS;
					// Create a new IF and use the recently created 'CompoundStmt' as 'then' condition
					//IfStmt *ifs = new (getContext()) IfStmt(getContext(), SourceLocation(), false, nullptr, nullptr, ceCond, st);
					// Push each IF into a vector
					//ifStmts.push_back(ifs);
				}
				
				
				// Create the 'CompoundStmt' containing all created IFs
				//CompS = new (getContext()) CompoundStmt(getContext(), ifStmts, SourceLocation(), SourceLocation());
				//st = CompS;
                
                	
                
                // Set the 'CompoundStmt' as the For body in the AST
				dyn_cast<ForStmt>(dyn_cast<CapturedStmt>(S.getAssociatedStmt())->getCapturedStmt())->setBody(st);

                //Create a new 'OMPNumThreadsClause' node to insert the 'num_threads' clause in the pragma
				IntegerLiteral *numThreads = IntegerLiteral::Create(getContext(), llvm::APInt(getContext().getTypeSize(getContext().IntTy), (uint64_t) llvm::sys::getHostNumPhysicalCores()), getContext().IntTy, SourceLocation());
//				IntegerLiteral *numThreads = IntegerLiteral::Create(getContext(), llvm::APInt(getContext().getTypeSize(getContext().IntTy), (uint64_t) 4), getContext().IntTy, SourceLocation());
			 	OMPNumThreadsClause *numThreadsClause =  new (getContext()) OMPNumThreadsClause(numThreads, S.getLocStart(), S.getLocEnd(), S.getLocEnd());

				IntegerLiteral *chunkSize = IntegerLiteral::Create(getContext(), llvm::APInt(getContext().getTypeSize(getContext().IntTy), (uint64_t) 1), getContext().IntTy, SourceLocation());
				OMPScheduleClause *scheduleClause = new (getContext()) OMPScheduleClause(S.getLocStart(), S.getLocEnd(), S.getLocEnd(), S.getLocEnd(), S.getLocEnd(), OpenMPScheduleClauseKind::OMPC_SCHEDULE_static, chunkSize, nullptr, OpenMPScheduleClauseModifier::OMPC_SCHEDULE_MODIFIER_unknown, SourceLocation(), OpenMPScheduleClauseModifier::OMPC_SCHEDULE_MODIFIER_unknown, SourceLocation());

//				Expr *name = DeclRefExpr::Create(getContext(), NestedNameSpecifierLoc(), SourceLocation(), vdPost, false, *dniFlags, qt, clang::ExprValueKind::VK_RValue, cast<NamedDecl>(vdPost->getCanonicalDecl()));
//				std::vector<Expr *> shareds;
//				shareds.push_back(name);
//				OMPSharedClause *sharedClause = OMPSharedClause::Create(getContext(), S.getLocStart(), S.getLocEnd(), S.getLocEnd(), shareds);

                std::vector<OMPClause *> clauses = S.clauses().vec();

                OMPPrivateClause *P = nullptr;
                for(auto cl = clauses.begin(); cl != clauses.end(); cl++) {
                  if(isa<OMPPrivateClause>(*cl)) {
                    P = dyn_cast<OMPPrivateClause>(*cl);
                    break;
                  }
                }
                    
			
			    std::vector<Expr *> privates;
                if(P != nullptr) {
                  clang::OMPVarListClause<OMPPrivateClause> *Pvars = dyn_cast<clang::OMPVarListClause<OMPPrivateClause> >(P);

                  for (auto vr = Pvars->varlist_begin(); vr != Pvars->varlist_end(); vr++) {
                    DeclRefExpr *dre = dyn_cast<DeclRefExpr>(*vr);
                    //VarDecl *vd = cast<VarDecl>(dre->getDecl());
                    privates.push_back(dre);
                  }
                }

				
			
				OMPPrivateClause *privateClause = OMPPrivateClause::Create(getContext(), S.getLocStart(), S.getLocEnd(), S.getLocEnd(), privates, privates);

					// Get all original clauses from pragma


				bool found=false;
		            
					for(auto cl = clauses.begin(); cl != clauses.end(); cl++) {
						if(isa<OMPNumThreadsClause>(*cl)) {
							//clauses.erase(cl);
							found=true;
							break;
						}
					}
					
			
					for(auto cl = clauses.begin(); cl != clauses.end(); cl++) {
						if(isa<OMPOrderedClause>(*cl)) {
							clauses.erase(cl);
							break;
						}
					}
					
					for(auto cl = clauses.begin(); cl != clauses.end(); cl++) {
                        if(isa<OMPPrivateClause>(*cl)) {
                           clauses.erase(cl);
                           break;
                        }
                    }

					// Insert 'num_threads' and 'schedule' clause
					
					if (!found)clauses.push_back(numThreadsClause);
					clauses.push_back(scheduleClause);
					clauses.push_back(privateClause);

					OMPLoopDirective::HelperExprs B;

					// Copy the struct 'HelperExprs', which is responsible for controlling loop bounds
			
			
			
				B.IterationVarRef = S.getIterationVariable();
					B.LastIteration = S.getLastIteration();
					B.CalcLastIteration = S.getCalcLastIteration();
					B.PreCond = S.getPreCond();
					B.Cond = S.getCond();
					B.Init = S.getInit();
					B.Inc = S.getInc();
					B.IL = S.getIsLastIterVariable();
					B.LB = S.getLowerBoundVariable();
					B.UB = S.getUpperBoundVariable();
					B.ST = S.getStrideVariable();
					B.EUB = S.getEnsureUpperBound();
					B.NLB = S.getNextLowerBound();
					B.NUB = S.getNextUpperBound();
					B.NumIterations = S.getNumIterations();
					B.PrevLB = S.getPrevLowerBoundVariable();
					B.PrevUB = S.getPrevUpperBoundVariable();
					for(auto it=S.counters().begin();it!=S.counters().end();it++) {(B.Counters).push_back(*it);}
					for(auto it=S.private_counters().begin();it!=S.private_counters().end();it++) {(B.PrivateCounters).push_back(*it);}
					for(auto it=S.inits().begin();it!=S.inits().end();it++) {(B.Inits).push_back(*it);}
					for(auto it=S.updates().begin();it!=S.updates().end();it++) {(B.Updates).push_back(*it);}
					for(auto it=S.finals().begin();it!=S.finals().end();it++) {(B.Finals).push_back(*it);}
					Stmt* temp = const_cast<Stmt *>(S.getPreInits());
					B.PreInits = temp;
			
				OMPForDirective *newS = OMPForDirective::Create(getContext(), S.getLocStart(), S.getLocEnd(), S.getCollapsedNumber(), clauses, S.getAssociatedStmt(), B, false);

				newS->dumpPretty(getContext());
				//newS->dumpColor();

			    //Generate a new CodeGen structure, based on the new 'parallel for' pragma
				auto &&CodeGen = [newS](CodeGenFunction &CGF, PrePostActionTy &) {
				 		CGF.EmitOMPWorksharingLoop(*newS);
				};
				
				
				// Emit the OpenMP using the modified AST
//				emitCommonOMPParallelDirective(*this, *newS, OMPD_for, CodeGen);
                  {
					OMPLexicalScope Scope(*this, *newS, /*AsInlined=*/true);
					CGM.getOpenMPRuntime().emitInlinedDirective(*this, OMPD_for, CodeGen,
								                                S.hasCancel());
				  }

				  // Emit an implicit barrier at the end.
				  if (!S.getSingleClause<OMPNowaitClause>() || HasLastprivates) {
					CGM.getOpenMPRuntime().emitBarrierCall(*this, newS->getLocStart(), OMPD_for);
				  }

			
			}
			else
							  {
				OMPLexicalScope Scope(*this, S, /*AsInlined=*/true);
				CGM.getOpenMPRuntime().emitInlinedDirective(*this, OMPD_for, CodeGen,
						                                    S.hasCancel());
			  }

			  // Emit an implicit barrier at the end.
			  if (!S.getSingleClause<OMPNowaitClause>() || HasLastprivates) {
				CGM.getOpenMPRuntime().emitBarrierCall(*this, S.getLocStart(), OMPD_for);
			  }
	    }
	}
	else {
		//S.dumpColor();
		OMPLexicalScope Scope(*this, S, /*AsInlined=*/true);
		CGM.getOpenMPRuntime().emitInlinedDirective(*this, OMPD_for, CodeGen,
		                                            S.hasCancel());

	  // Emit an implicit barrier at the end.
	  if (!S.getSingleClause<OMPNowaitClause>() || HasLastprivates) {
		CGM.getOpenMPRuntime().emitBarrierCall(*this, S.getLocStart(), OMPD_for);
	  }
	}
#endif
}

void CodeGenFunction::EmitOMPForSimdDirective(const OMPForSimdDirective &S) {
  bool HasLastprivates = false;
  auto &&CodeGen = [&S, &HasLastprivates](CodeGenFunction &CGF,
                                          PrePostActionTy &) {
    HasLastprivates = CGF.EmitOMPWorksharingLoop(S);
  };
  {
    OMPLexicalScope Scope(*this, S, /*AsInlined=*/true);
    CGM.getOpenMPRuntime().emitInlinedDirective(*this, OMPD_simd, CodeGen);
  }

  // Emit an implicit barrier at the end.
  if (!S.getSingleClause<OMPNowaitClause>() || HasLastprivates) {
    CGM.getOpenMPRuntime().emitBarrierCall(*this, S.getLocStart(), OMPD_for);
  }
}

static LValue createSectionLVal(CodeGenFunction &CGF, QualType Ty,
                                const Twine &Name,
                                llvm::Value *Init = nullptr) {
  auto LVal = CGF.MakeAddrLValue(CGF.CreateMemTemp(Ty, Name), Ty);
  if (Init)
    CGF.EmitScalarInit(Init, LVal);
  return LVal;
}

void CodeGenFunction::EmitSections(const OMPExecutableDirective &S) {
  auto *Stmt = cast<CapturedStmt>(S.getAssociatedStmt())->getCapturedStmt();
  auto *CS = dyn_cast<CompoundStmt>(Stmt);
  bool HasLastprivates = false;
  auto &&CodeGen = [&S, Stmt, CS, &HasLastprivates](CodeGenFunction &CGF,
                                                    PrePostActionTy &) {
    auto &C = CGF.CGM.getContext();
    auto KmpInt32Ty = C.getIntTypeForBitwidth(/*DestWidth=*/32, /*Signed=*/1);
    // Emit helper vars inits.
    LValue LB = createSectionLVal(CGF, KmpInt32Ty, ".omp.sections.lb.",
                                  CGF.Builder.getInt32(0));
    auto *GlobalUBVal = CS != nullptr ? CGF.Builder.getInt32(CS->size() - 1)
                                      : CGF.Builder.getInt32(0);
    LValue UB =
        createSectionLVal(CGF, KmpInt32Ty, ".omp.sections.ub.", GlobalUBVal);
    LValue ST = createSectionLVal(CGF, KmpInt32Ty, ".omp.sections.st.",
                                  CGF.Builder.getInt32(1));
    LValue IL = createSectionLVal(CGF, KmpInt32Ty, ".omp.sections.il.",
                                  CGF.Builder.getInt32(0));
    // Loop counter.
    LValue IV = createSectionLVal(CGF, KmpInt32Ty, ".omp.sections.iv.");
    OpaqueValueExpr IVRefExpr(S.getLocStart(), KmpInt32Ty, VK_LValue);
    CodeGenFunction::OpaqueValueMapping OpaqueIV(CGF, &IVRefExpr, IV);
    OpaqueValueExpr UBRefExpr(S.getLocStart(), KmpInt32Ty, VK_LValue);
    CodeGenFunction::OpaqueValueMapping OpaqueUB(CGF, &UBRefExpr, UB);
    // Generate condition for loop.
    BinaryOperator Cond(&IVRefExpr, &UBRefExpr, BO_LE, C.BoolTy, VK_RValue,
                        OK_Ordinary, S.getLocStart(),
                        /*fpContractable=*/false);
    // Increment for loop counter.
    UnaryOperator Inc(&IVRefExpr, UO_PreInc, KmpInt32Ty, VK_RValue, OK_Ordinary,
                      S.getLocStart());
    auto BodyGen = [Stmt, CS, &S, &IV](CodeGenFunction &CGF) {
      // Iterate through all sections and emit a switch construct:
      // switch (IV) {
      //   case 0:
      //     <SectionStmt[0]>;
      //     break;
      // ...
      //   case <NumSection> - 1:
      //     <SectionStmt[<NumSection> - 1]>;
      //     break;
      // }
      // .omp.sections.exit:
      auto *ExitBB = CGF.createBasicBlock(".omp.sections.exit");
      auto *SwitchStmt = CGF.Builder.CreateSwitch(
          CGF.EmitLoadOfLValue(IV, S.getLocStart()).getScalarVal(), ExitBB,
          CS == nullptr ? 1 : CS->size());
      if (CS) {
        unsigned CaseNumber = 0;
        for (auto *SubStmt : CS->children()) {
          auto CaseBB = CGF.createBasicBlock(".omp.sections.case");
          CGF.EmitBlock(CaseBB);
          SwitchStmt->addCase(CGF.Builder.getInt32(CaseNumber), CaseBB);
          CGF.EmitStmt(SubStmt);
          CGF.EmitBranch(ExitBB);
          ++CaseNumber;
        }
      } else {
        auto CaseBB = CGF.createBasicBlock(".omp.sections.case");
        CGF.EmitBlock(CaseBB);
        SwitchStmt->addCase(CGF.Builder.getInt32(0), CaseBB);
        CGF.EmitStmt(Stmt);
        CGF.EmitBranch(ExitBB);
      }
      CGF.EmitBlock(ExitBB, /*IsFinished=*/true);
    };

    CodeGenFunction::OMPPrivateScope LoopScope(CGF);
    if (CGF.EmitOMPFirstprivateClause(S, LoopScope)) {
      // Emit implicit barrier to synchronize threads and avoid data races on
      // initialization of firstprivate variables and post-update of lastprivate
      // variables.
      CGF.CGM.getOpenMPRuntime().emitBarrierCall(
          CGF, S.getLocStart(), OMPD_unknown, /*EmitChecks=*/false,
          /*ForceSimpleCall=*/true);
    }
    CGF.EmitOMPPrivateClause(S, LoopScope);
    HasLastprivates = CGF.EmitOMPLastprivateClauseInit(S, LoopScope);
    CGF.EmitOMPReductionClauseInit(S, LoopScope);
    (void)LoopScope.Privatize();

    // Emit static non-chunked loop.
    OpenMPScheduleTy ScheduleKind;
    ScheduleKind.Schedule = OMPC_SCHEDULE_static;
    CGF.CGM.getOpenMPRuntime().emitForStaticInit(
        CGF, S.getLocStart(), ScheduleKind, /*IVSize=*/32,
        /*IVSigned=*/true, /*Ordered=*/false, IL.getAddress(), LB.getAddress(),
        UB.getAddress(), ST.getAddress());
    // UB = min(UB, GlobalUB);
    auto *UBVal = CGF.EmitLoadOfScalar(UB, S.getLocStart());
    auto *MinUBGlobalUB = CGF.Builder.CreateSelect(
        CGF.Builder.CreateICmpSLT(UBVal, GlobalUBVal), UBVal, GlobalUBVal);
    CGF.EmitStoreOfScalar(MinUBGlobalUB, UB);
    // IV = LB;
    CGF.EmitStoreOfScalar(CGF.EmitLoadOfScalar(LB, S.getLocStart()), IV);
    // while (idx <= UB) { BODY; ++idx; }
    CGF.EmitOMPInnerLoop(S, /*RequiresCleanup=*/false, &Cond, &Inc, BodyGen,
                         [](CodeGenFunction &) {});
    // Tell the runtime we are done.
    CGF.CGM.getOpenMPRuntime().emitForStaticFinish(CGF, S.getLocStart());
    CGF.EmitOMPReductionClauseFinal(S);
    // Emit post-update of the reduction variables if IsLastIter != 0.
    emitPostUpdateForReductionClause(
        CGF, S, [&](CodeGenFunction &CGF) -> llvm::Value * {
          return CGF.Builder.CreateIsNotNull(
              CGF.EmitLoadOfScalar(IL, S.getLocStart()));
        });

    // Emit final copy of the lastprivate variables if IsLastIter != 0.
    if (HasLastprivates)
      CGF.EmitOMPLastprivateClauseFinal(
          S, /*NoFinals=*/false,
          CGF.Builder.CreateIsNotNull(
              CGF.EmitLoadOfScalar(IL, S.getLocStart())));
  };

  bool HasCancel = false;
  if (auto *OSD = dyn_cast<OMPSectionsDirective>(&S))
    HasCancel = OSD->hasCancel();
  else if (auto *OPSD = dyn_cast<OMPParallelSectionsDirective>(&S))
    HasCancel = OPSD->hasCancel();
  CGM.getOpenMPRuntime().emitInlinedDirective(*this, OMPD_sections, CodeGen,
                                              HasCancel);
  // Emit barrier for lastprivates only if 'sections' directive has 'nowait'
  // clause. Otherwise the barrier will be generated by the codegen for the
  // directive.
  if (HasLastprivates && S.getSingleClause<OMPNowaitClause>()) {
    // Emit implicit barrier to synchronize threads and avoid data races on
    // initialization of firstprivate variables.
    CGM.getOpenMPRuntime().emitBarrierCall(*this, S.getLocStart(),
                                           OMPD_unknown);
  }
}

void CodeGenFunction::EmitOMPSectionsDirective(const OMPSectionsDirective &S) {
  {
    OMPLexicalScope Scope(*this, S, /*AsInlined=*/true);
    EmitSections(S);
  }
  // Emit an implicit barrier at the end.
  if (!S.getSingleClause<OMPNowaitClause>()) {
    CGM.getOpenMPRuntime().emitBarrierCall(*this, S.getLocStart(),
                                           OMPD_sections);
  }
}

void CodeGenFunction::EmitOMPSectionDirective(const OMPSectionDirective &S) {
  auto &&CodeGen = [&S](CodeGenFunction &CGF, PrePostActionTy &) {
    CGF.EmitStmt(cast<CapturedStmt>(S.getAssociatedStmt())->getCapturedStmt());
  };
  OMPLexicalScope Scope(*this, S, /*AsInlined=*/true);
  CGM.getOpenMPRuntime().emitInlinedDirective(*this, OMPD_section, CodeGen,
                                              S.hasCancel());
}

void CodeGenFunction::EmitOMPSingleDirective(const OMPSingleDirective &S) {
  llvm::SmallVector<const Expr *, 8> CopyprivateVars;
  llvm::SmallVector<const Expr *, 8> DestExprs;
  llvm::SmallVector<const Expr *, 8> SrcExprs;
  llvm::SmallVector<const Expr *, 8> AssignmentOps;
  // Check if there are any 'copyprivate' clauses associated with this
  // 'single' construct.
  // Build a list of copyprivate variables along with helper expressions
  // (<source>, <destination>, <destination>=<source> expressions)
  for (const auto *C : S.getClausesOfKind<OMPCopyprivateClause>()) {
    CopyprivateVars.append(C->varlists().begin(), C->varlists().end());
    DestExprs.append(C->destination_exprs().begin(),
                     C->destination_exprs().end());
    SrcExprs.append(C->source_exprs().begin(), C->source_exprs().end());
    AssignmentOps.append(C->assignment_ops().begin(),
                         C->assignment_ops().end());
  }
  // Emit code for 'single' region along with 'copyprivate' clauses
  auto &&CodeGen = [&S](CodeGenFunction &CGF, PrePostActionTy &Action) {
    Action.Enter(CGF);
    OMPPrivateScope SingleScope(CGF);
    (void)CGF.EmitOMPFirstprivateClause(S, SingleScope);
    CGF.EmitOMPPrivateClause(S, SingleScope);
    (void)SingleScope.Privatize();
    CGF.EmitStmt(cast<CapturedStmt>(S.getAssociatedStmt())->getCapturedStmt());
  };
  {
    OMPLexicalScope Scope(*this, S, /*AsInlined=*/true);
    CGM.getOpenMPRuntime().emitSingleRegion(*this, CodeGen, S.getLocStart(),
                                            CopyprivateVars, DestExprs,
                                            SrcExprs, AssignmentOps);
  }
  // Emit an implicit barrier at the end (to avoid data race on firstprivate
  // init or if no 'nowait' clause was specified and no 'copyprivate' clause).
  if (!S.getSingleClause<OMPNowaitClause>() && CopyprivateVars.empty()) {
    CGM.getOpenMPRuntime().emitBarrierCall(
        *this, S.getLocStart(),
        S.getSingleClause<OMPNowaitClause>() ? OMPD_unknown : OMPD_single);
  }
}

void CodeGenFunction::EmitOMPMasterDirective(const OMPMasterDirective &S) {
  auto &&CodeGen = [&S](CodeGenFunction &CGF, PrePostActionTy &Action) {
    Action.Enter(CGF);
    CGF.EmitStmt(cast<CapturedStmt>(S.getAssociatedStmt())->getCapturedStmt());
  };
  OMPLexicalScope Scope(*this, S, /*AsInlined=*/true);
  CGM.getOpenMPRuntime().emitMasterRegion(*this, CodeGen, S.getLocStart());
}

void CodeGenFunction::EmitOMPCriticalDirective(const OMPCriticalDirective &S) {
  auto &&CodeGen = [&S](CodeGenFunction &CGF, PrePostActionTy &Action) {
    Action.Enter(CGF);
    CGF.EmitStmt(cast<CapturedStmt>(S.getAssociatedStmt())->getCapturedStmt());
  };
  Expr *Hint = nullptr;
  if (auto *HintClause = S.getSingleClause<OMPHintClause>())
    Hint = HintClause->getHint();
  OMPLexicalScope Scope(*this, S, /*AsInlined=*/true);
  CGM.getOpenMPRuntime().emitCriticalRegion(*this,
                                            S.getDirectiveName().getAsString(),
                                            CodeGen, S.getLocStart(), Hint);
}

static
OMPParallelForDirective*
emitOMPParallelForWithUseTLS(CodeGenFunction& CGF, CodeGenModule& CGM,
    const OMPParallelForDirective& S, const OMPUseClause* useClause,
    const OMPOrderedClause* orderedClause) {

  ASTContext& context = CGM.getContext();

  IdentifierTable *idt2 = new IdentifierTable(CGM.getLangOpts());

  const IntegerLiteral* numForLoopsExpr =
    static_cast<const IntegerLiteral*>(orderedClause->getNumForLoops());
  const unsigned int num_loops = numForLoopsExpr->getValue().getZExtValue();

  llvm::errs() << "tls ("<< num_loops <<")\n";

  FunctionArgList TemplateArgsCond, TemplateArgs1, TemplateArgs2,TemplateArgsCreate, TemplateArgsSetM;

  // Get the original loop body, associated with the 'parallel for' directive
  Stmt *Body = S.getAssociatedStmt()->IgnoreContainers(true);

  Body = dyn_cast<ForStmt>(Body)->getBody();

  // Call the function to separate the body into loop components, based on the 'ordered' directive
  visitBody2(Body);
  checkDeclRefExpr2(Body);

  S.dumpPretty(context);

  int num_components = blocos.size();

  llvm::errs() << "numero componentes "<< num_components <<"\n";

  if(num_components <= 0) {
    llvm::errs() << "error: number of loops must be a strictly positive integer!\n";
    return nullptr;
  }

  //VarDecl *vdPost;
  //QualType qt = context.getConstantArrayType(context.IntTy,
  //    llvm::APInt(context.getTypeSize(context.IntTy), (uint64_t)
  //      num_components), ArrayType::ArraySizeModifier::Normal, 0);
  //IdentifierInfo* iiFlags = &(idt2->get("__tls_flags"));
  //DeclarationName *dnFlags = new DeclarationName(iiFlags);
  //DeclarationNameInfo *dniFlags = new DeclarationNameInfo(*dnFlags,
  //    SourceLocation());
  //vdPost = VarDecl::Create(context, declCTX, SourceLocation(),
  //    SourceLocation(), iiFlags, qt, nullptr, StorageClass::SC_None);


  // Create the identifier for using the condition function call
  IdentifierInfo* iiParam = &(idt2->get("tls_stage_param"));
  ParmVarDecl *stageFlag = ParmVarDecl::Create(context, declCTX,
      SourceLocation(), SourceLocation(), iiParam, context.IntTy, nullptr,
      StorageClass::SC_None, nullptr);
  TemplateArgsCond.push_back(stageFlag);


  IdentifierInfo* iiParam3 = &(idt2->get("cpu"));
  ParmVarDecl *stageFlag3 = ParmVarDecl::Create(context, declCTX,
      SourceLocation(), SourceLocation(), iiParam3, context.IntTy, nullptr,
      StorageClass::SC_None, nullptr);
  TemplateArgsSetM.push_back(stageFlag3);

  IdentifierInfo* iiParam2 = &(idt2->get("mask"));
  ParmVarDecl *stageFlag2 = ParmVarDecl::Create(context, declCTX,
      SourceLocation(), SourceLocation(), iiParam2, context.VoidPtrTy,
      nullptr, StorageClass::SC_None, nullptr);
  TemplateArgsCreate.push_back(stageFlag2);
  TemplateArgsSetM.push_back(stageFlag2);

  // Create the identifier for using the condition function call
  IdentifierInfo* iiParam4 = &(idt2->get("par_abort"));
  ParmVarDecl *stageFlag4 = ParmVarDecl::Create(context, declCTX,
      SourceLocation(), SourceLocation(), iiParam4, context.CharTy, nullptr,
      StorageClass::SC_None, nullptr);
  TemplateArgs2.push_back(stageFlag4);

  // Create a new identifiers table based on the current context
  IdentifierTable *idt = new IdentifierTable(CGM.getLangOpts());
  NestedNameSpecifierLoc *nnsl = new NestedNameSpecifierLoc();

  // Create the identifier for using the condition function call
  IdentifierInfo* iiCond = &(idt->get("__tls_cond__"));
  DeclarationName *dnCond = new DeclarationName(iiCond);
  //DeclarationNameInfo *dniCond = new DeclarationNameInfo(*dnCond, SourceLocation());

  // Create the identifier for finding the start of the component
  IdentifierInfo* iiBegin = &(idt->get("_xbegin"));
  DeclarationName *dnBegin = new DeclarationName(iiBegin);
  DeclarationNameInfo *dniBegin = new DeclarationNameInfo(*dnBegin, SourceLocation());

  // Create the identifier for finding the end of the component
  IdentifierInfo* iiEnd = &(idt->get("_xend"));
  DeclarationName *dnEnd = new DeclarationName(iiEnd);
  DeclarationNameInfo *dniEnd = new DeclarationNameInfo(*dnEnd, SourceLocation());

  IdentifierInfo* iiCreate = &(idt->get("kmp_create_affinity_mask"));
  DeclarationName *dnCreate = new DeclarationName(iiCreate);
  //DeclarationNameInfo *dniCreate = new DeclarationNameInfo(*dnCreate, SourceLocation());

  IdentifierInfo* iiSetM = &(idt->get("kmp_set_affinity_mask_proc"));
  DeclarationName *dnSetM = new DeclarationName(iiSetM);
  //DeclarationNameInfo *dniSetM = new DeclarationNameInfo(*dnSetM, SourceLocation());

  IdentifierInfo* iiSet = &(idt->get("kmp_set_affinity"));
  DeclarationName *dnSet = new DeclarationName(iiSet);
  //DeclarationNameInfo *dniSet = new DeclarationNameInfo(*dnSet, SourceLocation());

  // Create the 'FunctionDecl' nodes for inserting on the AST
  std::vector<QualType> argsCond, args1,args2, argsCreate, argsSetM, argsSet;
  FunctionProtoType::ExtProtoInfo fpi;
  fpi.Variadic = false;

  argsCond.push_back(context.IntTy);
  args2.push_back(context.CharTy);

  argsCreate.push_back(context.VoidPtrTy);

  argsSetM.push_back(context.IntTy);
  argsSetM.push_back(context.VoidPtrTy);

  argsSet.push_back(context.VoidPtrTy);

  QualType funcTypeCond = context.getFunctionType(context.IntTy, argsCond, fpi);
  QualType funcType1 = context.getFunctionType(context.UnsignedIntTy,args1, fpi);
  QualType funcType2 = context.getFunctionType(context.VoidTy, args1, fpi); //xend
  QualType funcType3 = context.getFunctionType(context.VoidTy, args2, fpi); //xabort

  QualType funcTypeCreate = context.getFunctionType(context.VoidTy, argsCreate, fpi);

  QualType funcTypeSetM = context.getFunctionType(context.VoidTy, argsSetM, fpi);

  QualType funcTypeSet = context.getFunctionType(context.VoidTy, argsSet, fpi);

  LinkageSpecDecl* lsd = LinkageSpecDecl::Create(context,
      context.getTranslationUnitDecl(), SourceLocation(), SourceLocation(),
      LinkageSpecDecl::lang_c, false);

  FunctionDecl *FDCond = FunctionDecl::Create(context, lsd,
      SourceLocation(), SourceLocation(), *dnCond, funcTypeCond, nullptr,
      StorageClass::SC_None);

  FunctionDecl *FDBegin = FunctionDecl::Create(context, lsd,
      SourceLocation(), SourceLocation(), *dnBegin, funcType1, nullptr,
      StorageClass::SC_Static);
  FunctionDecl *FDEnd = FunctionDecl::Create(context, lsd, SourceLocation(),
      SourceLocation(), *dnEnd, funcType2, nullptr, StorageClass::SC_Static);
  FunctionDecl *FDAbort;

  IdentifierInfo* iiAbort = &(idt->get("_xabort2"));
  DeclarationName *dnAbort = new DeclarationName(iiAbort);
  DeclarationNameInfo *dniAbort = new DeclarationNameInfo(*dnAbort, SourceLocation());

  FDAbort = FunctionDecl::Create(context, lsd, SourceLocation(),
      SourceLocation(), *dnAbort, funcType3, nullptr, StorageClass::SC_None);

  FunctionDecl *FDCreate = FunctionDecl::Create(context, lsd,
      SourceLocation(), SourceLocation(), *dnCreate, funcTypeCreate, nullptr,
      StorageClass::SC_None);

  FunctionDecl *FDSetM = FunctionDecl::Create(context, lsd,
      SourceLocation(), SourceLocation(), *dnSetM, funcTypeSetM, nullptr,
      StorageClass::SC_None);

  FunctionDecl *FDSet = FunctionDecl::Create(context, lsd,
      SourceLocation(), SourceLocation(), *dnSet, funcTypeSet, nullptr,
      StorageClass::SC_None);

  std::vector<ParmVarDecl *>
    params,params1,params2,paramsCreate,paramsSetM,paramsSet;
  params.push_back(stageFlag);
  params2.push_back(stageFlag4);

  paramsCreate.push_back(stageFlag2);

  paramsSetM.push_back(stageFlag3);
  paramsSetM.push_back(stageFlag2);

  paramsSet.push_back(stageFlag2);

  FDCond->setParams(params);
  FDAbort->setParams(params2);
  FDCreate->setParams(paramsCreate);
  FDSetM->setParams(paramsSetM);
  FDSet->setParams(paramsSet);

  // Auxiliary vector
  std::vector<Stmt *> ifStmts;

  std::vector<Expr *> ArgsCond, Args1 , Args2, ArgsCreate, ArgsSetM, ArgsSet;

  Stmt *forInit =
    dyn_cast<ForStmt>(dyn_cast<CapturedStmt>(S.getAssociatedStmt())->getCapturedStmt())->getInit();
  Expr *forCond =
    cast<ForStmt>(cast<CapturedStmt>(S.getAssociatedStmt())->getCapturedStmt())->getCond();

  Expr *iterations = dyn_cast<BinaryOperator>(forCond)->getRHS();

  BinaryOperator *boInit = dyn_cast<BinaryOperator>(forInit);

  VarDecl *forCondVar =
    dyn_cast<VarDecl>(dyn_cast<DeclRefExpr>(boInit->getLHS())->getDecl());
  Expr *forCondVal = boInit->getRHS();

  IdentifierInfo* iiforCondVar = &(idt->get(forCondVar->getNameAsString()));
  DeclarationName *dnforCondVar = new DeclarationName(iiforCondVar);
  DeclarationNameInfo *dniforCondVar = new DeclarationNameInfo(*dnforCondVar,
      SourceLocation());

  Expr *dreCondVar = DeclRefExpr::Create(context, NestedNameSpecifierLoc(),
      SourceLocation(), forCondVar, false, *dniforCondVar,
      forCondVar->getType(), clang::ExprValueKind::VK_RValue);
  ImplicitCastExpr *iceCondVar = ImplicitCastExpr::Create(context,
      forCondVar->getType(), CastKind::CK_LValueToRValue, dreCondVar,
      nullptr, ExprValueKind::VK_RValue);

  std::vector<Stmt *> InitArray;
  std::vector<VarDecl *> tempVars;

  Expr* chunk;
  if(useClause->getChunkSize()) {
    chunk = const_cast<Expr *>(useClause->getChunkSize());
    if(chunk->getType() != forCondVar->getType()) {
      chunk = ImplicitCastExpr::Create(context, forCondVar->getType(),
          CastKind::CK_IntegralCast, chunk, nullptr, ExprValueKind::VK_RValue);
    }
  } else {
    chunk = IntegerLiteral::Create(context,
        llvm::APInt(context.getTypeSize(forCondVar->getType()), (uint64_t) 1),
        forCondVar->getType(), SourceLocation());
  }

  //Expr *lhs, *rhs;
  int i_component = 0;
  for(auto it = blocos.begin();it != blocos.end();it++) {

    std::vector<Stmt *> forStmts;
    //forStmts.push_back(ceBegin);
    for(auto it2 = (*it).begin(); it2 != (*it).end(); it2++) forStmts.push_back(*it2);
    //forStmts.push_back(ceEnd);

    Stmt *forCompS = new (context) CompoundStmt(context, forStmts, SourceLocation(), SourceLocation());

    IdentifierInfo* iiVar = &(idt->get("tls_inner_iterator"));
    VarDecl *condVar = VarDecl::Create(context, declCTX, SourceLocation(),
        SourceLocation(), iiVar, forCondVar->getType(), nullptr,
        StorageClass::SC_None);

    condVar->setInit(iceCondVar);

    DeclStmt *declStmt = new (context) DeclStmt(DeclGroupRef(condVar),
        SourceLocation(), SourceLocation());

    Expr *dreCondVar2 = DeclRefExpr::Create(context,
        NestedNameSpecifierLoc(), SourceLocation(), condVar, false,
        *dniforCondVar, condVar->getType(), clang::ExprValueKind::VK_RValue);
    Expr *dreCondVar3 = DeclRefExpr::Create(context,
        NestedNameSpecifierLoc(), SourceLocation(), forCondVar, false,
        *dniforCondVar, forCondVar->getType(),
        clang::ExprValueKind::VK_RValue);
    BinaryOperator *boforCondAdd = new (context) BinaryOperator(dreCondVar3,
        chunk, BinaryOperatorKind::BO_Add, forCondVar->getType(),
        clang::ExprValueKind::VK_RValue, ExprObjectKind::OK_Ordinary,
        SourceLocation(), false);

    BinaryOperator *boforCondAnt = new (context) BinaryOperator(dreCondVar2,
        iterations, BinaryOperatorKind::BO_LT,forCondVar->getType(),
        clang::ExprValueKind::VK_RValue, ExprObjectKind::OK_Ordinary,
        SourceLocation(), false);


    BinaryOperator *boforCond = new (context) BinaryOperator(dreCondVar2,
        boforCondAdd, BinaryOperatorKind::BO_LT, forCondVar->getType(),
        clang::ExprValueKind::VK_RValue, ExprObjectKind::OK_Ordinary,
        SourceLocation(), false);

    BinaryOperator *boforCond2 = new (context) BinaryOperator(boforCond,
        boforCondAnt, BinaryOperatorKind::BO_LAnd, forCondVar->getType(),
        clang::ExprValueKind::VK_RValue, ExprObjectKind::OK_Ordinary,
        SourceLocation(), false);

    Expr *dreCondVar4 = DeclRefExpr::Create(context,
        NestedNameSpecifierLoc(), SourceLocation(), condVar, false,
        *dniforCondVar, forCondVar->getType(),
        clang::ExprValueKind::VK_RValue);
    UnaryOperator *uoforInc = new (context) UnaryOperator(dreCondVar4,
        UnaryOperatorKind::UO_PostInc, forCondVar->getType(),
        clang::ExprValueKind::VK_RValue, ExprObjectKind::OK_Ordinary,
        SourceLocation());

    ForStmt *forStmt = new (context) ForStmt (context, declStmt, boforCond2,
        condVar, uoforInc, forCompS, SourceLocation(), SourceLocation(),
        SourceLocation());

    ReplaceVarDecl(forCompS, forCondVar, condVar);

    // Build __bdx_flags[0]
    //lhs = DeclRefExpr::Create(context, NestedNameSpecifierLoc(),
    //    SourceLocation(), vdPost, false, *dniFlags, qt,
    //    clang::ExprValueKind::VK_RValue,
    //    dyn_cast<NamedDecl>(vdPost->getCanonicalDecl()));
    //rhs = IntegerLiteral::Create(context,
    //    llvm::APInt(context.getTypeSize(context.IntTy), (uint64_t)
    //      i_component), context.IntTy, SourceLocation());
    //ImplicitCastExpr *iceFlags = ImplicitCastExpr::Create(context,
    //    context.getPointerType(qt), CastKind::CK_ArrayToPointerDecay, lhs,
    //    nullptr, ExprValueKind::VK_RValue);
    //ArraySubscriptExpr *asexpr = new (context) ArraySubscriptExpr(iceFlags,
    //    rhs, context.IntTy, clang::ExprValueKind::VK_RValue,
    //    clang::ExprObjectKind::OK_Ordinary, SourceLocation());

    IdentifierInfo*iiExecute = &(idt->get("execute_in_htm"));
    VarDecl * executeVar = VarDecl::Create(context, declCTX,
        SourceLocation(), SourceLocation(), iiExecute, context.IntTy,
        nullptr, StorageClass::SC_None);

    DeclarationName *dnExecuteVar = new DeclarationName(iiExecute);
    DeclarationNameInfo *dniExecuteVar = new
      DeclarationNameInfo(*dnExecuteVar, SourceLocation());

    Expr *dreExecuteVar = DeclRefExpr::Create(context,
        NestedNameSpecifierLoc(), SourceLocation(), executeVar, false,
        *dniExecuteVar, executeVar->getType(),
        clang::ExprValueKind::VK_RValue);
    //ImplicitCastExpr *iceExecuteVar = ImplicitCastExpr::Create(context,
    //    executeVar->getType(), CastKind::CK_LValueToRValue, dreExecuteVar,
    //    nullptr, ExprValueKind::VK_RValue);


    DeclStmt *declStmt2 = new (context) DeclStmt(DeclGroupRef(executeVar),
        SourceLocation(), SourceLocation());

    IdentifierInfo*iiRetry = &(idt->get("Retry"));
    LabelDecl * RetryLab = LabelDecl::Create(context, declCTX,
        SourceLocation(),  iiRetry);

    //DeclarationName *dnRetryLab = new DeclarationName(iiRetry);
    //DeclarationNameInfo *dniRetryLab = new DeclarationNameInfo(*dnRetryLab,
    //    SourceLocation());

    IdentifierInfo*iiStatus = &(idt->get("status"));
    VarDecl * statusVar = VarDecl::Create(context, declCTX, SourceLocation(),
        SourceLocation(), iiStatus, context.UnsignedIntTy, nullptr,
        StorageClass::SC_None);

    DeclarationName *dnStatusVar = new DeclarationName(iiStatus);
    DeclarationNameInfo *dniStatusVar = new DeclarationNameInfo(*dnStatusVar,
        SourceLocation());

    Expr *dreStatusVar = DeclRefExpr::Create(context,
        NestedNameSpecifierLoc(), SourceLocation(), statusVar, false,
        *dniStatusVar, statusVar->getType(),
        clang::ExprValueKind::VK_RValue);
    //ImplicitCastExpr *iceStatusVar = ImplicitCastExpr::Create(context,
    //    statusVar->getType(), CastKind::CK_LValueToRValue, dreStatusVar,
    //    nullptr, ExprValueKind::VK_RValue);


    DeclStmt *declStmt3 = new (context) DeclStmt(DeclGroupRef(statusVar),
        SourceLocation(), SourceLocation());


    IdentifierInfo*iiNext = &(idt->get("next"));
    VarDecl * nextVar = VarDecl::Create(context, declCTX, SourceLocation(),
        SourceLocation(), iiNext,
        context.getVolatileType(forCondVar->getType()), nullptr,
        StorageClass::SC_None);

    DeclarationName *dnNextVar = new DeclarationName(iiNext);
    DeclarationNameInfo *dniNextVar = new DeclarationNameInfo(*dnNextVar, SourceLocation());

    Expr *dreNextVar = DeclRefExpr::Create(context, NestedNameSpecifierLoc(),
        SourceLocation(), nextVar, false, *dniNextVar, nextVar->getType(),
        clang::ExprValueKind::VK_RValue);
    //ImplicitCastExpr *iceNextVar = ImplicitCastExpr::Create(context,
    //    nextVar->getType(), CastKind::CK_LValueToRValue, dreNextVar, nullptr,
    //    ExprValueKind::VK_RValue);

    BinaryOperator *boInitVar = new (context)
      BinaryOperator(dreNextVar,forCondVal , BinaryOperatorKind::BO_Assign,
          forCondVar->getType(), clang::ExprValueKind::VK_RValue,
          ExprObjectKind::OK_Ordinary, SourceLocation(), false);

    CGF.EmitBinaryOperatorLValue(boInitVar);

    BinaryOperator *boAssign1 = new (context) BinaryOperator(dreExecuteVar,
        IntegerLiteral::Create(context,
          llvm::APInt(context.getTypeSize(context.IntTy), (uint64_t) 0),
          context.IntTy, SourceLocation()) , BinaryOperatorKind::BO_Assign,
        context.IntTy, clang::ExprValueKind::VK_RValue,
        ExprObjectKind::OK_Ordinary, SourceLocation(), false);

    LabelStmt *labelStmt = new (context) LabelStmt(SourceLocation(), RetryLab, boAssign1);

    BinaryOperator *boNE = new (context) BinaryOperator(dreNextVar,
        dreCondVar, BinaryOperatorKind::BO_NE, context.IntTy,
        clang::ExprValueKind::VK_RValue, ExprObjectKind::OK_Ordinary,
        SourceLocation(), false);


    IdentifierInfo*iiMask = &(idt->get("mask"));
    VarDecl * maskVar = VarDecl::Create(context, declCTX, SourceLocation(),
        SourceLocation(), iiMask, context.VoidPtrTy, nullptr,
        StorageClass::SC_None);

    DeclarationName *dnMaskVar = new DeclarationName(iiMask);
    DeclarationNameInfo *dniMaskVar = new DeclarationNameInfo(*dnMaskVar, SourceLocation());

    Expr *dreMaskVar = DeclRefExpr::Create(context, NestedNameSpecifierLoc(),
        SourceLocation(), maskVar, false, *dniMaskVar, maskVar->getType(),
        clang::ExprValueKind::VK_RValue);
    //ImplicitCastExpr *iceMaskVar = ImplicitCastExpr::Create(context,
    //    nextVar->getType(), CastKind::CK_LValueToRValue, dreMaskVar, nullptr,
    //    ExprValueKind::VK_RValue);

    //DeclStmt *declStmt4 = new (context) DeclStmt(DeclGroupRef(maskVar),
    //    SourceLocation(), SourceLocation());

    UnaryOperator *uoAd = new (context) UnaryOperator(dreMaskVar,
        UnaryOperatorKind::UO_AddrOf, context.VoidPtrTy,
        clang::ExprValueKind::VK_RValue, ExprObjectKind::OK_Ordinary,
        SourceLocation());

    ArgsCreate.push_back(uoAd);

    //DeclRefExpr *dreCreate = DeclRefExpr::Create (context, *nnsl ,
    //    SourceLocation(), FDCreate, false, *dniCreate, funcTypeCreate,
    //    ExprValueKind::VK_RValue);
    //ImplicitCastExpr *iceCreate = ImplicitCastExpr::Create(context,
    //    context.getPointerType(funcTypeCreate),
    //    CastKind::CK_FunctionToPointerDecay, dreCreate, nullptr,
    //    ExprValueKind::VK_RValue);
    //clang::CallExpr *ceCreate = new (context) CallExpr(context,
    //    iceCreate, ArgsCreate, context.VoidPtrTy,
    //    clang::ExprValueKind::VK_RValue, SourceLocation());

    BinaryOperator *boiSub = new (context) BinaryOperator(dreCondVar,
        forCondVal , BinaryOperatorKind::BO_Sub, context.IntTy,
        clang::ExprValueKind::VK_RValue, ExprObjectKind::OK_Ordinary,
        SourceLocation(), false);

    ParenExpr * peiArg= new (context)
      ParenExpr(SourceLocation(),SourceLocation(),boiSub);

    BinaryOperator *boiDiv = new (context) BinaryOperator(peiArg, chunk,
        BinaryOperatorKind::BO_Div, context.IntTy,
        clang::ExprValueKind::VK_RValue, ExprObjectKind::OK_Ordinary,
        SourceLocation(), false);

    ParenExpr * peiArg2= new (context) ParenExpr(SourceLocation(),SourceLocation(),boiDiv);

    BinaryOperator *boiMod = new (context) BinaryOperator(peiArg2,
        IntegerLiteral::Create(context,
          llvm::APInt(context.getTypeSize(context.IntTy),
            (uint64_t) llvm::sys::getHostNumPhysicalCores()),
          context.IntTy, SourceLocation()) , BinaryOperatorKind::BO_Rem,
        context.IntTy, clang::ExprValueKind::VK_RValue,
        ExprObjectKind::OK_Ordinary, SourceLocation(), false);

    ArgsSetM.push_back(boiMod);
    ArgsSetM.push_back(uoAd);

    //DeclRefExpr *dreSetM = DeclRefExpr::Create (context, *nnsl ,
    //    SourceLocation(), FDSetM, false, *dniSetM, funcTypeSetM,
    //    ExprValueKind::VK_RValue);
    //ImplicitCastExpr *iceSetM = ImplicitCastExpr::Create(context,
    //    context.getPointerType(funcTypeSetM),
    //    CastKind::CK_FunctionToPointerDecay, dreSetM, nullptr,
    //    ExprValueKind::VK_RValue);
    //clang::CallExpr *ceSetM = new (context) CallExpr(context,
    //    iceSetM, ArgsSetM, context.VoidPtrTy,
    //    clang::ExprValueKind::VK_RValue, SourceLocation());

    ArgsSet.push_back(uoAd);

    //DeclRefExpr *dreSet = DeclRefExpr::Create (context, *nnsl ,
    //    SourceLocation(), FDSet, false, *dniSet, funcTypeSet,
    //    ExprValueKind::VK_RValue);
    //ImplicitCastExpr *iceSet = ImplicitCastExpr::Create(context,
    //    context.getPointerType(funcTypeSet),
    //    CastKind::CK_FunctionToPointerDecay, dreSet, nullptr,
    //    ExprValueKind::VK_RValue);
    //clang::CallExpr *ceSet = new (context) CallExpr(context,
    //    iceSet, ArgsSet, context.VoidPtrTy,
    //    clang::ExprValueKind::VK_RValue, SourceLocation());

    // Build an 'begin' call node
    DeclRefExpr *dreBegin = DeclRefExpr::Create (context, *nnsl ,
        SourceLocation(), FDBegin, false, *dniBegin, funcType1,
        ExprValueKind::VK_RValue);
    ImplicitCastExpr *iceBegin = ImplicitCastExpr::Create(context,
        context.getPointerType(funcType1),
        CastKind::CK_FunctionToPointerDecay, dreBegin, nullptr,
        ExprValueKind::VK_RValue);
    clang::CallExpr *ceBegin = new (context) CallExpr(context,
        iceBegin, Args1, context.UnsignedIntTy,
        clang::ExprValueKind::VK_RValue, SourceLocation());

    BinaryOperator *boAssign2 = new (context)
      BinaryOperator(dreExecuteVar, IntegerLiteral::Create(context,
            llvm::APInt(context.getTypeSize(context.IntTy),
              (uint64_t) 1), context.IntTy, SourceLocation()) ,
          BinaryOperatorKind::BO_Assign, context.IntTy,
          clang::ExprValueKind::VK_RValue, ExprObjectKind::OK_Ordinary,
          SourceLocation(), false);

    BinaryOperator *boAssign = new (context)
      BinaryOperator(dreStatusVar, ceBegin , BinaryOperatorKind::BO_Assign,
          context.UnsignedIntTy, clang::ExprValueKind::VK_RValue,
          ExprObjectKind::OK_Ordinary, SourceLocation(), false);

    std::vector<Stmt *> stmts,stmts1,stmts2;

    stmts.push_back(boAssign2);
    stmts.push_back(boAssign);

    IntegerLiteral * startedArg = IntegerLiteral::Create(context,
        llvm::APInt(context.getTypeSize(context.UnsignedIntTy),
          (uint64_t) 0), context.UnsignedIntTy, SourceLocation());

    UnaryOperator *uoNeg = new (context) UnaryOperator(startedArg,
        UnaryOperatorKind::UO_Not, context.UnsignedIntTy,
        clang::ExprValueKind::VK_RValue, ExprObjectKind::OK_Ordinary,
        SourceLocation());

    ParenExpr * peStartedArg= new (context)
      ParenExpr(SourceLocation(),SourceLocation(),uoNeg);

    BinaryOperator *boNE1 = new (context) BinaryOperator(dreStatusVar,
        peStartedArg, BinaryOperatorKind::BO_NE, context.UnsignedIntTy,
        clang::ExprValueKind::VK_RValue, ExprObjectKind::OK_Ordinary,
        SourceLocation(), false);

    GotoStmt *gotos = new (context) GotoStmt(RetryLab,
        SourceLocation(),SourceLocation());

    stmts1.push_back(gotos);

    CompoundStmt *CompSt1 = new (context) CompoundStmt(context,
        stmts1, SourceLocation(), SourceLocation());
    Stmt *stm1 = CompSt1;


    IfStmt *ifs1 = new (context) IfStmt(context, SourceLocation(),
        false, nullptr, nullptr, boNE1, stm1);
    stmts.push_back(ifs1);

    CompoundStmt *CompSt = new (context) CompoundStmt(context,
        stmts, SourceLocation(), SourceLocation());
    Stmt *stm = CompSt;

    IfStmt *ifs = new (context) IfStmt(context, SourceLocation(),
        false, nullptr, nullptr, boNE, stm);

    IntegerLiteral * abortArg = IntegerLiteral::Create(context,
        llvm::APInt(context.getTypeSize(context.IntTy), 255),
        context.IntTy, SourceLocation());

    clang::ParenExpr * peAbortArg= new (context)
      ParenExpr(SourceLocation(),SourceLocation(),abortArg);

    ImplicitCastExpr *iceAbortArg = ImplicitCastExpr::Create(context,
        context.CharTy, CastKind::CK_IntegralCast, peAbortArg, nullptr,
        ExprValueKind::VK_RValue);

    Args2.push_back(iceAbortArg);


    DeclRefExpr *dreEnd = DeclRefExpr::Create (context, *nnsl ,
        SourceLocation(), FDEnd, false, *dniEnd, funcType2,
        ExprValueKind::VK_RValue);
    ImplicitCastExpr *iceEnd = ImplicitCastExpr::Create(context,
        context.getPointerType(funcType2),
        CastKind::CK_FunctionToPointerDecay, dreEnd, nullptr,
        ExprValueKind::VK_RValue);
    clang::CallExpr *ceEnd = new (context) CallExpr(context,
        iceEnd, Args1, context.VoidTy, clang::ExprValueKind::VK_RValue,
        SourceLocation());

    DeclRefExpr *dreAbort = DeclRefExpr::Create (context, *nnsl ,
        SourceLocation(), FDAbort, false, *dniAbort, funcType3,
        ExprValueKind::VK_RValue);

    ImplicitCastExpr *iceAbort = ImplicitCastExpr::Create(context,
        context.getPointerType(funcType3),
        CastKind::CK_FunctionToPointerDecay, dreAbort, nullptr,
        ExprValueKind::VK_RValue);

    clang::CallExpr *ceAbort = new (context) CallExpr(context,
        iceAbort, Args2, FDAbort->getCallResultType(),
        Expr::getValueKindForType(FDAbort->getReturnType()),
        SourceLocation());

    IfStmt *ifs2 = new (context) IfStmt(context, SourceLocation(), false,
        nullptr, nullptr, boNE, ceAbort);

    stmts2.push_back(ifs2);
    stmts2.push_back(ceEnd);

    BinaryOperator *nextAdd = new (context) BinaryOperator(dreNextVar,
        chunk, BinaryOperatorKind::BO_Add, forCondVar->getType(),
        clang::ExprValueKind::VK_RValue, ExprObjectKind::OK_Ordinary,
        SourceLocation(), false);

    BinaryOperator *boAssign3 = new (context) BinaryOperator(dreNextVar,
        nextAdd , BinaryOperatorKind::BO_Assign, forCondVar->getType(),
        clang::ExprValueKind::VK_RValue, ExprObjectKind::OK_Ordinary,
        SourceLocation(), false);

    CompoundStmt *CompSt2 = new (context) CompoundStmt(context,
        stmts2, SourceLocation(), SourceLocation());
    Stmt *stm2 = CompSt2;

    IfStmt *ifs3 = new (context) IfStmt(context, SourceLocation(),
        false, nullptr, nullptr, dreExecuteVar, stm2);

    (*it).clear();
    (*it).push_back(declStmt2);
    (*it).push_back(declStmt3);
    (*it).push_back(labelStmt);
    (*it).push_back(ifs);
    (*it).push_back(forStmt);
    (*it).push_back(ifs3);
    (*it).push_back(boAssign3);
    i_component++;
  }

  CompoundStmt *CompS;
  Stmt *st;
  IntegerLiteral *stageFlagArg;

  // Iterate over the new components, with the already inserted function
  for(auto it = blocos.begin();it != blocos.end();it++) {
    ArgsCond.clear();

    stageFlagArg = IntegerLiteral::Create(context,
        llvm::APInt(context.getTypeSize(context.IntTy), (uint64_t)
          1), context.IntTy, SourceLocation());

    ArgsCond.push_back(stageFlagArg);

    // Create the 'CallExpr' node for the IF condition
    //DeclRefExpr *dreCond = DeclRefExpr::Create (context, *nnsl ,
    //    SourceLocation(), FDCond, false, *dniCond, funcTypeCond,
    //    ExprValueKind::VK_RValue);
    //ImplicitCastExpr *iceCond = ImplicitCastExpr::Create(context,
    //    context.getPointerType(funcTypeCond),
    //    CastKind::CK_FunctionToPointerDecay, dreCond, nullptr,
    //    ExprValueKind::VK_RValue);
    //clang::CallExpr *ceCond = new (context) CallExpr(context,
    //    iceCond, ArgsCond, context.IntTy,
    //    clang::ExprValueKind::VK_RValue, SourceLocation());

    // Create a new 'CompoundStmt' containing the component statements
    CompS = new (context) CompoundStmt(context, *it,
        SourceLocation(), SourceLocation());
    st = CompS;
  }

  // Set the 'CompoundStmt' as the For body in the AST
  dyn_cast<ForStmt>(dyn_cast<CapturedStmt>(S.getAssociatedStmt())->getCapturedStmt())->setBody(st);

  //Create a new 'OMPNumThreadsClause' node to insert the 'num_threads' clause in the pragma
  IntegerLiteral *numThreads = IntegerLiteral::Create(context,
      llvm::APInt(context.getTypeSize(context.IntTy), (uint64_t)
        llvm::sys::getHostNumPhysicalCores()), context.IntTy,
      SourceLocation());
  OMPNumThreadsClause *numThreadsClause =  new (context)
    OMPNumThreadsClause(numThreads, S.getLocStart(), S.getLocEnd(),
        S.getLocEnd());

  IntegerLiteral *chunkSize = IntegerLiteral::Create(context,
      llvm::APInt(context.getTypeSize(context.IntTy), (uint64_t)
        1), context.IntTy, SourceLocation());
  OMPScheduleClause *scheduleClause = new (context)
    OMPScheduleClause(S.getLocStart(), S.getLocEnd(), S.getLocEnd(),
        S.getLocEnd(), S.getLocEnd(),
        OpenMPScheduleClauseKind::OMPC_SCHEDULE_static, chunkSize, nullptr,
        OpenMPScheduleClauseModifier::OMPC_SCHEDULE_MODIFIER_unknown,
        SourceLocation(),
        OpenMPScheduleClauseModifier::OMPC_SCHEDULE_MODIFIER_unknown,
        SourceLocation());

  std::vector<OMPClause *> clauses = S.clauses().vec();

  OMPPrivateClause *P = nullptr;
  for(auto cl = clauses.begin(); cl != clauses.end(); cl++) {
    if(isa<OMPPrivateClause>(*cl)) {
      P = dyn_cast<OMPPrivateClause>(*cl);
      break;
    }
  }

  std::vector<Expr *> privates;
  if(P != nullptr) {
    clang::OMPVarListClause<OMPPrivateClause> *Pvars =
      dyn_cast<clang::OMPVarListClause<OMPPrivateClause> >(P);

    for (auto vr = Pvars->varlist_begin(); vr != Pvars->varlist_end(); vr++) {
      DeclRefExpr *dre = dyn_cast<DeclRefExpr>(*vr);
      privates.push_back(dre);
    }
  }

  OMPPrivateClause *privateClause = OMPPrivateClause::Create(context,
      S.getLocStart(), S.getLocEnd(), S.getLocEnd(), privates, privates);


  bool found=false;
  for(auto cl = clauses.begin(); cl != clauses.end(); cl++) {
    if(isa<OMPNumThreadsClause>(*cl)) {
      //clauses.erase(cl);
      found=true;
    }
  }
  for(auto cl = clauses.begin(); cl != clauses.end(); cl++) {
    if(isa<OMPOrderedClause>(*cl)) {
      clauses.erase(cl);
      break;
    }
  }
  for(auto cl = clauses.begin(); cl != clauses.end(); cl++) {
    if(isa<OMPPrivateClause>(*cl)) {
      clauses.erase(cl);
      break;
    }
  }

  // Insert 'num_threads' and 'schedule' clause
  if (!found) clauses.push_back(numThreadsClause);
  clauses.push_back(scheduleClause);
  clauses.push_back(privateClause);

  OMPLoopDirective::HelperExprs B;
  // Copy the struct 'HelperExprs', which is responsible for controlling loop bounds
  B.IterationVarRef = S.getIterationVariable();
  B.LastIteration = S.getLastIteration();
  B.CalcLastIteration = S.getCalcLastIteration();
  B.PreCond = S.getPreCond();
  B.Cond = S.getCond();
  B.Init = S.getInit();
  B.Inc = S.getInc();
  B.IL = S.getIsLastIterVariable();
  B.LB = S.getLowerBoundVariable();
  B.UB = S.getUpperBoundVariable();
  B.ST = S.getStrideVariable();
  B.EUB = S.getEnsureUpperBound();
  B.NLB = S.getNextLowerBound();
  B.NUB = S.getNextUpperBound();
  B.NumIterations = S.getNumIterations();
  B.PrevLB = S.getPrevLowerBoundVariable();
  B.PrevUB = S.getPrevUpperBoundVariable();
  for (auto it=S.counters().begin();it!=S.counters().end();it++) {
    (B.Counters).push_back(*it);
  }
  for (auto it=S.private_counters().begin();it!=S.private_counters().end();it++) {
    (B.PrivateCounters).push_back(*it);
  }
  for (auto it=S.inits().begin();it!=S.inits().end();it++) {
    (B.Inits).push_back(*it);
  }
  for (auto it=S.updates().begin();it!=S.updates().end();it++) {
    (B.Updates).push_back(*it);
  }
  for (auto it=S.finals().begin();it!=S.finals().end();it++) {
    (B.Finals).push_back(*it);
  }
  Stmt* temp = const_cast<Stmt *>(S.getPreInits());
  B.PreInits = temp;

  OMPParallelForDirective *newS = OMPParallelForDirective::Create(context,
      S.getLocStart(), S.getLocEnd(), S.getCollapsedNumber(), clauses,
      S.getAssociatedStmt(), B, false);

  newS->dumpPretty(context);
  return newS;
}

static
OMPParallelForDirective*
emitOMPParallelForWithUseBDX(CodeGenFunction& CGF, CodeGenModule& CGM,
    const OMPParallelForDirective& S, const OMPUseClause* useClause,
    const OMPOrderedClause* orderedClause) {

  ASTContext& context = CGM.getContext();

  // TODO: Fix to handle other values that are not IntegerLiteral
  const IntegerLiteral* numForLoopsExpr =
    static_cast<const IntegerLiteral*>(orderedClause->getNumForLoops());
  const unsigned int num_loops = numForLoopsExpr->getValue().getZExtValue();

  llvm::errs() << "bdx ("<< num_loops <<")\n";

	FunctionArgList TemplateArgsCond, TemplateArgs, TemplateArgsBegin;

	// Get the original loop body, associated with the 'parallel for' directive
	Stmt *Body = S.getAssociatedStmt()->IgnoreContainers(true);

	Body = dyn_cast<ForStmt>(Body)->getBody();

	// Call the function to separate the body into loop components, based on the 'ordered' directive
	visitBody(Body);
	checkDeclRefExpr2(Body);

  S.dumpPretty(context);

  int num_components = blocos.size();

  llvm::errs() << "numero componentes "<< num_components <<"\n";

  if(num_components <= 0) {
    llvm::errs() << "error: number of loops must be a strictly positive integer!\n";
    return nullptr;
  }

  IdentifierTable *idt2 = new IdentifierTable(CGM.getLangOpts());

  std::vector<VarDecl *> bdx_flags;
  VarDecl *vdPost;
  VarDecl *vdBufferSize;
  NestedNameSpecifierLoc *nnsl = new NestedNameSpecifierLoc();

  QualType qt = context.getVolatileType(context.IntTy);

  IdentifierInfo* iiFlags = &(idt2->get("__bdx_flags"));
  DeclarationName *dnFlags = new DeclarationName(iiFlags);
  DeclarationNameInfo *dniFlags = new DeclarationNameInfo(*dnFlags, SourceLocation());

  auto cTy = blocosTy.begin();
  for(int i=0;i<num_components;i++, cTy++) {
    vdPost = VarDecl::Create(context, declCTX, SourceLocation(),
        SourceLocation(), iiFlags, qt, nullptr, StorageClass::SC_None);
    vdPost->setInit(IntegerLiteral::Create(context,
          llvm::APInt(context.getTypeSize(context.IntTy), 0), context.IntTy,
          SourceLocation()));
    bdx_flags.push_back(vdPost);
  }

  // Create the identifier for using the condition function call
  IdentifierInfo* iiParam = &(idt2->get("bdx_stage_param"));

  ParmVarDecl *stageFlag = ParmVarDecl::Create(context, declCTX,
      SourceLocation(), SourceLocation(), iiParam, context.IntTy, nullptr,
      StorageClass::SC_None, nullptr);
  TemplateArgsCond.push_back(stageFlag);

  // Create a new identifiers table based on the current context
  IdentifierTable *idt = new IdentifierTable(CGM.getLangOpts());

  // Create the identifier for using the condition function call
  IdentifierInfo* iiCond = &(idt->get("__bdx_cond__"));
  DeclarationName *dnCond = new DeclarationName(iiCond);
  DeclarationNameInfo *dniCond = new DeclarationNameInfo(*dnCond, SourceLocation());

  // Create the identifier for finding the start of the component
  IdentifierInfo* iiBegin = &(idt->get("__bdx_stage_begin__"));
  DeclarationName *dnBegin = new DeclarationName(iiBegin);
  DeclarationNameInfo *dniBegin = new DeclarationNameInfo(*dnBegin, SourceLocation());

  // Create the identifier for finding the end of the component
  IdentifierInfo* iiEnd = &(idt->get("__bdx_stage_end__"));
  DeclarationName *dnEnd = new DeclarationName(iiEnd);
  DeclarationNameInfo *dniEnd = new DeclarationNameInfo(*dnEnd, SourceLocation());

  // Create the identifier for using the condition function call
  IdentifierInfo* iiNumThreads = &(idt->get("omp_get_thread_num"));
  DeclarationName *dnNumThreads = new DeclarationName(iiNumThreads);
  DeclarationNameInfo *dniNumThreads = new DeclarationNameInfo(*dnNumThreads, SourceLocation());

  // Create the identifier for using the condition function call
  IdentifierInfo* iiNumThreads0 = &(idt->get("omp_get_num_threads"));
  DeclarationName *dnNumThreads0 = new DeclarationName(iiNumThreads0);
  DeclarationNameInfo *dniNumThreads0 = new DeclarationNameInfo(*dnNumThreads0, SourceLocation());

  // Create the 'FunctionDecl' nodes for inserting on the AST
  std::vector<QualType> argsCond, args;
  FunctionProtoType::ExtProtoInfo fpi;
  fpi.Variadic = false;

  argsCond.push_back(context.IntTy);

  LinkageSpecDecl* lsd = LinkageSpecDecl::Create(context,
      context.getTranslationUnitDecl(), SourceLocation(),
      SourceLocation(), LinkageSpecDecl::lang_c, false);

  QualType funcTypeCond = context.getFunctionType(context.IntTy, argsCond, fpi);
  QualType funcTypeNumThreads = context.getFunctionType(context.IntTy, None, fpi);
  QualType funcType = context.getFunctionType(context.VoidTy, None, fpi);

  FunctionDecl *FDCond = FunctionDecl::Create(context, lsd,
      SourceLocation(), SourceLocation(), *dnCond, funcTypeCond, nullptr,
      StorageClass::SC_Extern);
  FDCond->addAttr(NoThrowAttr::CreateImplicit(context));
  FunctionDecl *FDEnd = FunctionDecl::Create(context, lsd,
      SourceLocation(), SourceLocation(), *dnEnd, funcType, nullptr,
      StorageClass::SC_Extern);
  FDEnd->addAttr(NoThrowAttr::CreateImplicit(context));

  FunctionDecl *FDNumThreads = FunctionDecl::Create(context, lsd,
      SourceLocation(), SourceLocation(), *dnNumThreads, funcTypeNumThreads,
      nullptr, StorageClass::SC_Extern);
  FunctionDecl *FDNumThreads0 = FunctionDecl::Create(context, lsd,
      SourceLocation(), SourceLocation(), *dnNumThreads0, funcTypeNumThreads,
      nullptr, StorageClass::SC_Extern);

  std::vector<ParmVarDecl *> params;
  params.push_back(stageFlag);

  FDCond->setParams(params);

  // Auxiliary vector
  std::vector<Stmt *> ifStmts;

  std::vector<Expr *> ArgsCond, Args;

  Stmt *forInit =
    dyn_cast<ForStmt>(dyn_cast<CapturedStmt>(S.getAssociatedStmt())->getCapturedStmt())->getInit();

  BinaryOperator *boInit = dyn_cast<BinaryOperator>(forInit);
  VarDecl *forCondVar = dyn_cast<VarDecl>(dyn_cast<DeclRefExpr>(boInit->getLHS())->getDecl());

  IdentifierInfo* iiParam2 = &(idt2->get("bdx_stage_param"));
  ParmVarDecl *stageFlag2 = ParmVarDecl::Create(context, declCTX,
      SourceLocation(), SourceLocation(), iiParam2, forCondVar->getType(),
      nullptr, StorageClass::SC_None, nullptr);
  TemplateArgsBegin.push_back(stageFlag2);

  argsCond.clear();
  argsCond.push_back(forCondVar->getType());

  QualType funcTypeBegin = context.getFunctionType(context.VoidTy, argsCond, fpi);

  FunctionDecl *FDBegin = FunctionDecl::Create(context, lsd,
      SourceLocation(), SourceLocation(), *dnBegin, funcTypeBegin, nullptr,
      StorageClass::SC_Extern);
  FDBegin->addAttr(NoThrowAttr::CreateImplicit(context));

  params.clear();
  params.push_back(stageFlag2);

  FDBegin->setParams(params);

  IdentifierInfo* iiBS = &(idt2->get("__bdx_buffer_size"));
  DeclarationName *dnBS = new DeclarationName(iiBS);
  DeclarationNameInfo *dniBS = new DeclarationNameInfo(*dnBS, SourceLocation());
  vdBufferSize = VarDecl::Create(context, declCTX, SourceLocation(),
      SourceLocation(), iiBS, context.getVolatileType(forCondVar->getType()),
      nullptr, StorageClass::SC_None);

  Expr *chunk;

  DeclRefExpr *dreInitBS = DeclRefExpr::Create (context, *nnsl ,
      SourceLocation(), vdBufferSize, false, *dniBS, vdBufferSize->getType(),
      ExprValueKind::VK_RValue);
  if(useClause->getChunkSize()) {
    chunk = const_cast<Expr *>(useClause->getChunkSize());
    if(chunk->getType() != forCondVar->getType()) {
      chunk = ImplicitCastExpr::Create(context, forCondVar->getType(),
          CastKind::CK_IntegralCast, chunk, nullptr, ExprValueKind::VK_RValue);
    }
  } else {
    chunk = IntegerLiteral::Create(context,
        llvm::APInt(context.getTypeSize(forCondVar->getType()), (uint64_t) 1),
        forCondVar->getType(), SourceLocation());
  }
  LValue lv = CGF.EmitLValue(dreInitBS);
  RValue rv = CGF.EmitAnyExpr(chunk);
  CGF.EmitStoreThroughLValue(rv, lv);

  for (int i=0; i < num_components; i++, cTy++) {
    Expr *lhs4 = DeclRefExpr::Create(context, NestedNameSpecifierLoc(),
        SourceLocation(), bdx_flags[i], false, *dniFlags, qt,
        clang::ExprValueKind::VK_RValue,
        dyn_cast<NamedDecl>(vdPost->getCanonicalDecl()));
    ImplicitCastExpr *iceArray2 = ImplicitCastExpr::Create(context, qt,
        CastKind::CK_LValueToRValue, lhs4, nullptr, ExprValueKind::VK_RValue);

    BinaryOperator *boAssign2 = new (context) BinaryOperator(iceArray2,
        IntegerLiteral::Create(context,
          llvm::APInt(context.getTypeSize(context.IntTy), (uint64_t) 0),
          context.IntTy, SourceLocation()) , BinaryOperatorKind::BO_Assign,
        context.IntTy, clang::ExprValueKind::VK_RValue,
        ExprObjectKind::OK_Ordinary, SourceLocation(), false);
    CGF.EmitBinaryOperatorLValue(boAssign2);
  }


  IdentifierInfo* iiforCondVar = &(idt->get(forCondVar->getNameAsString()));
  DeclarationName *dnforCondVar = new DeclarationName(iiforCondVar);
  DeclarationNameInfo *dniforCondVar = new DeclarationNameInfo(*dnforCondVar, SourceLocation());

  Expr *dreCondVar = DeclRefExpr::Create(context, NestedNameSpecifierLoc(),
      SourceLocation(), forCondVar, false, *dniforCondVar,
      forCondVar->getType(), clang::ExprValueKind::VK_LValue);
  ImplicitCastExpr *iceCondVar = ImplicitCastExpr::Create(context,
      forCondVar->getType(), CastKind::CK_LValueToRValue, dreCondVar, nullptr,
      ExprValueKind::VK_RValue);

  std::vector<Stmt *> InitArray;
  std::vector<VarDecl *> tempVars;

  Expr *lhs, *lhs2, *lhs3;
  int i_component = 0;
  int i_component_serial = 0;
  // Iterate over the components
  auto itTy = blocosTy.begin();
  for(auto it = blocos.begin();it != blocos.end();it++, itTy++) {


    IdentifierInfo* iiVar = &(idt->get("bdx_inner_iterator"));
    VarDecl *condVar = VarDecl::Create(context, declCTX, SourceLocation(),
        SourceLocation(), iiVar, forCondVar->getType(), nullptr,
        StorageClass::SC_None);

    Expr *dreBeginVar = DeclRefExpr::Create(context, NestedNameSpecifierLoc(),
        SourceLocation(), condVar, false, *dniforCondVar, condVar->getType(),
        clang::ExprValueKind::VK_LValue);
    ImplicitCastExpr *iceBeginVar = ImplicitCastExpr::Create(context,
        condVar->getType(), CastKind::CK_LValueToRValue, dreBeginVar, nullptr,
        ExprValueKind::VK_RValue);

    ArgsCond.clear();
    ArgsCond.push_back(iceBeginVar);

    // Create the 'CallExpr' nodes for inserting on the AST
    DeclRefExpr *dreBegin = DeclRefExpr::Create (context, *nnsl ,
        SourceLocation(), FDBegin, false, *dniBegin, funcTypeBegin,
        ExprValueKind::VK_RValue);
    ImplicitCastExpr *iceBegin = ImplicitCastExpr::Create(context,
        context.getPointerType(funcTypeBegin),
        CastKind::CK_FunctionToPointerDecay, dreBegin, nullptr,
        ExprValueKind::VK_RValue);
    clang::CallExpr *ceBegin = new (context) CallExpr(context, iceBegin,
        ArgsCond, context.VoidTy, clang::ExprValueKind::VK_RValue,
        SourceLocation());

    DeclRefExpr *dreEnd = DeclRefExpr::Create (context, *nnsl ,
        SourceLocation(), FDEnd, false, *dniEnd, funcType,
        ExprValueKind::VK_RValue);
    ImplicitCastExpr *iceEnd = ImplicitCastExpr::Create(context,
        context.getPointerType(funcType), CastKind::CK_FunctionToPointerDecay,
        dreEnd, nullptr, ExprValueKind::VK_RValue);
    clang::CallExpr *ceEnd = new (context) CallExpr(context, iceEnd, None,
        context.VoidTy, clang::ExprValueKind::VK_RValue, SourceLocation());

    std::vector<Stmt *> forStmts;
    forStmts.push_back(ceBegin);
    for(auto it2 = (*it).begin(); it2 != (*it).end(); it2++) forStmts.push_back(*it2);
    forStmts.push_back(ceEnd);

    Stmt *forCompS = new (context) CompoundStmt(context, forStmts, SourceLocation(), SourceLocation());
    condVar->setInit(iceCondVar);
    DeclStmt *declStmt = new (context) DeclStmt(DeclGroupRef(condVar), SourceLocation(), SourceLocation());

    Expr *dreCondVar2 = DeclRefExpr::Create(context, NestedNameSpecifierLoc(),
        SourceLocation(), condVar, false, *dniforCondVar, condVar->getType(),
        clang::ExprValueKind::VK_LValue);
    Expr *dreCondVar3 = DeclRefExpr::Create(context, NestedNameSpecifierLoc(),
        SourceLocation(), forCondVar, false, *dniforCondVar,
        forCondVar->getType(), clang::ExprValueKind::VK_LValue);

    Expr *dreCondVarIC2 = ImplicitCastExpr::Create(context, condVar->getType(),
        CastKind::CK_LValueToRValue, dreCondVar2, nullptr,
        ExprValueKind::VK_RValue);
    Expr *dreCondVarIC3 = ImplicitCastExpr::Create(context,
        forCondVar->getType(), CastKind::CK_LValueToRValue, dreCondVar3,
        nullptr, ExprValueKind::VK_RValue);

    BinaryOperator *boforCondAdd = new (context) BinaryOperator(dreCondVarIC3,
        chunk, BinaryOperatorKind::BO_Add, forCondVar->getType(),
        clang::ExprValueKind::VK_RValue, ExprObjectKind::OK_Ordinary,
        SourceLocation(), false);
    BinaryOperator *boforCond = new (context) BinaryOperator(dreCondVarIC2,
        boforCondAdd, BinaryOperatorKind::BO_LT, forCondVar->getType(),
        clang::ExprValueKind::VK_RValue, ExprObjectKind::OK_Ordinary,
        SourceLocation(), false);

    Expr *dreCondVar4 = DeclRefExpr::Create(context, NestedNameSpecifierLoc(),
        SourceLocation(), condVar, false, *dniforCondVar,
        forCondVar->getType(), clang::ExprValueKind::VK_LValue);
    UnaryOperator *uoforInc = new (context) UnaryOperator(dreCondVar4,
        UnaryOperatorKind::UO_PostInc, forCondVar->getType(),
        clang::ExprValueKind::VK_RValue, ExprObjectKind::OK_Ordinary,
        SourceLocation());

    ForStmt *forStmt = new (context) ForStmt (context, declStmt, boforCond,
        NULL, uoforInc, forCompS, SourceLocation(), SourceLocation(),
        SourceLocation());

    ReplaceVarDecl(forCompS, forCondVar, condVar);

    // Build __bdx_flags[0]
    lhs = DeclRefExpr::Create(context, NestedNameSpecifierLoc(),
        SourceLocation(), bdx_flags[i_component], false, *dniFlags, qt,
        clang::ExprValueKind::VK_RValue,
        dyn_cast<NamedDecl>(vdPost->getCanonicalDecl()));

    // Build another __bdx_flags[0]
    lhs2 = DeclRefExpr::Create(context, NestedNameSpecifierLoc(),
        SourceLocation(), bdx_flags[i_component], false, *dniFlags, qt,
        clang::ExprValueKind::VK_RValue,
        dyn_cast<NamedDecl>(vdPost->getCanonicalDecl()));

    // Build __bdx_flags[0] + 1
    BinaryOperator *boAdd = new (context) BinaryOperator(lhs2,
        IntegerLiteral::Create(context,
          llvm::APInt(context.getTypeSize(context.IntTy), (uint64_t) 1),
          context.IntTy, SourceLocation()), BinaryOperatorKind::BO_Add,
        context.IntTy, clang::ExprValueKind::VK_RValue,
        ExprObjectKind::OK_Ordinary, SourceLocation(), false);
    ParenExpr *peAdd = new (context) ParenExpr(SourceLocation(), SourceLocation(), boAdd);

    DeclRefExpr *dreNumThreads0 = DeclRefExpr::Create (context, *nnsl ,
        SourceLocation(), FDNumThreads0, false, *dniNumThreads0,
        funcTypeNumThreads, ExprValueKind::VK_RValue);
    ImplicitCastExpr *iceNumThreads0 = ImplicitCastExpr::Create(context,
        context.getPointerType(funcTypeNumThreads),
        CastKind::CK_FunctionToPointerDecay, dreNumThreads0, nullptr,
        ExprValueKind::VK_RValue);
    clang::CallExpr *ceNumThreads0 = new (context) CallExpr(context,
        iceNumThreads0, None, context.IntTy, clang::ExprValueKind::VK_RValue,
        SourceLocation());

    // Build (__bdx_flags + 1) % num_components
    BinaryOperator *boMod = new (context) BinaryOperator(peAdd, ceNumThreads0,
        BinaryOperatorKind::BO_Rem, context.IntTy,
        clang::ExprValueKind::VK_RValue, ExprObjectKind::OK_Ordinary,
        SourceLocation(), false);

    // Build __bdx_flags[0] = (__bdx_flags[0] + 1) % num_components
    BinaryOperator *boAssign = new (context) BinaryOperator(lhs, boMod,
        BinaryOperatorKind::BO_Assign, context.IntTy,
        clang::ExprValueKind::VK_RValue, ExprObjectKind::OK_Ordinary,
        SourceLocation(), false);

    // Build another __bdx_flags[0]
    lhs3 = DeclRefExpr::Create(context, NestedNameSpecifierLoc(),
        SourceLocation(), bdx_flags[i_component], false, *dniFlags, qt,
        clang::ExprValueKind::VK_RValue,
        dyn_cast<NamedDecl>(vdPost->getCanonicalDecl()));
    ImplicitCastExpr *iceArray = ImplicitCastExpr::Create(context, qt,
        CastKind::CK_LValueToRValue, lhs3, nullptr, ExprValueKind::VK_RValue);

    // Build an 'omp_get_thread_num' call node
    DeclRefExpr *dreNumThreads = DeclRefExpr::Create (context, *nnsl ,
        SourceLocation(), FDNumThreads, false, *dniNumThreads,
        funcTypeNumThreads, ExprValueKind::VK_RValue);
    ImplicitCastExpr *iceNumThreads = ImplicitCastExpr::Create(context,
        context.getPointerType(funcTypeNumThreads),
        CastKind::CK_FunctionToPointerDecay, dreNumThreads, nullptr,
        ExprValueKind::VK_RValue);
    clang::CallExpr *ceNumThreads = new (context) CallExpr(context,
        iceNumThreads, None, context.IntTy, clang::ExprValueKind::VK_RValue,
        SourceLocation());

    // Build __bdx_flags[0] != omp_get_thread_num()
    BinaryOperator *boNE = new (context) BinaryOperator(iceArray, ceNumThreads,
        BinaryOperatorKind::BO_NE, context.IntTy,
        clang::ExprValueKind::VK_RValue, ExprObjectKind::OK_Ordinary,
        SourceLocation(), false);

    // Build an empty body
    Stmt *nullBody = new (context) NullStmt(SourceLocation());

    // Build while(__bdx_flags[0] != omp_get_thread_num());
    WhileStmt *whileStmt = new (context) WhileStmt(context, nullptr, boNE,
        nullBody, SourceLocation());

    (*it).clear();

    //if(useClause->getUseKind() == OMPC_USE_bdx && (*itTy) == true) (*it).push_back(whileStmt);

    if((*itTy) == false) {
      (*it).push_back(whileStmt);
      //int iVar = 0;
      if(blocosVars.size() > 0) {
        for(auto var = (blocosVars[i_component_serial]).begin(); var != (blocosVars[i_component_serial]).end(); var++) {
          IdentifierInfo *iiVar = &(idt->get("__bdx_loop_carried_" + (*var)->getNameAsString()));
          DeclarationName *dnVar = new DeclarationName(iiVar);
          DeclarationNameInfo *dniVar = new DeclarationNameInfo(*dnVar, SourceLocation());

          IdentifierInfo *iiVar2 = &(idt->get((*var)->getNameAsString()));
          DeclarationName *dnVar2 = new DeclarationName(iiVar2);
          DeclarationNameInfo *dniVar2 = new DeclarationNameInfo(*dnVar2, SourceLocation());

          VarDecl *bdxVar = VarDecl::Create(context, declCTX, SourceLocation(),
              SourceLocation(), iiVar,
              context.getVolatileType((*var)->getType()), nullptr,
              StorageClass::SC_None);
          //bdxVar->setInit((*var)->getInit());
          tempVars.push_back(bdxVar);
          //VarDecl *bdxVar = tempVars[iVar++];

          DeclRefExpr *dreVar = DeclRefExpr::Create (context, *nnsl ,
              SourceLocation(), bdxVar, false, *dniVar,
              context.getVolatileType(bdxVar->getType()),
              ExprValueKind::VK_RValue);
          DeclRefExpr *dreVar2 = DeclRefExpr::Create (context, *nnsl ,
              SourceLocation(), *var, false, *dniVar2, (*var)->getType(),
              ExprValueKind::VK_RValue);
          BinaryOperator *boforVar = new (context) BinaryOperator(dreVar2,
              dreVar, BinaryOperatorKind::BO_Assign, (*var)->getType(),
              clang::ExprValueKind::VK_RValue, ExprObjectKind::OK_Ordinary,
              SourceLocation(), false);
          (*it).push_back(boforVar);
        }
      }
    }

    (*it).push_back(forStmt);
    //if(useClause->getUseKind() == OMPC_USE_bdx && (*itTy) == true) (*it).push_back(boAssign);
    if((*itTy) == false) {
      int iVar = 0;
      if(blocosVars.size() > 0) {
        for(auto var = (blocosVars[i_component_serial]).begin(); var != (blocosVars[i_component_serial]).end(); var++) {

          IdentifierInfo *iiVar = &(idt->get("__bdx_loop_carried_" + (*var)->getNameAsString()));

          DeclarationName *dnVar = new DeclarationName(iiVar);
          DeclarationNameInfo *dniVar = new DeclarationNameInfo(*dnVar, SourceLocation());

          IdentifierInfo *iiVar2 = &(idt->get((*var)->getNameAsString()));
          DeclarationName *dnVar2 = new DeclarationName(iiVar2);
          DeclarationNameInfo *dniVar2 = new DeclarationNameInfo(*dnVar2, SourceLocation());

          VarDecl *bdxVar = tempVars[iVar++];

          DeclRefExpr *dreVar = DeclRefExpr::Create (context, *nnsl ,
              SourceLocation(), bdxVar, false, *dniVar,
              context.getVolatileType(context.IntTy),
              ExprValueKind::VK_RValue);
          DeclRefExpr *dreVar2 = DeclRefExpr::Create (context, *nnsl ,
              SourceLocation(), *var, false, *dniVar2, context.IntTy,
              ExprValueKind::VK_RValue);
          BinaryOperator *boforVar = new (context) BinaryOperator(dreVar,
              dreVar2, BinaryOperatorKind::BO_Assign, context.IntTy,
              clang::ExprValueKind::VK_RValue, ExprObjectKind::OK_Ordinary,
              SourceLocation(), false);
          (*it).push_back(boforVar);

          DeclRefExpr *dreInitVar = DeclRefExpr::Create (context, *nnsl ,
              SourceLocation(), bdxVar, false, *dniVar,
              context.getVolatileType(bdxVar->getType()),
              ExprValueKind::VK_RValue);
          DeclRefExpr *dreInitVar2 = DeclRefExpr::Create (context, *nnsl ,
              SourceLocation(), *var, false, *dniVar2, (*var)->getType(),
              ExprValueKind::VK_RValue);
          ImplicitCastExpr *iceVar2 = ImplicitCastExpr::Create(context,
              (*var)->getType(), CastKind::CK_LValueToRValue, dreInitVar2,
              nullptr, ExprValueKind::VK_RValue);


          BinaryOperator *boInitVar = new (context) BinaryOperator(dreInitVar,
              iceVar2, BinaryOperatorKind::BO_Assign, context.IntTy,
              clang::ExprValueKind::VK_RValue, ExprObjectKind::OK_Ordinary,
              SourceLocation(), false);

          CGF.EmitBinaryOperatorLValue(boInitVar);

        }
      }
      i_component_serial++;
      (*it).push_back(boAssign);
    }
    tempVars.clear();
    i_component++;
  }

  CompoundStmt *CompS;
  itTy = blocosTy.begin();
  IntegerLiteral *stageFlagArg;

  // Iterate over the new components, with the already inserted function call for begin and end
  for(auto it = blocos.begin();it != blocos.end();it++, itTy++) {
    ArgsCond.clear();

    if((*itTy) == true) {
      stageFlagArg = IntegerLiteral::Create(context,
          llvm::APInt(context.getTypeSize(context.IntTy), (uint64_t) 1),
          context.IntTy, SourceLocation());
    } else {
      stageFlagArg = IntegerLiteral::Create(context,
          llvm::APInt(context.getTypeSize(context.IntTy), (uint64_t) 0),
          context.IntTy, SourceLocation());
    }
    ArgsCond.push_back(stageFlagArg);

    // Create the 'CallExpr' node for the IF condition
    DeclRefExpr *dreCond = DeclRefExpr::Create (context, *nnsl ,
        SourceLocation(), FDCond, false, *dniCond, funcTypeCond,
        ExprValueKind::VK_RValue);
    ImplicitCastExpr *iceCond = ImplicitCastExpr::Create(context,
        context.getPointerType(funcTypeCond),
        CastKind::CK_FunctionToPointerDecay, dreCond, nullptr,
        ExprValueKind::VK_RValue);
    clang::CallExpr *ceCond = new (context) CallExpr(context, iceCond,
        ArgsCond, context.IntTy, clang::ExprValueKind::VK_RValue,
        SourceLocation());

    // Create a new 'CompoundStmt' containing the component statements
    CompS = new (context) CompoundStmt(context, *it, SourceLocation(), SourceLocation());
    Stmt *st = CompS;
    // Create a new IF and use the recently created 'CompoundStmt' as 'then' condition
    IfStmt *ifs = new (context) IfStmt(context, SourceLocation(), false, nullptr, nullptr, ceCond, st);
    // Push each IF into a vector
    ifStmts.push_back(ifs);
  }

  // Create the 'CompoundStmt' containing all created IFs
  CompS = new (context) CompoundStmt(context, ifStmts, SourceLocation(), SourceLocation());
  Stmt *st = CompS;

  // Set the 'CompoundStmt' as the For body in the AST
  dyn_cast<ForStmt>(dyn_cast<CapturedStmt>(S.getAssociatedStmt())->getCapturedStmt())->setBody(st);

  // Create a new 'OMPNumThreadsClause' node to insert the 'num_threads' clause in the pragma
  IntegerLiteral *numThreads = IntegerLiteral::Create(context,
      llvm::APInt(context.getTypeSize(context.IntTy), (uint64_t)
        blocos.size()), context.IntTy, SourceLocation());
  OMPNumThreadsClause *numThreadsClause =  new (context)
    OMPNumThreadsClause(numThreads, S.getLocStart(), S.getLocEnd(),
        S.getLocEnd());

  IntegerLiteral *chunkSize = IntegerLiteral::Create(context,
      llvm::APInt(context.getTypeSize(context.IntTy), (uint64_t) 1),
      context.IntTy, SourceLocation());
  OMPScheduleClause *scheduleClause = new (context)
    OMPScheduleClause(S.getLocStart(), S.getLocEnd(), S.getLocEnd(),
        S.getLocEnd(), S.getLocEnd(),
        OpenMPScheduleClauseKind::OMPC_SCHEDULE_static, chunkSize, nullptr,
        OpenMPScheduleClauseModifier::OMPC_SCHEDULE_MODIFIER_unknown,
        SourceLocation(),
        OpenMPScheduleClauseModifier::OMPC_SCHEDULE_MODIFIER_unknown,
        SourceLocation());

  std::vector<OMPClause *> clauses = S.clauses().vec();

  OMPPrivateClause *P = nullptr;
  for(auto cl = clauses.begin(); cl != clauses.end(); cl++) {
    if(isa<OMPPrivateClause>(*cl)) {
      P = dyn_cast<OMPPrivateClause>(*cl);
      break;
    }
  }

  std::vector<Expr *> privates;
  if(P != nullptr) {
    clang::OMPVarListClause<OMPPrivateClause> *Pvars =
      dyn_cast<clang::OMPVarListClause<OMPPrivateClause> >(P);
    for (auto vr = Pvars->varlist_begin(); vr != Pvars->varlist_end(); vr++) {
      DeclRefExpr *dre = dyn_cast<DeclRefExpr>(*vr);
      privates.push_back(dre);
    }
  }

  for(auto comp = blocosVars.begin(); comp != blocosVars.end(); comp++) {
    for(auto var = (*comp).begin(); var != (*comp).end(); var++) {
      DeclarationNameInfo *dniPvtVar = new
        DeclarationNameInfo((*var)->getDeclName(), SourceLocation());
      Expr *name = DeclRefExpr::Create(context, NestedNameSpecifierLoc(),
          SourceLocation(), *var, false, *dniPvtVar, (*var)->getType(),
          clang::ExprValueKind::VK_RValue,
          dyn_cast<NamedDecl>(vdPost->getCanonicalDecl()));
      privates.push_back(name);
    }
  }

  OMPPrivateClause *privateClause = OMPPrivateClause::Create(context,
      S.getLocStart(), S.getLocEnd(), S.getLocEnd(), privates, privates);

  if(useClause->getUseKind() == OMPC_USE_bdx) {
    for(auto cl = clauses.begin(); cl != clauses.end(); cl++) {
      if(isa<OMPNumThreadsClause>(*cl)) {
        clauses.erase(cl);
        break;
      }
    }
  }

  for(auto cl = clauses.begin(); cl != clauses.end(); cl++) {
    if(isa<OMPPrivateClause>(*cl)) {
      clauses.erase(cl);
      break;
    }
  }
  for(auto cl = clauses.begin(); cl != clauses.end(); cl++) {
    if(isa<OMPOrderedClause>(*cl)) {
      clauses.erase(cl);
      break;
    }
  }
  for(auto cl = clauses.begin(); cl != clauses.end(); cl++) {
    if(isa<OMPUseClause>(*cl)) {
      clauses.erase(cl);
      break;
    }
  }

  // Insert 'num_threads' and 'schedule' clause
  if(useClause->getUseKind() == OMPC_USE_bdx) clauses.push_back(numThreadsClause);
  clauses.push_back(scheduleClause);
  clauses.push_back(privateClause);

  OMPLoopDirective::HelperExprs B;

  // Copy the struct 'HelperExprs', which is responsible for controlling loop bounds

  B.IterationVarRef = S.getIterationVariable();
  B.LastIteration = S.getLastIteration();
  B.CalcLastIteration = S.getCalcLastIteration();
  B.PreCond = S.getPreCond();
  B.Cond = S.getCond();
  B.Init = S.getInit();
  B.Inc = S.getInc();
  B.IL = S.getIsLastIterVariable();
  B.LB = S.getLowerBoundVariable();
  B.UB = S.getUpperBoundVariable();
  B.ST = S.getStrideVariable();
  B.EUB = S.getEnsureUpperBound();
  B.NLB = S.getNextLowerBound();
  B.NUB = S.getNextUpperBound();
  B.NumIterations = S.getNumIterations();
  B.PrevLB = S.getPrevLowerBoundVariable();
  B.PrevUB = S.getPrevUpperBoundVariable();
  for(auto it=S.counters().begin();it!=S.counters().end();it++) {(B.Counters).push_back(*it);}
  for(auto it=S.private_counters().begin();it!=S.private_counters().end();it++) {(B.PrivateCounters).push_back(*it);}
  for(auto it=S.inits().begin();it!=S.inits().end();it++) {(B.Inits).push_back(*it);}
  for(auto it=S.updates().begin();it!=S.updates().end();it++) {(B.Updates).push_back(*it);}
  for(auto it=S.finals().begin();it!=S.finals().end();it++) {(B.Finals).push_back(*it);}
  Stmt* temp = const_cast<Stmt *>(S.getPreInits());
  B.PreInits = temp;

  // Create a new 'OMPParallelForDirective' node for replacing the original pragma
  OMPParallelForDirective *newS = OMPParallelForDirective::Create(context,
      S.getLocStart(), S.getLocEnd(), S.getCollapsedNumber(), clauses,
      S.getAssociatedStmt(), B, false);

  newS->dumpPretty(context);
  return newS;
}

static
const OMPParallelForDirective*
emitOMPParallelForWithUseClause(CodeGenFunction& CGF, CodeGenModule& CGM,
    const OMPParallelForDirective& S, const OMPUseClause* useClause) {
  OMPParallelForDirective * newS = const_cast<OMPParallelForDirective*>(&S);
	const OMPOrderedClause *orderedClause = S.getSingleClause<OMPOrderedClause>();
  if (orderedClause == nullptr) {
    llvm::errs() << "The 'use' clause must be used with the 'ordered' clause!\n";
    return newS;
  }
  switch(useClause->getUseKind()) {
    case OMPC_USE_dswp:
      llvm::errs() << "error: DSWP not implemented yet!" << '\n';
      break;
    case OMPC_USE_psbdx:
    case OMPC_USE_bdx:
      newS = emitOMPParallelForWithUseBDX(CGF, CGM, S, useClause, orderedClause);
      break;
    case OMPC_USE_tls:
      newS = emitOMPParallelForWithUseTLS(CGF, CGM, S, useClause, orderedClause);
      break;
    default:
      llvm::errs() << "error: Unknown algorithm select!" << '\n';
      break;
  }
  return newS;
}

void CodeGenFunction::EmitOMPParallelForDirective(
    const OMPParallelForDirective &S) {

	const OMPUseClause *C = S.getSingleClause<OMPUseClause>();

  if (C != nullptr) {
    // Emit parallel-for directive with use clause
    const OMPParallelForDirective *newS =
      emitOMPParallelForWithUseClause(*this, CGM, S, C);
	  auto &&CodeGen = [newS](CodeGenFunction &CGF, PrePostActionTy &) {
	    CGF.EmitOMPWorksharingLoop(*newS);
	  };
		emitCommonOMPParallelDirective(*this, S, OMPD_for, CodeGen);
  } else {
    // Emit directive as a combined directive that consists of two implicit
    // directives: 'parallel' with 'for' directive.
	  auto &&CodeGen = [&S](CodeGenFunction &CGF, PrePostActionTy &) {
      CGF.EmitOMPWorksharingLoop(S);
	  };
		emitCommonOMPParallelDirective(*this, S, OMPD_for, CodeGen);
  }
}

void CodeGenFunction::EmitOMPParallelForSimdDirective(
    const OMPParallelForSimdDirective &S) {
  // Emit directive as a combined directive that consists of two implicit
  // directives: 'parallel' with 'for' directive.
  auto &&CodeGen = [&S](CodeGenFunction &CGF, PrePostActionTy &) {
    CGF.EmitOMPWorksharingLoop(S);
  };
  emitCommonOMPParallelDirective(*this, S, OMPD_simd, CodeGen);
}

void CodeGenFunction::EmitOMPParallelSectionsDirective(
    const OMPParallelSectionsDirective &S) {
  // Emit directive as a combined directive that consists of two implicit
  // directives: 'parallel' with 'sections' directive.
  auto &&CodeGen = [&S](CodeGenFunction &CGF, PrePostActionTy &) {
    CGF.EmitSections(S);
  };
  emitCommonOMPParallelDirective(*this, S, OMPD_sections, CodeGen);
}

void CodeGenFunction::EmitOMPTaskBasedDirective(const OMPExecutableDirective &S,
                                                const RegionCodeGenTy &BodyGen,
                                                const TaskGenTy &TaskGen,
                                                OMPTaskDataTy &Data) {
  // Emit outlined function for task construct.
  auto CS = cast<CapturedStmt>(S.getAssociatedStmt());
  auto *I = CS->getCapturedDecl()->param_begin();
  auto *PartId = std::next(I);
  auto *TaskT = std::next(I, 4);
  // Check if the task is final
  if (const auto *Clause = S.getSingleClause<OMPFinalClause>()) {
    // If the condition constant folds and can be elided, try to avoid emitting
    // the condition and the dead arm of the if/else.
    auto *Cond = Clause->getCondition();
    bool CondConstant;
    if (ConstantFoldsToSimpleInteger(Cond, CondConstant))
      Data.Final.setInt(CondConstant);
    else
      Data.Final.setPointer(EvaluateExprAsBool(Cond));
  } else {
    // By default the task is not final.
    Data.Final.setInt(/*IntVal=*/false);
  }
  // Check if the task has 'priority' clause.
  if (const auto *Clause = S.getSingleClause<OMPPriorityClause>()) {
    // Runtime currently does not support codegen for priority clause argument.
    // TODO: Add codegen for priority clause arg when runtime lib support it.
    auto *Prio = Clause->getPriority();
    Data.Priority.setInt(Prio);
    Data.Priority.setPointer(EmitScalarConversion(
        EmitScalarExpr(Prio), Prio->getType(),
        getContext().getIntTypeForBitwidth(/*DestWidth=*/32, /*Signed=*/1),
        Prio->getExprLoc()));
  }
  // The first function argument for tasks is a thread id, the second one is a
  // part id (0 for tied tasks, >=0 for untied task).
  llvm::DenseSet<const VarDecl *> EmittedAsPrivate;
  // Get list of private variables.
  for (const auto *C : S.getClausesOfKind<OMPPrivateClause>()) {
    auto IRef = C->varlist_begin();
    for (auto *IInit : C->private_copies()) {
      auto *OrigVD = cast<VarDecl>(cast<DeclRefExpr>(*IRef)->getDecl());
      if (EmittedAsPrivate.insert(OrigVD->getCanonicalDecl()).second) {
        Data.PrivateVars.push_back(*IRef);
        Data.PrivateCopies.push_back(IInit);
      }
      ++IRef;
    }
  }
  EmittedAsPrivate.clear();
  // Get list of firstprivate variables.
  for (const auto *C : S.getClausesOfKind<OMPFirstprivateClause>()) {
    auto IRef = C->varlist_begin();
    auto IElemInitRef = C->inits().begin();
    for (auto *IInit : C->private_copies()) {
      auto *OrigVD = cast<VarDecl>(cast<DeclRefExpr>(*IRef)->getDecl());
      if (EmittedAsPrivate.insert(OrigVD->getCanonicalDecl()).second) {
        Data.FirstprivateVars.push_back(*IRef);
        Data.FirstprivateCopies.push_back(IInit);
        Data.FirstprivateInits.push_back(*IElemInitRef);
      }
      ++IRef;
      ++IElemInitRef;
    }
  }
  // Get list of lastprivate variables (for taskloops).
  llvm::DenseMap<const VarDecl *, const DeclRefExpr *> LastprivateDstsOrigs;
  for (const auto *C : S.getClausesOfKind<OMPLastprivateClause>()) {
    auto IRef = C->varlist_begin();
    auto ID = C->destination_exprs().begin();
    for (auto *IInit : C->private_copies()) {
      auto *OrigVD = cast<VarDecl>(cast<DeclRefExpr>(*IRef)->getDecl());
      if (EmittedAsPrivate.insert(OrigVD->getCanonicalDecl()).second) {
        Data.LastprivateVars.push_back(*IRef);
        Data.LastprivateCopies.push_back(IInit);
      }
      LastprivateDstsOrigs.insert(
          {cast<VarDecl>(cast<DeclRefExpr>(*ID)->getDecl()),
           cast<DeclRefExpr>(*IRef)});
      ++IRef;
      ++ID;
    }
  }
  // Build list of dependences.
  for (const auto *C : S.getClausesOfKind<OMPDependClause>())
    for (auto *IRef : C->varlists())
      Data.Dependences.push_back(std::make_pair(C->getDependencyKind(), IRef));
  auto &&CodeGen = [PartId, &S, &Data, CS, &BodyGen, &LastprivateDstsOrigs](
      CodeGenFunction &CGF, PrePostActionTy &Action) {
    // Set proper addresses for generated private copies.
    OMPPrivateScope Scope(CGF);
    if (!Data.PrivateVars.empty() || !Data.FirstprivateVars.empty() ||
        !Data.LastprivateVars.empty()) {
      auto *CopyFn = CGF.Builder.CreateLoad(
          CGF.GetAddrOfLocalVar(CS->getCapturedDecl()->getParam(3)));
      auto *PrivatesPtr = CGF.Builder.CreateLoad(
          CGF.GetAddrOfLocalVar(CS->getCapturedDecl()->getParam(2)));
      // Map privates.
      llvm::SmallVector<std::pair<const VarDecl *, Address>, 16> PrivatePtrs;
      llvm::SmallVector<llvm::Value *, 16> CallArgs;
      CallArgs.push_back(PrivatesPtr);
      for (auto *E : Data.PrivateVars) {
        auto *VD = cast<VarDecl>(cast<DeclRefExpr>(E)->getDecl());
        Address PrivatePtr = CGF.CreateMemTemp(
            CGF.getContext().getPointerType(E->getType()), ".priv.ptr.addr");
        PrivatePtrs.push_back(std::make_pair(VD, PrivatePtr));
        CallArgs.push_back(PrivatePtr.getPointer());
      }
      for (auto *E : Data.FirstprivateVars) {
        auto *VD = cast<VarDecl>(cast<DeclRefExpr>(E)->getDecl());
        Address PrivatePtr =
            CGF.CreateMemTemp(CGF.getContext().getPointerType(E->getType()),
                              ".firstpriv.ptr.addr");
        PrivatePtrs.push_back(std::make_pair(VD, PrivatePtr));
        CallArgs.push_back(PrivatePtr.getPointer());
      }
      for (auto *E : Data.LastprivateVars) {
        auto *VD = cast<VarDecl>(cast<DeclRefExpr>(E)->getDecl());
        Address PrivatePtr =
            CGF.CreateMemTemp(CGF.getContext().getPointerType(E->getType()),
                              ".lastpriv.ptr.addr");
        PrivatePtrs.push_back(std::make_pair(VD, PrivatePtr));
        CallArgs.push_back(PrivatePtr.getPointer());
      }
      CGF.EmitRuntimeCall(CopyFn, CallArgs);
      for (auto &&Pair : LastprivateDstsOrigs) {
        auto *OrigVD = cast<VarDecl>(Pair.second->getDecl());
        DeclRefExpr DRE(
            const_cast<VarDecl *>(OrigVD),
            /*RefersToEnclosingVariableOrCapture=*/CGF.CapturedStmtInfo->lookup(
                OrigVD) != nullptr,
            Pair.second->getType(), VK_LValue, Pair.second->getExprLoc());
        Scope.addPrivate(Pair.first, [&CGF, &DRE]() {
          return CGF.EmitLValue(&DRE).getAddress();
        });
      }
      for (auto &&Pair : PrivatePtrs) {
        Address Replacement(CGF.Builder.CreateLoad(Pair.second),
                            CGF.getContext().getDeclAlign(Pair.first));
        Scope.addPrivate(Pair.first, [Replacement]() { return Replacement; });
      }
    }
    (void)Scope.Privatize();

    Action.Enter(CGF);
    BodyGen(CGF);
  };
  auto *OutlinedFn = CGM.getOpenMPRuntime().emitTaskOutlinedFunction(
      S, *I, *PartId, *TaskT, S.getDirectiveKind(), CodeGen, Data.Tied,
      Data.NumberOfParts);
  OMPLexicalScope Scope(*this, S);
  TaskGen(*this, OutlinedFn, Data);
}

void CodeGenFunction::EmitOMPTaskDirective(const OMPTaskDirective &S) {
  // Emit outlined function for task construct.
  auto CS = cast<CapturedStmt>(S.getAssociatedStmt());
  auto CapturedStruct = GenerateCapturedStmtArgument(*CS);
  auto SharedsTy = getContext().getRecordType(CS->getCapturedRecordDecl());
  const Expr *IfCond = nullptr;
  for (const auto *C : S.getClausesOfKind<OMPIfClause>()) {
    if (C->getNameModifier() == OMPD_unknown ||
        C->getNameModifier() == OMPD_task) {
      IfCond = C->getCondition();
      break;
    }
  }

  OMPTaskDataTy Data;
  // Check if we should emit tied or untied task.
  Data.Tied = !S.getSingleClause<OMPUntiedClause>();
  auto &&BodyGen = [CS](CodeGenFunction &CGF, PrePostActionTy &) {
    CGF.EmitStmt(CS->getCapturedStmt());
  };
  auto &&TaskGen = [&S, SharedsTy, CapturedStruct,
                    IfCond](CodeGenFunction &CGF, llvm::Value *OutlinedFn,
                            const OMPTaskDataTy &Data) {
    CGF.CGM.getOpenMPRuntime().emitTaskCall(CGF, S.getLocStart(), S, OutlinedFn,
                                            SharedsTy, CapturedStruct, IfCond,
                                            Data);
  };
  EmitOMPTaskBasedDirective(S, BodyGen, TaskGen, Data);
}

void CodeGenFunction::EmitOMPTaskyieldDirective(
    const OMPTaskyieldDirective &S) {
  CGM.getOpenMPRuntime().emitTaskyieldCall(*this, S.getLocStart());
}

void CodeGenFunction::EmitOMPBarrierDirective(const OMPBarrierDirective &S) {
  CGM.getOpenMPRuntime().emitBarrierCall(*this, S.getLocStart(), OMPD_barrier);
}

void CodeGenFunction::EmitOMPTaskwaitDirective(const OMPTaskwaitDirective &S) {
  CGM.getOpenMPRuntime().emitTaskwaitCall(*this, S.getLocStart());
}

void CodeGenFunction::EmitOMPTaskgroupDirective(
    const OMPTaskgroupDirective &S) {
  auto &&CodeGen = [&S](CodeGenFunction &CGF, PrePostActionTy &Action) {
    Action.Enter(CGF);
    CGF.EmitStmt(cast<CapturedStmt>(S.getAssociatedStmt())->getCapturedStmt());
  };
  OMPLexicalScope Scope(*this, S, /*AsInlined=*/true);
  CGM.getOpenMPRuntime().emitTaskgroupRegion(*this, CodeGen, S.getLocStart());
}

void CodeGenFunction::EmitOMPFlushDirective(const OMPFlushDirective &S) {
  CGM.getOpenMPRuntime().emitFlush(*this, [&]() -> ArrayRef<const Expr *> {
    if (const auto *FlushClause = S.getSingleClause<OMPFlushClause>()) {
      return llvm::makeArrayRef(FlushClause->varlist_begin(),
                                FlushClause->varlist_end());
    }
    return llvm::None;
  }(), S.getLocStart());
}

void CodeGenFunction::EmitOMPDistributeLoop(const OMPDistributeDirective &S) {
  // Emit the loop iteration variable.
  auto IVExpr = cast<DeclRefExpr>(S.getIterationVariable());
  auto IVDecl = cast<VarDecl>(IVExpr->getDecl());
  EmitVarDecl(*IVDecl);

  // Emit the iterations count variable.
  // If it is not a variable, Sema decided to calculate iterations count on each
  // iteration (e.g., it is foldable into a constant).
  if (auto LIExpr = dyn_cast<DeclRefExpr>(S.getLastIteration())) {
    EmitVarDecl(*cast<VarDecl>(LIExpr->getDecl()));
    // Emit calculation of the iterations count.
    EmitIgnoredExpr(S.getCalcLastIteration());
  }

  auto &RT = CGM.getOpenMPRuntime();

  // Check pre-condition.
  {
    OMPLoopScope PreInitScope(*this, S);
    // Skip the entire loop if we don't meet the precondition.
    // If the condition constant folds and can be elided, avoid emitting the
    // whole loop.
    bool CondConstant;
    llvm::BasicBlock *ContBlock = nullptr;
    if (ConstantFoldsToSimpleInteger(S.getPreCond(), CondConstant)) {
      if (!CondConstant)
        return;
    } else {
      auto *ThenBlock = createBasicBlock("omp.precond.then");
      ContBlock = createBasicBlock("omp.precond.end");
      emitPreCond(*this, S, S.getPreCond(), ThenBlock, ContBlock,
                  getProfileCount(&S));
      EmitBlock(ThenBlock);
      incrementProfileCounter(&S);
    }

    // Emit 'then' code.
    {
      // Emit helper vars inits.
      LValue LB =
          EmitOMPHelperVar(*this, cast<DeclRefExpr>(S.getLowerBoundVariable()));
      LValue UB =
          EmitOMPHelperVar(*this, cast<DeclRefExpr>(S.getUpperBoundVariable()));
      LValue ST =
          EmitOMPHelperVar(*this, cast<DeclRefExpr>(S.getStrideVariable()));
      LValue IL =
          EmitOMPHelperVar(*this, cast<DeclRefExpr>(S.getIsLastIterVariable()));

      OMPPrivateScope LoopScope(*this);
      EmitOMPPrivateLoopCounters(S, LoopScope);
      (void)LoopScope.Privatize();

      // Detect the distribute schedule kind and chunk.
      llvm::Value *Chunk = nullptr;
      OpenMPDistScheduleClauseKind ScheduleKind = OMPC_DIST_SCHEDULE_unknown;
      if (auto *C = S.getSingleClause<OMPDistScheduleClause>()) {
        ScheduleKind = C->getDistScheduleKind();
        if (const auto *Ch = C->getChunkSize()) {
          Chunk = EmitScalarExpr(Ch);
          Chunk = EmitScalarConversion(Chunk, Ch->getType(),
          S.getIterationVariable()->getType(),
          S.getLocStart());
        }
      }
      const unsigned IVSize = getContext().getTypeSize(IVExpr->getType());
      const bool IVSigned = IVExpr->getType()->hasSignedIntegerRepresentation();

      // OpenMP [2.10.8, distribute Construct, Description]
      // If dist_schedule is specified, kind must be static. If specified,
      // iterations are divided into chunks of size chunk_size, chunks are
      // assigned to the teams of the league in a round-robin fashion in the
      // order of the team number. When no chunk_size is specified, the
      // iteration space is divided into chunks that are approximately equal
      // in size, and at most one chunk is distributed to each team of the
      // league. The size of the chunks is unspecified in this case.
      if (RT.isStaticNonchunked(ScheduleKind,
                                /* Chunked */ Chunk != nullptr)) {
        RT.emitDistributeStaticInit(*this, S.getLocStart(), ScheduleKind,
                             IVSize, IVSigned, /* Ordered = */ false,
                             IL.getAddress(), LB.getAddress(),
                             UB.getAddress(), ST.getAddress());
        auto LoopExit =
            getJumpDestInCurrentScope(createBasicBlock("omp.loop.exit"));
        // UB = min(UB, GlobalUB);
        EmitIgnoredExpr(S.getEnsureUpperBound());
        // IV = LB;
        EmitIgnoredExpr(S.getInit());
        // while (idx <= UB) { BODY; ++idx; }
        EmitOMPInnerLoop(S, LoopScope.requiresCleanups(), S.getCond(),
                         S.getInc(),
                         [&S, LoopExit](CodeGenFunction &CGF) {
                           CGF.EmitOMPLoopBody(S, LoopExit);
                           CGF.EmitStopPoint(&S);
                         },
                         [](CodeGenFunction &) {});
        EmitBlock(LoopExit.getBlock());
        // Tell the runtime we are done.
        RT.emitForStaticFinish(*this, S.getLocStart());
      } else {
        // Emit the outer loop, which requests its work chunk [LB..UB] from
        // runtime and runs the inner loop to process it.
        EmitOMPDistributeOuterLoop(ScheduleKind, S, LoopScope,
                            LB.getAddress(), UB.getAddress(), ST.getAddress(),
                            IL.getAddress(), Chunk);
      }
    }

    // We're now done with the loop, so jump to the continuation block.
    if (ContBlock) {
      EmitBranch(ContBlock);
      EmitBlock(ContBlock, true);
    }
  }
}

void CodeGenFunction::EmitOMPDistributeDirective(
    const OMPDistributeDirective &S) {
  auto &&CodeGen = [&S](CodeGenFunction &CGF, PrePostActionTy &) {
    CGF.EmitOMPDistributeLoop(S);
  };
  OMPLexicalScope Scope(*this, S, /*AsInlined=*/true);
  CGM.getOpenMPRuntime().emitInlinedDirective(*this, OMPD_distribute, CodeGen,
                                              false);
}

static llvm::Function *emitOutlinedOrderedFunction(CodeGenModule &CGM,
                                                   const CapturedStmt *S) {
  CodeGenFunction CGF(CGM, /*suppressNewContext=*/true);
  CodeGenFunction::CGCapturedStmtInfo CapStmtInfo;
  CGF.CapturedStmtInfo = &CapStmtInfo;
  auto *Fn = CGF.GenerateOpenMPCapturedStmtFunction(*S);
  Fn->addFnAttr(llvm::Attribute::NoInline);
  return Fn;
}

void CodeGenFunction::EmitOMPOrderedDirective(const OMPOrderedDirective &S) {

	/* Luis Felipe -
		const OMPDependClause *depend = S.getSingleClause<OMPDependClause>();
		if(depend) {
			if(depend->getDependencyKind() == OMPC_DEPEND_source) { llvm::errs() << "Source Found!\n"; (*this).isParallel = true;}
			else if(depend->getDependencyKind() == OMPC_DEPEND_sink) { llvm::errs() << "Sink Found! Dependences: \n"; (*this).isParallel = false;}
			for (auto *IRef : depend->varlists()) {
				llvm::Value * v = EmitScalarExpr(IRef);
				llvm::errs() << "Sink Dependence: " << *v << "\n";
			}
		}*/


  if (!S.getAssociatedStmt()) {
    for (const auto *DC : S.getClausesOfKind<OMPDependClause>())
      CGM.getOpenMPRuntime().emitDoacrossOrdered(*this, DC);
    return;
  }
  auto *C = S.getSingleClause<OMPSIMDClause>();
  auto &&CodeGen = [&S, C, this](CodeGenFunction &CGF,
                                 PrePostActionTy &Action) {
    if (C) {
      auto CS = cast<CapturedStmt>(S.getAssociatedStmt());
      llvm::SmallVector<llvm::Value *, 16> CapturedVars;
      CGF.GenerateOpenMPCapturedVars(*CS, CapturedVars);
      auto *OutlinedFn = emitOutlinedOrderedFunction(CGM, CS);
      CGF.EmitNounwindRuntimeCall(OutlinedFn, CapturedVars);
    } else {
      Action.Enter(CGF);
      CGF.EmitStmt(
          cast<CapturedStmt>(S.getAssociatedStmt())->getCapturedStmt());
    }
  };
  OMPLexicalScope Scope(*this, S, /*AsInlined=*/true);
  CGM.getOpenMPRuntime().emitOrderedRegion(*this, CodeGen, S.getLocStart(), !C);
}

static llvm::Value *convertToScalarValue(CodeGenFunction &CGF, RValue Val,
                                         QualType SrcType, QualType DestType,
                                         SourceLocation Loc) {
  assert(CGF.hasScalarEvaluationKind(DestType) &&
         "DestType must have scalar evaluation kind.");
  assert(!Val.isAggregate() && "Must be a scalar or complex.");
  return Val.isScalar()
             ? CGF.EmitScalarConversion(Val.getScalarVal(), SrcType, DestType,
                                        Loc)
             : CGF.EmitComplexToScalarConversion(Val.getComplexVal(), SrcType,
                                                 DestType, Loc);
}

static CodeGenFunction::ComplexPairTy
convertToComplexValue(CodeGenFunction &CGF, RValue Val, QualType SrcType,
                      QualType DestType, SourceLocation Loc) {
  assert(CGF.getEvaluationKind(DestType) == TEK_Complex &&
         "DestType must have complex evaluation kind.");
  CodeGenFunction::ComplexPairTy ComplexVal;
  if (Val.isScalar()) {
    // Convert the input element to the element type of the complex.
    auto DestElementType = DestType->castAs<ComplexType>()->getElementType();
    auto ScalarVal = CGF.EmitScalarConversion(Val.getScalarVal(), SrcType,
                                              DestElementType, Loc);
    ComplexVal = CodeGenFunction::ComplexPairTy(
        ScalarVal, llvm::Constant::getNullValue(ScalarVal->getType()));
  } else {
    assert(Val.isComplex() && "Must be a scalar or complex.");
    auto SrcElementType = SrcType->castAs<ComplexType>()->getElementType();
    auto DestElementType = DestType->castAs<ComplexType>()->getElementType();
    ComplexVal.first = CGF.EmitScalarConversion(
        Val.getComplexVal().first, SrcElementType, DestElementType, Loc);
    ComplexVal.second = CGF.EmitScalarConversion(
        Val.getComplexVal().second, SrcElementType, DestElementType, Loc);
  }
  return ComplexVal;
}

static void emitSimpleAtomicStore(CodeGenFunction &CGF, bool IsSeqCst,
                                  LValue LVal, RValue RVal) {
  if (LVal.isGlobalReg()) {
    CGF.EmitStoreThroughGlobalRegLValue(RVal, LVal);
  } else {
    CGF.EmitAtomicStore(RVal, LVal,
                        IsSeqCst ? llvm::AtomicOrdering::SequentiallyConsistent
                                 : llvm::AtomicOrdering::Monotonic,
                        LVal.isVolatile(), /*IsInit=*/false);
  }
}

void CodeGenFunction::emitOMPSimpleStore(LValue LVal, RValue RVal,
                                         QualType RValTy, SourceLocation Loc) {
  switch (getEvaluationKind(LVal.getType())) {
  case TEK_Scalar:
    EmitStoreThroughLValue(RValue::get(convertToScalarValue(
                               *this, RVal, RValTy, LVal.getType(), Loc)),
                           LVal);
    break;
  case TEK_Complex:
    EmitStoreOfComplex(
        convertToComplexValue(*this, RVal, RValTy, LVal.getType(), Loc), LVal,
        /*isInit=*/false);
    break;
  case TEK_Aggregate:
    llvm_unreachable("Must be a scalar or complex.");
  }
}

static void EmitOMPAtomicReadExpr(CodeGenFunction &CGF, bool IsSeqCst,
                                  const Expr *X, const Expr *V,
                                  SourceLocation Loc) {
  // v = x;
  assert(V->isLValue() && "V of 'omp atomic read' is not lvalue");
  assert(X->isLValue() && "X of 'omp atomic read' is not lvalue");
  LValue XLValue = CGF.EmitLValue(X);
  LValue VLValue = CGF.EmitLValue(V);
  RValue Res = XLValue.isGlobalReg()
                   ? CGF.EmitLoadOfLValue(XLValue, Loc)
                   : CGF.EmitAtomicLoad(
                         XLValue, Loc,
                         IsSeqCst ? llvm::AtomicOrdering::SequentiallyConsistent
                                  : llvm::AtomicOrdering::Monotonic,
                         XLValue.isVolatile());
  // OpenMP, 2.12.6, atomic Construct
  // Any atomic construct with a seq_cst clause forces the atomically
  // performed operation to include an implicit flush operation without a
  // list.
  if (IsSeqCst)
    CGF.CGM.getOpenMPRuntime().emitFlush(CGF, llvm::None, Loc);
  CGF.emitOMPSimpleStore(VLValue, Res, X->getType().getNonReferenceType(), Loc);
}

static void EmitOMPAtomicWriteExpr(CodeGenFunction &CGF, bool IsSeqCst,
                                   const Expr *X, const Expr *E,
                                   SourceLocation Loc) {
  // x = expr;
  assert(X->isLValue() && "X of 'omp atomic write' is not lvalue");
  emitSimpleAtomicStore(CGF, IsSeqCst, CGF.EmitLValue(X), CGF.EmitAnyExpr(E));
  // OpenMP, 2.12.6, atomic Construct
  // Any atomic construct with a seq_cst clause forces the atomically
  // performed operation to include an implicit flush operation without a
  // list.
  if (IsSeqCst)
    CGF.CGM.getOpenMPRuntime().emitFlush(CGF, llvm::None, Loc);
}

static std::pair<bool, RValue> emitOMPAtomicRMW(CodeGenFunction &CGF, LValue X,
                                                RValue Update,
                                                BinaryOperatorKind BO,
                                                llvm::AtomicOrdering AO,
                                                bool IsXLHSInRHSPart) {
  auto &Context = CGF.CGM.getContext();
  // Allow atomicrmw only if 'x' and 'update' are integer values, lvalue for 'x'
  // expression is simple and atomic is allowed for the given type for the
  // target platform.
  if (BO == BO_Comma || !Update.isScalar() ||
      !Update.getScalarVal()->getType()->isIntegerTy() ||
      !X.isSimple() || (!isa<llvm::ConstantInt>(Update.getScalarVal()) &&
                        (Update.getScalarVal()->getType() !=
                         X.getAddress().getElementType())) ||
      !X.getAddress().getElementType()->isIntegerTy() ||
      !Context.getTargetInfo().hasBuiltinAtomic(
          Context.getTypeSize(X.getType()), Context.toBits(X.getAlignment())))
    return std::make_pair(false, RValue::get(nullptr));

  llvm::AtomicRMWInst::BinOp RMWOp;
  switch (BO) {
  case BO_Add:
    RMWOp = llvm::AtomicRMWInst::Add;
    break;
  case BO_Sub:
    if (!IsXLHSInRHSPart)
      return std::make_pair(false, RValue::get(nullptr));
    RMWOp = llvm::AtomicRMWInst::Sub;
    break;
  case BO_And:
    RMWOp = llvm::AtomicRMWInst::And;
    break;
  case BO_Or:
    RMWOp = llvm::AtomicRMWInst::Or;
    break;
  case BO_Xor:
    RMWOp = llvm::AtomicRMWInst::Xor;
    break;
  case BO_LT:
    RMWOp = X.getType()->hasSignedIntegerRepresentation()
                ? (IsXLHSInRHSPart ? llvm::AtomicRMWInst::Min
                                   : llvm::AtomicRMWInst::Max)
                : (IsXLHSInRHSPart ? llvm::AtomicRMWInst::UMin
                                   : llvm::AtomicRMWInst::UMax);
    break;
  case BO_GT:
    RMWOp = X.getType()->hasSignedIntegerRepresentation()
                ? (IsXLHSInRHSPart ? llvm::AtomicRMWInst::Max
                                   : llvm::AtomicRMWInst::Min)
                : (IsXLHSInRHSPart ? llvm::AtomicRMWInst::UMax
                                   : llvm::AtomicRMWInst::UMin);
    break;
  case BO_Assign:
    RMWOp = llvm::AtomicRMWInst::Xchg;
    break;
  case BO_Mul:
  case BO_Div:
  case BO_Rem:
  case BO_Shl:
  case BO_Shr:
  case BO_LAnd:
  case BO_LOr:
    return std::make_pair(false, RValue::get(nullptr));
  case BO_PtrMemD:
  case BO_PtrMemI:
  case BO_LE:
  case BO_GE:
  case BO_EQ:
  case BO_NE:
  case BO_AddAssign:
  case BO_SubAssign:
  case BO_AndAssign:
  case BO_OrAssign:
  case BO_XorAssign:
  case BO_MulAssign:
  case BO_DivAssign:
  case BO_RemAssign:
  case BO_ShlAssign:
  case BO_ShrAssign:
  case BO_Comma:
    llvm_unreachable("Unsupported atomic update operation");
  }
  auto *UpdateVal = Update.getScalarVal();
  if (auto *IC = dyn_cast<llvm::ConstantInt>(UpdateVal)) {
    UpdateVal = CGF.Builder.CreateIntCast(
        IC, X.getAddress().getElementType(),
        X.getType()->hasSignedIntegerRepresentation());
  }
  auto *Res = CGF.Builder.CreateAtomicRMW(RMWOp, X.getPointer(), UpdateVal, AO);
  return std::make_pair(true, RValue::get(Res));
}

std::pair<bool, RValue> CodeGenFunction::EmitOMPAtomicSimpleUpdateExpr(
    LValue X, RValue E, BinaryOperatorKind BO, bool IsXLHSInRHSPart,
    llvm::AtomicOrdering AO, SourceLocation Loc,
    const llvm::function_ref<RValue(RValue)> &CommonGen) {
  // Update expressions are allowed to have the following forms:
  // x binop= expr; -> xrval + expr;
  // x++, ++x -> xrval + 1;
  // x--, --x -> xrval - 1;
  // x = x binop expr; -> xrval binop expr
  // x = expr Op x; - > expr binop xrval;
  auto Res = emitOMPAtomicRMW(*this, X, E, BO, AO, IsXLHSInRHSPart);
  if (!Res.first) {
    if (X.isGlobalReg()) {
      // Emit an update expression: 'xrval' binop 'expr' or 'expr' binop
      // 'xrval'.
      EmitStoreThroughLValue(CommonGen(EmitLoadOfLValue(X, Loc)), X);
    } else {
      // Perform compare-and-swap procedure.
      EmitAtomicUpdate(X, AO, CommonGen, X.getType().isVolatileQualified());
    }
  }
  return Res;
}

static void EmitOMPAtomicUpdateExpr(CodeGenFunction &CGF, bool IsSeqCst,
                                    const Expr *X, const Expr *E,
                                    const Expr *UE, bool IsXLHSInRHSPart,
                                    SourceLocation Loc) {
  assert(isa<BinaryOperator>(UE->IgnoreImpCasts()) &&
         "Update expr in 'atomic update' must be a binary operator.");
  auto *BOUE = cast<BinaryOperator>(UE->IgnoreImpCasts());
  // Update expressions are allowed to have the following forms:
  // x binop= expr; -> xrval + expr;
  // x++, ++x -> xrval + 1;
  // x--, --x -> xrval - 1;
  // x = x binop expr; -> xrval binop expr
  // x = expr Op x; - > expr binop xrval;
  assert(X->isLValue() && "X of 'omp atomic update' is not lvalue");
  LValue XLValue = CGF.EmitLValue(X);
  RValue ExprRValue = CGF.EmitAnyExpr(E);
  auto AO = IsSeqCst ? llvm::AtomicOrdering::SequentiallyConsistent
                     : llvm::AtomicOrdering::Monotonic;
  auto *LHS = cast<OpaqueValueExpr>(BOUE->getLHS()->IgnoreImpCasts());
  auto *RHS = cast<OpaqueValueExpr>(BOUE->getRHS()->IgnoreImpCasts());
  auto *XRValExpr = IsXLHSInRHSPart ? LHS : RHS;
  auto *ERValExpr = IsXLHSInRHSPart ? RHS : LHS;
  auto Gen =
      [&CGF, UE, ExprRValue, XRValExpr, ERValExpr](RValue XRValue) -> RValue {
        CodeGenFunction::OpaqueValueMapping MapExpr(CGF, ERValExpr, ExprRValue);
        CodeGenFunction::OpaqueValueMapping MapX(CGF, XRValExpr, XRValue);
        return CGF.EmitAnyExpr(UE);
      };
  (void)CGF.EmitOMPAtomicSimpleUpdateExpr(
      XLValue, ExprRValue, BOUE->getOpcode(), IsXLHSInRHSPart, AO, Loc, Gen);
  // OpenMP, 2.12.6, atomic Construct
  // Any atomic construct with a seq_cst clause forces the atomically
  // performed operation to include an implicit flush operation without a
  // list.
  if (IsSeqCst)
    CGF.CGM.getOpenMPRuntime().emitFlush(CGF, llvm::None, Loc);
}

static RValue convertToType(CodeGenFunction &CGF, RValue Value,
                            QualType SourceType, QualType ResType,
                            SourceLocation Loc) {
  switch (CGF.getEvaluationKind(ResType)) {
  case TEK_Scalar:
    return RValue::get(
        convertToScalarValue(CGF, Value, SourceType, ResType, Loc));
  case TEK_Complex: {
    auto Res = convertToComplexValue(CGF, Value, SourceType, ResType, Loc);
    return RValue::getComplex(Res.first, Res.second);
  }
  case TEK_Aggregate:
    break;
  }
  llvm_unreachable("Must be a scalar or complex.");
}

static void EmitOMPAtomicCaptureExpr(CodeGenFunction &CGF, bool IsSeqCst,
                                     bool IsPostfixUpdate, const Expr *V,
                                     const Expr *X, const Expr *E,
                                     const Expr *UE, bool IsXLHSInRHSPart,
                                     SourceLocation Loc) {
  assert(X->isLValue() && "X of 'omp atomic capture' is not lvalue");
  assert(V->isLValue() && "V of 'omp atomic capture' is not lvalue");
  RValue NewVVal;
  LValue VLValue = CGF.EmitLValue(V);
  LValue XLValue = CGF.EmitLValue(X);
  RValue ExprRValue = CGF.EmitAnyExpr(E);
  auto AO = IsSeqCst ? llvm::AtomicOrdering::SequentiallyConsistent
                     : llvm::AtomicOrdering::Monotonic;
  QualType NewVValType;
  if (UE) {
    // 'x' is updated with some additional value.
    assert(isa<BinaryOperator>(UE->IgnoreImpCasts()) &&
           "Update expr in 'atomic capture' must be a binary operator.");
    auto *BOUE = cast<BinaryOperator>(UE->IgnoreImpCasts());
    // Update expressions are allowed to have the following forms:
    // x binop= expr; -> xrval + expr;
    // x++, ++x -> xrval + 1;
    // x--, --x -> xrval - 1;
    // x = x binop expr; -> xrval binop expr
    // x = expr Op x; - > expr binop xrval;
    auto *LHS = cast<OpaqueValueExpr>(BOUE->getLHS()->IgnoreImpCasts());
    auto *RHS = cast<OpaqueValueExpr>(BOUE->getRHS()->IgnoreImpCasts());
    auto *XRValExpr = IsXLHSInRHSPart ? LHS : RHS;
    NewVValType = XRValExpr->getType();
    auto *ERValExpr = IsXLHSInRHSPart ? RHS : LHS;
    auto &&Gen = [&CGF, &NewVVal, UE, ExprRValue, XRValExpr, ERValExpr,
                  IsSeqCst, IsPostfixUpdate](RValue XRValue) -> RValue {
      CodeGenFunction::OpaqueValueMapping MapExpr(CGF, ERValExpr, ExprRValue);
      CodeGenFunction::OpaqueValueMapping MapX(CGF, XRValExpr, XRValue);
      RValue Res = CGF.EmitAnyExpr(UE);
      NewVVal = IsPostfixUpdate ? XRValue : Res;
      return Res;
    };
    auto Res = CGF.EmitOMPAtomicSimpleUpdateExpr(
        XLValue, ExprRValue, BOUE->getOpcode(), IsXLHSInRHSPart, AO, Loc, Gen);
    if (Res.first) {
      // 'atomicrmw' instruction was generated.
      if (IsPostfixUpdate) {
        // Use old value from 'atomicrmw'.
        NewVVal = Res.second;
      } else {
        // 'atomicrmw' does not provide new value, so evaluate it using old
        // value of 'x'.
        CodeGenFunction::OpaqueValueMapping MapExpr(CGF, ERValExpr, ExprRValue);
        CodeGenFunction::OpaqueValueMapping MapX(CGF, XRValExpr, Res.second);
        NewVVal = CGF.EmitAnyExpr(UE);
      }
    }
  } else {
    // 'x' is simply rewritten with some 'expr'.
    NewVValType = X->getType().getNonReferenceType();
    ExprRValue = convertToType(CGF, ExprRValue, E->getType(),
                               X->getType().getNonReferenceType(), Loc);
    auto &&Gen = [&CGF, &NewVVal, ExprRValue](RValue XRValue) -> RValue {
      NewVVal = XRValue;
      return ExprRValue;
    };
    // Try to perform atomicrmw xchg, otherwise simple exchange.
    auto Res = CGF.EmitOMPAtomicSimpleUpdateExpr(
        XLValue, ExprRValue, /*BO=*/BO_Assign, /*IsXLHSInRHSPart=*/false, AO,
        Loc, Gen);
    if (Res.first) {
      // 'atomicrmw' instruction was generated.
      NewVVal = IsPostfixUpdate ? Res.second : ExprRValue;
    }
  }
  // Emit post-update store to 'v' of old/new 'x' value.
  CGF.emitOMPSimpleStore(VLValue, NewVVal, NewVValType, Loc);
  // OpenMP, 2.12.6, atomic Construct
  // Any atomic construct with a seq_cst clause forces the atomically
  // performed operation to include an implicit flush operation without a
  // list.
  if (IsSeqCst)
    CGF.CGM.getOpenMPRuntime().emitFlush(CGF, llvm::None, Loc);
}

static void EmitOMPAtomicExpr(CodeGenFunction &CGF, OpenMPClauseKind Kind,
                              bool IsSeqCst, bool IsPostfixUpdate,
                              const Expr *X, const Expr *V, const Expr *E,
                              const Expr *UE, bool IsXLHSInRHSPart,
                              SourceLocation Loc) {
  switch (Kind) {
  case OMPC_read:
    EmitOMPAtomicReadExpr(CGF, IsSeqCst, X, V, Loc);
    break;
  case OMPC_write:
    EmitOMPAtomicWriteExpr(CGF, IsSeqCst, X, E, Loc);
    break;
  case OMPC_unknown:
  case OMPC_update:
    EmitOMPAtomicUpdateExpr(CGF, IsSeqCst, X, E, UE, IsXLHSInRHSPart, Loc);
    break;
  case OMPC_capture:
    EmitOMPAtomicCaptureExpr(CGF, IsSeqCst, IsPostfixUpdate, V, X, E, UE,
                             IsXLHSInRHSPart, Loc);
    break;
  case OMPC_if:
  case OMPC_final:
  case OMPC_num_threads:
  case OMPC_private:
  case OMPC_firstprivate:
  case OMPC_lastprivate:
  case OMPC_reduction:
  case OMPC_safelen:
  case OMPC_simdlen:
  case OMPC_collapse:
  case OMPC_default:
  case OMPC_check:
  case OMPC_use:
  case OMPC_seq_cst:
  case OMPC_shared:
  case OMPC_linear:
  case OMPC_aligned:
  case OMPC_copyin:
  case OMPC_copyprivate:
  case OMPC_flush:
  case OMPC_proc_bind:
  case OMPC_schedule:
  case OMPC_ordered:
  case OMPC_nowait:
  case OMPC_untied:
  case OMPC_threadprivate:
  case OMPC_depend:
  case OMPC_mergeable:
  case OMPC_device:
  case OMPC_threads:
  case OMPC_simd:
  case OMPC_map:
  case OMPC_num_teams:
  case OMPC_thread_limit:
  case OMPC_priority:
  case OMPC_grainsize:
  case OMPC_nogroup:
  case OMPC_num_tasks:
  case OMPC_hint:
  case OMPC_dist_schedule:
  case OMPC_defaultmap:
  case OMPC_uniform:
  case OMPC_to:
  case OMPC_from:
  case OMPC_use_device_ptr:
  case OMPC_is_device_ptr:
    llvm_unreachable("Clause is not allowed in 'omp atomic'.");
  }
}

void CodeGenFunction::EmitOMPAtomicDirective(const OMPAtomicDirective &S) {
  bool IsSeqCst = S.getSingleClause<OMPSeqCstClause>();
  OpenMPClauseKind Kind = OMPC_unknown;
  for (auto *C : S.clauses()) {
    // Find first clause (skip seq_cst clause, if it is first).
    if (C->getClauseKind() != OMPC_seq_cst) {
      Kind = C->getClauseKind();
      break;
    }
  }

  const auto *CS =
      S.getAssociatedStmt()->IgnoreContainers(/*IgnoreCaptured=*/true);
  if (const auto *EWC = dyn_cast<ExprWithCleanups>(CS)) {
    enterFullExpression(EWC);
  }
  // Processing for statements under 'atomic capture'.
  if (const auto *Compound = dyn_cast<CompoundStmt>(CS)) {
    for (const auto *C : Compound->body()) {
      if (const auto *EWC = dyn_cast<ExprWithCleanups>(C)) {
        enterFullExpression(EWC);
      }
    }
  }

  auto &&CodeGen = [&S, Kind, IsSeqCst, CS](CodeGenFunction &CGF,
                                            PrePostActionTy &) {
    CGF.EmitStopPoint(CS);
    EmitOMPAtomicExpr(CGF, Kind, IsSeqCst, S.isPostfixUpdate(), S.getX(),
                      S.getV(), S.getExpr(), S.getUpdateExpr(),
                      S.isXLHSInRHSPart(), S.getLocStart());
  };
  OMPLexicalScope Scope(*this, S, /*AsInlined=*/true);
  CGM.getOpenMPRuntime().emitInlinedDirective(*this, OMPD_atomic, CodeGen);
}

std::pair<llvm::Function * /*OutlinedFn*/, llvm::Constant * /*OutlinedFnID*/>
CodeGenFunction::EmitOMPTargetDirectiveOutlinedFunction(
    CodeGenModule &CGM, const OMPTargetDirective &S, StringRef ParentName,
    bool IsOffloadEntry) {
  llvm::Function *OutlinedFn = nullptr;
  llvm::Constant *OutlinedFnID = nullptr;
  auto &&CodeGen = [&S](CodeGenFunction &CGF, PrePostActionTy &Action) {
    OMPPrivateScope PrivateScope(CGF);
    (void)CGF.EmitOMPFirstprivateClause(S, PrivateScope);
    CGF.EmitOMPPrivateClause(S, PrivateScope);
    (void)PrivateScope.Privatize();

    Action.Enter(CGF);
    CGF.EmitStmt(cast<CapturedStmt>(S.getAssociatedStmt())->getCapturedStmt());
  };
  // Emit target region as a standalone region.
  CGM.getOpenMPRuntime().emitTargetOutlinedFunction(
      S, ParentName, OutlinedFn, OutlinedFnID, IsOffloadEntry, CodeGen);
  return std::make_pair(OutlinedFn, OutlinedFnID);
}

void CodeGenFunction::EmitOMPTargetDirective(const OMPTargetDirective &S) {
  const CapturedStmt &CS = *cast<CapturedStmt>(S.getAssociatedStmt());

  llvm::SmallVector<llvm::Value *, 16> CapturedVars;
  GenerateOpenMPCapturedVars(CS, CapturedVars);

  llvm::Function *Fn = nullptr;
  llvm::Constant *FnID = nullptr;

  // Check if we have any if clause associated with the directive.
  const Expr *IfCond = nullptr;

  if (auto *C = S.getSingleClause<OMPIfClause>()) {
    IfCond = C->getCondition();
  }

  // Check if we have any device clause associated with the directive.
  const Expr *Device = nullptr;
  if (auto *C = S.getSingleClause<OMPDeviceClause>()) {
    Device = C->getDevice();
  }

  // Check if we have an if clause whose conditional always evaluates to false
  // or if we do not have any targets specified. If so the target region is not
  // an offload entry point.
  bool IsOffloadEntry = true;
  if (IfCond) {
    bool Val;
    if (ConstantFoldsToSimpleInteger(IfCond, Val) && !Val)
      IsOffloadEntry = false;
  }
  if (CGM.getLangOpts().OMPTargetTriples.empty())
    IsOffloadEntry = false;

  assert(CurFuncDecl && "No parent declaration for target region!");
  StringRef ParentName;
  // In case we have Ctors/Dtors we use the complete type variant to produce
  // the mangling of the device outlined kernel.
  if (auto *D = dyn_cast<CXXConstructorDecl>(CurFuncDecl))
    ParentName = CGM.getMangledName(GlobalDecl(D, Ctor_Complete));
  else if (auto *D = dyn_cast<CXXDestructorDecl>(CurFuncDecl))
    ParentName = CGM.getMangledName(GlobalDecl(D, Dtor_Complete));
  else
    ParentName =
        CGM.getMangledName(GlobalDecl(cast<FunctionDecl>(CurFuncDecl)));

  std::tie(Fn, FnID) = EmitOMPTargetDirectiveOutlinedFunction(
      CGM, S, ParentName, IsOffloadEntry);
  OMPLexicalScope Scope(*this, S);
  CGM.getOpenMPRuntime().emitTargetCall(*this, S, Fn, FnID, IfCond, Device,
                                        CapturedVars);
}

static void emitCommonOMPTeamsDirective(CodeGenFunction &CGF,
                                        const OMPExecutableDirective &S,
                                        OpenMPDirectiveKind InnermostKind,
                                        const RegionCodeGenTy &CodeGen) {
  auto CS = cast<CapturedStmt>(S.getAssociatedStmt());
  auto OutlinedFn = CGF.CGM.getOpenMPRuntime().
      emitParallelOrTeamsOutlinedFunction(S,
          *CS->getCapturedDecl()->param_begin(), InnermostKind, CodeGen);

  const OMPTeamsDirective &TD = *dyn_cast<OMPTeamsDirective>(&S);
  const OMPNumTeamsClause *NT = TD.getSingleClause<OMPNumTeamsClause>();
  const OMPThreadLimitClause *TL = TD.getSingleClause<OMPThreadLimitClause>();
  if (NT || TL) {
    Expr *NumTeams = (NT) ? NT->getNumTeams() : nullptr;
    Expr *ThreadLimit = (TL) ? TL->getThreadLimit() : nullptr;

    CGF.CGM.getOpenMPRuntime().emitNumTeamsClause(CGF, NumTeams, ThreadLimit,
                                                  S.getLocStart());
  }

  OMPLexicalScope Scope(CGF, S);
  llvm::SmallVector<llvm::Value *, 16> CapturedVars;
  CGF.GenerateOpenMPCapturedVars(*CS, CapturedVars);
  CGF.CGM.getOpenMPRuntime().emitTeamsCall(CGF, S, S.getLocStart(), OutlinedFn,
                                           CapturedVars);
}

void CodeGenFunction::EmitOMPTeamsDirective(const OMPTeamsDirective &S) {
  // Emit parallel region as a standalone region.
  auto &&CodeGen = [&S](CodeGenFunction &CGF, PrePostActionTy &) {
    OMPPrivateScope PrivateScope(CGF);
    (void)CGF.EmitOMPFirstprivateClause(S, PrivateScope);
    CGF.EmitOMPPrivateClause(S, PrivateScope);
    (void)PrivateScope.Privatize();
    CGF.EmitStmt(cast<CapturedStmt>(S.getAssociatedStmt())->getCapturedStmt());
  };
  emitCommonOMPTeamsDirective(*this, S, OMPD_teams, CodeGen);
}

void CodeGenFunction::EmitOMPCancellationPointDirective(
    const OMPCancellationPointDirective &S) {
  CGM.getOpenMPRuntime().emitCancellationPointCall(*this, S.getLocStart(),
                                                   S.getCancelRegion());
}

void CodeGenFunction::EmitOMPCancelDirective(const OMPCancelDirective &S) {
  const Expr *IfCond = nullptr;
  for (const auto *C : S.getClausesOfKind<OMPIfClause>()) {
    if (C->getNameModifier() == OMPD_unknown ||
        C->getNameModifier() == OMPD_cancel) {
      IfCond = C->getCondition();
      break;
    }
  }
  CGM.getOpenMPRuntime().emitCancelCall(*this, S.getLocStart(), IfCond,
                                        S.getCancelRegion());
}

CodeGenFunction::JumpDest
CodeGenFunction::getOMPCancelDestination(OpenMPDirectiveKind Kind) {
  if (Kind == OMPD_parallel || Kind == OMPD_task)
    return ReturnBlock;
  assert(Kind == OMPD_for || Kind == OMPD_section || Kind == OMPD_sections ||
         Kind == OMPD_parallel_sections || Kind == OMPD_parallel_for);
  return BreakContinueStack.back().BreakBlock;
}

// Generate the instructions for '#pragma omp target data' directive.
void CodeGenFunction::EmitOMPTargetDataDirective(
    const OMPTargetDataDirective &S) {
  // The target data enclosed region is implemented just by emitting the
  // statement.
  auto &&CodeGen = [&S](CodeGenFunction &CGF, PrePostActionTy &) {
    CGF.EmitStmt(cast<CapturedStmt>(S.getAssociatedStmt())->getCapturedStmt());
  };

  // If we don't have target devices, don't bother emitting the data mapping
  // code.
  if (CGM.getLangOpts().OMPTargetTriples.empty()) {
    OMPLexicalScope Scope(*this, S, /*AsInlined=*/true);

    CGM.getOpenMPRuntime().emitInlinedDirective(*this, OMPD_target_data,
                                                CodeGen);
    return;
  }

  // Check if we have any if clause associated with the directive.
  const Expr *IfCond = nullptr;
  if (auto *C = S.getSingleClause<OMPIfClause>())
    IfCond = C->getCondition();

  // Check if we have any device clause associated with the directive.
  const Expr *Device = nullptr;
  if (auto *C = S.getSingleClause<OMPDeviceClause>())
    Device = C->getDevice();

  CGM.getOpenMPRuntime().emitTargetDataCalls(*this, S, IfCond, Device, CodeGen);
}

void CodeGenFunction::EmitOMPTargetEnterDataDirective(
    const OMPTargetEnterDataDirective &S) {
  // If we don't have target devices, don't bother emitting the data mapping
  // code.
  if (CGM.getLangOpts().OMPTargetTriples.empty())
    return;

  // Check if we have any if clause associated with the directive.
  const Expr *IfCond = nullptr;
  if (auto *C = S.getSingleClause<OMPIfClause>())
    IfCond = C->getCondition();

  // Check if we have any device clause associated with the directive.
  const Expr *Device = nullptr;
  if (auto *C = S.getSingleClause<OMPDeviceClause>())
    Device = C->getDevice();

  CGM.getOpenMPRuntime().emitTargetDataStandAloneCall(*this, S, IfCond, Device);
}

void CodeGenFunction::EmitOMPTargetExitDataDirective(
    const OMPTargetExitDataDirective &S) {
  // If we don't have target devices, don't bother emitting the data mapping
  // code.
  if (CGM.getLangOpts().OMPTargetTriples.empty())
    return;

  // Check if we have any if clause associated with the directive.
  const Expr *IfCond = nullptr;
  if (auto *C = S.getSingleClause<OMPIfClause>())
    IfCond = C->getCondition();

  // Check if we have any device clause associated with the directive.
  const Expr *Device = nullptr;
  if (auto *C = S.getSingleClause<OMPDeviceClause>())
    Device = C->getDevice();

  CGM.getOpenMPRuntime().emitTargetDataStandAloneCall(*this, S, IfCond, Device);
}

void CodeGenFunction::EmitOMPTargetParallelDirective(
    const OMPTargetParallelDirective &S) {
  // TODO: codegen for target parallel.
}

void CodeGenFunction::EmitOMPTargetParallelForDirective(
    const OMPTargetParallelForDirective &S) {
  // TODO: codegen for target parallel for.
}

/// Emit a helper variable and return corresponding lvalue.
static void mapParam(CodeGenFunction &CGF, const DeclRefExpr *Helper,
                     const ImplicitParamDecl *PVD,
                     CodeGenFunction::OMPPrivateScope &Privates) {
  auto *VDecl = cast<VarDecl>(Helper->getDecl());
  Privates.addPrivate(
      VDecl, [&CGF, PVD]() -> Address { return CGF.GetAddrOfLocalVar(PVD); });
}

void CodeGenFunction::EmitOMPTaskLoopBasedDirective(const OMPLoopDirective &S) {
  assert(isOpenMPTaskLoopDirective(S.getDirectiveKind()));
  // Emit outlined function for task construct.
  auto CS = cast<CapturedStmt>(S.getAssociatedStmt());
  auto CapturedStruct = GenerateCapturedStmtArgument(*CS);
  auto SharedsTy = getContext().getRecordType(CS->getCapturedRecordDecl());
  const Expr *IfCond = nullptr;
  for (const auto *C : S.getClausesOfKind<OMPIfClause>()) {
    if (C->getNameModifier() == OMPD_unknown ||
        C->getNameModifier() == OMPD_taskloop) {
      IfCond = C->getCondition();
      break;
    }
  }

  OMPTaskDataTy Data;
  // Check if taskloop must be emitted without taskgroup.
  Data.Nogroup = S.getSingleClause<OMPNogroupClause>();
  // TODO: Check if we should emit tied or untied task.
  Data.Tied = true;
  // Set scheduling for taskloop
  if (const auto* Clause = S.getSingleClause<OMPGrainsizeClause>()) {
    // grainsize clause
    Data.Schedule.setInt(/*IntVal=*/false);
    Data.Schedule.setPointer(EmitScalarExpr(Clause->getGrainsize()));
  } else if (const auto* Clause = S.getSingleClause<OMPNumTasksClause>()) {
    // num_tasks clause
    Data.Schedule.setInt(/*IntVal=*/true);
    Data.Schedule.setPointer(EmitScalarExpr(Clause->getNumTasks()));
  }

  auto &&BodyGen = [CS, &S](CodeGenFunction &CGF, PrePostActionTy &) {
    // if (PreCond) {
    //   for (IV in 0..LastIteration) BODY;
    //   <Final counter/linear vars updates>;
    // }
    //

    // Emit: if (PreCond) - begin.
    // If the condition constant folds and can be elided, avoid emitting the
    // whole loop.
    bool CondConstant;
    llvm::BasicBlock *ContBlock = nullptr;
    OMPLoopScope PreInitScope(CGF, S);
    if (CGF.ConstantFoldsToSimpleInteger(S.getPreCond(), CondConstant)) {
      if (!CondConstant)
        return;
    } else {
      auto *ThenBlock = CGF.createBasicBlock("taskloop.if.then");
      ContBlock = CGF.createBasicBlock("taskloop.if.end");
      emitPreCond(CGF, S, S.getPreCond(), ThenBlock, ContBlock,
                  CGF.getProfileCount(&S));
      CGF.EmitBlock(ThenBlock);
      CGF.incrementProfileCounter(&S);
    }

    if (isOpenMPSimdDirective(S.getDirectiveKind()))
      CGF.EmitOMPSimdInit(S);

    OMPPrivateScope LoopScope(CGF);
    // Emit helper vars inits.
    enum { LowerBound = 5, UpperBound, Stride, LastIter };
    auto *I = CS->getCapturedDecl()->param_begin();
    auto *LBP = std::next(I, LowerBound);
    auto *UBP = std::next(I, UpperBound);
    auto *STP = std::next(I, Stride);
    auto *LIP = std::next(I, LastIter);
    mapParam(CGF, cast<DeclRefExpr>(S.getLowerBoundVariable()), *LBP,
             LoopScope);
    mapParam(CGF, cast<DeclRefExpr>(S.getUpperBoundVariable()), *UBP,
             LoopScope);
    mapParam(CGF, cast<DeclRefExpr>(S.getStrideVariable()), *STP, LoopScope);
    mapParam(CGF, cast<DeclRefExpr>(S.getIsLastIterVariable()), *LIP,
             LoopScope);
    CGF.EmitOMPPrivateLoopCounters(S, LoopScope);
    bool HasLastprivateClause = CGF.EmitOMPLastprivateClauseInit(S, LoopScope);
    (void)LoopScope.Privatize();
    // Emit the loop iteration variable.
    const Expr *IVExpr = S.getIterationVariable();
    const VarDecl *IVDecl = cast<VarDecl>(cast<DeclRefExpr>(IVExpr)->getDecl());
    CGF.EmitVarDecl(*IVDecl);
    CGF.EmitIgnoredExpr(S.getInit());

    // Emit the iterations count variable.
    // If it is not a variable, Sema decided to calculate iterations count on
    // each iteration (e.g., it is foldable into a constant).
    if (auto LIExpr = dyn_cast<DeclRefExpr>(S.getLastIteration())) {
      CGF.EmitVarDecl(*cast<VarDecl>(LIExpr->getDecl()));
      // Emit calculation of the iterations count.
      CGF.EmitIgnoredExpr(S.getCalcLastIteration());
    }

    CGF.EmitOMPInnerLoop(S, LoopScope.requiresCleanups(), S.getCond(),
                         S.getInc(),
                         [&S](CodeGenFunction &CGF) {
                           CGF.EmitOMPLoopBody(S, JumpDest());
                           CGF.EmitStopPoint(&S);
                         },
                         [](CodeGenFunction &) {});
    // Emit: if (PreCond) - end.
    if (ContBlock) {
      CGF.EmitBranch(ContBlock);
      CGF.EmitBlock(ContBlock, true);
    }
    // Emit final copy of the lastprivate variables if IsLastIter != 0.
    if (HasLastprivateClause) {
      CGF.EmitOMPLastprivateClauseFinal(
          S, isOpenMPSimdDirective(S.getDirectiveKind()),
          CGF.Builder.CreateIsNotNull(CGF.EmitLoadOfScalar(
              CGF.GetAddrOfLocalVar(*LIP), /*Volatile=*/false,
              (*LIP)->getType(), S.getLocStart())));
    }
  };
  auto &&TaskGen = [&S, SharedsTy, CapturedStruct,
                    IfCond](CodeGenFunction &CGF, llvm::Value *OutlinedFn,
                            const OMPTaskDataTy &Data) {
    auto &&CodeGen = [&](CodeGenFunction &CGF, PrePostActionTy &) {
      OMPLoopScope PreInitScope(CGF, S);
      CGF.CGM.getOpenMPRuntime().emitTaskLoopCall(CGF, S.getLocStart(), S,
                                                  OutlinedFn, SharedsTy,
                                                  CapturedStruct, IfCond, Data);
    };
    CGF.CGM.getOpenMPRuntime().emitInlinedDirective(CGF, OMPD_taskloop,
                                                    CodeGen);
  };
  EmitOMPTaskBasedDirective(S, BodyGen, TaskGen, Data);
}



static
OMPTaskLoopDirective*
emitOMPTaskLoopWithUseTLS(CodeGenFunction& CGF, CodeGenModule& CGM,
    const OMPTaskLoopDirective& S, const OMPUseClause* useClause) {

  ASTContext& context = CGM.getContext();

  IdentifierTable *idt2 = new IdentifierTable(CGM.getLangOpts());

  //const IntegerLiteral* numForLoopsExpr = static_cast<const IntegerLiteral*>(orderedClause->getNumForLoops());
  //const unsigned int num_loops = numForLoopsExpr->getValue().getZExtValue();

  llvm::errs() << "tls\n";//<< num_loops <<")\n";

  FunctionArgList TemplateArgsCond, TemplateArgs1, TemplateArgs2,TemplateArgsCreate, TemplateArgsSetM;

  // Get the original loop body, associated with the 'parallel for' directive
  Stmt *Body = S.getAssociatedStmt()->IgnoreContainers(true);

  Body = dyn_cast<ForStmt>(Body)->getBody();

  // Call the function to separate the body into loop components, based on the 'ordered' directive
  visitBody2(Body);
  checkDeclRefExpr2(Body);

  S.dumpPretty(context);

  int num_components = blocos.size();

  llvm::errs() << "numero componentes "<< num_components <<"\n";

  if(num_components <= 0) {
    llvm::errs() << "error: number of loops must be a strictly positive integer!\n";
    return nullptr;
  }

  //VarDecl *vdPost;
  //QualType qt = context.getConstantArrayType(context.IntTy,
  //    llvm::APInt(context.getTypeSize(context.IntTy), (uint64_t)
  //      num_components), ArrayType::ArraySizeModifier::Normal, 0);
  //IdentifierInfo* iiFlags = &(idt2->get("__tls_flags"));
  //DeclarationName *dnFlags = new DeclarationName(iiFlags);
  //DeclarationNameInfo *dniFlags = new DeclarationNameInfo(*dnFlags,
  //    SourceLocation());
  //vdPost = VarDecl::Create(context, declCTX, SourceLocation(),
  //    SourceLocation(), iiFlags, qt, nullptr, StorageClass::SC_None);


  // Create the identifier for using the condition function call
  IdentifierInfo* iiParam = &(idt2->get("tls_stage_param"));
  ParmVarDecl *stageFlag = ParmVarDecl::Create(context, declCTX,
      SourceLocation(), SourceLocation(), iiParam, context.IntTy, nullptr,
      StorageClass::SC_None, nullptr);
  TemplateArgsCond.push_back(stageFlag);


  IdentifierInfo* iiParam3 = &(idt2->get("cpu"));
  ParmVarDecl *stageFlag3 = ParmVarDecl::Create(context, declCTX,
      SourceLocation(), SourceLocation(), iiParam3, context.IntTy, nullptr,
      StorageClass::SC_None, nullptr);
  TemplateArgsSetM.push_back(stageFlag3);

  IdentifierInfo* iiParam2 = &(idt2->get("mask"));
  ParmVarDecl *stageFlag2 = ParmVarDecl::Create(context, declCTX,
      SourceLocation(), SourceLocation(), iiParam2, context.VoidPtrTy,
      nullptr, StorageClass::SC_None, nullptr);
  TemplateArgsCreate.push_back(stageFlag2);
  TemplateArgsSetM.push_back(stageFlag2);

  // Create the identifier for using the condition function call
  IdentifierInfo* iiParam4 = &(idt2->get("par_abort"));
  ParmVarDecl *stageFlag4 = ParmVarDecl::Create(context, declCTX,
      SourceLocation(), SourceLocation(), iiParam4, context.CharTy, nullptr,
      StorageClass::SC_None, nullptr);
  TemplateArgs2.push_back(stageFlag4);

  // Create a new identifiers table based on the current context
  IdentifierTable *idt = new IdentifierTable(CGM.getLangOpts());
  NestedNameSpecifierLoc *nnsl = new NestedNameSpecifierLoc();

  // Create the identifier for using the condition function call
  IdentifierInfo* iiCond = &(idt->get("__tls_cond__"));
  DeclarationName *dnCond = new DeclarationName(iiCond);
  //DeclarationNameInfo *dniCond = new DeclarationNameInfo(*dnCond, SourceLocation());

  // Create the identifier for finding the start of the component
  IdentifierInfo* iiBegin = &(idt->get("_xbegin"));
  DeclarationName *dnBegin = new DeclarationName(iiBegin);
  DeclarationNameInfo *dniBegin = new DeclarationNameInfo(*dnBegin, SourceLocation());

  // Create the identifier for finding the end of the component
  IdentifierInfo* iiEnd = &(idt->get("_xend"));
  DeclarationName *dnEnd = new DeclarationName(iiEnd);
  DeclarationNameInfo *dniEnd = new DeclarationNameInfo(*dnEnd, SourceLocation());

  IdentifierInfo* iiCreate = &(idt->get("kmp_create_affinity_mask"));
  DeclarationName *dnCreate = new DeclarationName(iiCreate);
  //DeclarationNameInfo *dniCreate = new DeclarationNameInfo(*dnCreate, SourceLocation());

  IdentifierInfo* iiSetM = &(idt->get("kmp_set_affinity_mask_proc"));
  DeclarationName *dnSetM = new DeclarationName(iiSetM);
  //DeclarationNameInfo *dniSetM = new DeclarationNameInfo(*dnSetM, SourceLocation());

  IdentifierInfo* iiSet = &(idt->get("kmp_set_affinity"));
  DeclarationName *dnSet = new DeclarationName(iiSet);
  //DeclarationNameInfo *dniSet = new DeclarationNameInfo(*dnSet, SourceLocation());

  // Create the 'FunctionDecl' nodes for inserting on the AST
  std::vector<QualType> argsCond, args1,args2, argsCreate, argsSetM, argsSet;
  FunctionProtoType::ExtProtoInfo fpi;
  fpi.Variadic = false;

  argsCond.push_back(context.IntTy);
  args2.push_back(context.CharTy);

  argsCreate.push_back(context.VoidPtrTy);

  argsSetM.push_back(context.IntTy);
  argsSetM.push_back(context.VoidPtrTy);

  argsSet.push_back(context.VoidPtrTy);

  QualType funcTypeCond = context.getFunctionType(context.IntTy, argsCond, fpi);
  QualType funcType1 = context.getFunctionType(context.UnsignedIntTy,args1, fpi);
  QualType funcType2 = context.getFunctionType(context.VoidTy, args1, fpi); //xend
  QualType funcType3 = context.getFunctionType(context.VoidTy, args2, fpi); //xabort

  QualType funcTypeCreate = context.getFunctionType(context.VoidTy, argsCreate, fpi);

  QualType funcTypeSetM = context.getFunctionType(context.VoidTy, argsSetM, fpi);

  QualType funcTypeSet = context.getFunctionType(context.VoidTy, argsSet, fpi);

  LinkageSpecDecl* lsd = LinkageSpecDecl::Create(context,
      context.getTranslationUnitDecl(), SourceLocation(), SourceLocation(),
      LinkageSpecDecl::lang_c, false);

  FunctionDecl *FDCond = FunctionDecl::Create(context, lsd,
      SourceLocation(), SourceLocation(), *dnCond, funcTypeCond, nullptr,
      StorageClass::SC_None);

  FunctionDecl *FDBegin = FunctionDecl::Create(context, lsd,
      SourceLocation(), SourceLocation(), *dnBegin, funcType1, nullptr,
      StorageClass::SC_Static);
  FunctionDecl *FDEnd = FunctionDecl::Create(context, lsd, SourceLocation(),
      SourceLocation(), *dnEnd, funcType2, nullptr, StorageClass::SC_Static);
  FunctionDecl *FDAbort;

  IdentifierInfo* iiAbort = &(idt->get("_xabort2"));
  DeclarationName *dnAbort = new DeclarationName(iiAbort);
  DeclarationNameInfo *dniAbort = new DeclarationNameInfo(*dnAbort, SourceLocation());

  FDAbort = FunctionDecl::Create(context, lsd, SourceLocation(),
      SourceLocation(), *dnAbort, funcType3, nullptr, StorageClass::SC_None);

  FunctionDecl *FDCreate = FunctionDecl::Create(context, lsd,
      SourceLocation(), SourceLocation(), *dnCreate, funcTypeCreate, nullptr,
      StorageClass::SC_None);

  FunctionDecl *FDSetM = FunctionDecl::Create(context, lsd,
      SourceLocation(), SourceLocation(), *dnSetM, funcTypeSetM, nullptr,
      StorageClass::SC_None);

  FunctionDecl *FDSet = FunctionDecl::Create(context, lsd,
      SourceLocation(), SourceLocation(), *dnSet, funcTypeSet, nullptr,
      StorageClass::SC_None);

  std::vector<ParmVarDecl *>
    params,params1,params2,paramsCreate,paramsSetM,paramsSet;
  params.push_back(stageFlag);
  params2.push_back(stageFlag4);

  paramsCreate.push_back(stageFlag2);

  paramsSetM.push_back(stageFlag3);
  paramsSetM.push_back(stageFlag2);

  paramsSet.push_back(stageFlag2);

  FDCond->setParams(params);
  FDAbort->setParams(params2);
  FDCreate->setParams(paramsCreate);
  FDSetM->setParams(paramsSetM);
  FDSet->setParams(paramsSet);

  // Auxiliary vector
  std::vector<Stmt *> ifStmts;

  std::vector<Expr *> ArgsCond, Args1 , Args2, ArgsCreate, ArgsSetM, ArgsSet;

  Stmt *forInit =
    dyn_cast<ForStmt>(dyn_cast<CapturedStmt>(S.getAssociatedStmt())->getCapturedStmt())->getInit();
  Expr *forCond =
    cast<ForStmt>(cast<CapturedStmt>(S.getAssociatedStmt())->getCapturedStmt())->getCond();

  Expr *iterations = dyn_cast<BinaryOperator>(forCond)->getRHS();

  BinaryOperator *boInit = dyn_cast<BinaryOperator>(forInit);

  VarDecl *forCondVar =
    dyn_cast<VarDecl>(dyn_cast<DeclRefExpr>(boInit->getLHS())->getDecl());
  Expr *forCondVal = boInit->getRHS();

  IdentifierInfo* iiforCondVar = &(idt->get(forCondVar->getNameAsString()));
  DeclarationName *dnforCondVar = new DeclarationName(iiforCondVar);
  DeclarationNameInfo *dniforCondVar = new DeclarationNameInfo(*dnforCondVar,
      SourceLocation());

  Expr *dreCondVar = DeclRefExpr::Create(context, NestedNameSpecifierLoc(),
      SourceLocation(), forCondVar, false, *dniforCondVar,
      forCondVar->getType(), clang::ExprValueKind::VK_RValue);
  ImplicitCastExpr *iceCondVar = ImplicitCastExpr::Create(context,
      forCondVar->getType(), CastKind::CK_LValueToRValue, dreCondVar,
      nullptr, ExprValueKind::VK_RValue);

  std::vector<Stmt *> InitArray;
  std::vector<VarDecl *> tempVars;

  Expr* chunk;
  if(useClause->getChunkSize()) {
    chunk = const_cast<Expr *>(useClause->getChunkSize());
    if(chunk->getType() != forCondVar->getType()) {
      chunk = ImplicitCastExpr::Create(context, forCondVar->getType(),
          CastKind::CK_IntegralCast, chunk, nullptr, ExprValueKind::VK_RValue);
    }
  } else {
    chunk = IntegerLiteral::Create(context,
        llvm::APInt(context.getTypeSize(forCondVar->getType()), (uint64_t) 1),
        forCondVar->getType(), SourceLocation());
  }

  Expr *dreNextVar ;

  //Expr *lhs, *rhs;
  int i_component = 0;
  for(auto it = blocos.begin();it != blocos.end();it++) {

    std::vector<Stmt *> forStmts;
    //forStmts.push_back(ceBegin);
    for(auto it2 = (*it).begin(); it2 != (*it).end(); it2++) forStmts.push_back(*it2);
    //forStmts.push_back(ceEnd);

    Stmt *forCompS = new (context) CompoundStmt(context, forStmts, SourceLocation(), SourceLocation());

    IdentifierInfo* iiVar = &(idt->get("tls_inner_iterator"));
    VarDecl *condVar = VarDecl::Create(context, declCTX, SourceLocation(),
        SourceLocation(), iiVar, forCondVar->getType(), nullptr,
        StorageClass::SC_None);

    condVar->setInit(iceCondVar);

    DeclStmt *declStmt = new (context) DeclStmt(DeclGroupRef(condVar),
        SourceLocation(), SourceLocation());

    Expr *dreCondVar2 = DeclRefExpr::Create(context,
        NestedNameSpecifierLoc(), SourceLocation(), condVar, false,
        *dniforCondVar, condVar->getType(), clang::ExprValueKind::VK_RValue);
    Expr *dreCondVar3 = DeclRefExpr::Create(context,
        NestedNameSpecifierLoc(), SourceLocation(), forCondVar, false,
        *dniforCondVar, forCondVar->getType(),
        clang::ExprValueKind::VK_RValue);
    BinaryOperator *boforCondAdd = new (context) BinaryOperator(dreCondVar3,
        chunk, BinaryOperatorKind::BO_Add, forCondVar->getType(),
        clang::ExprValueKind::VK_RValue, ExprObjectKind::OK_Ordinary,
        SourceLocation(), false);

    BinaryOperator *boforCondAnt = new (context) BinaryOperator(dreCondVar2,
        iterations, BinaryOperatorKind::BO_LT,forCondVar->getType(),
        clang::ExprValueKind::VK_RValue, ExprObjectKind::OK_Ordinary,
        SourceLocation(), false);


    BinaryOperator *boforCond = new (context) BinaryOperator(dreCondVar2,
        boforCondAdd, BinaryOperatorKind::BO_LT, forCondVar->getType(),
        clang::ExprValueKind::VK_RValue, ExprObjectKind::OK_Ordinary,
        SourceLocation(), false);

    BinaryOperator *boforCond2 = new (context) BinaryOperator(boforCond,
        boforCondAnt, BinaryOperatorKind::BO_LAnd, forCondVar->getType(),
        clang::ExprValueKind::VK_RValue, ExprObjectKind::OK_Ordinary,
        SourceLocation(), false);

    Expr *dreCondVar4 = DeclRefExpr::Create(context,
        NestedNameSpecifierLoc(), SourceLocation(), condVar, false,
        *dniforCondVar, forCondVar->getType(),
        clang::ExprValueKind::VK_RValue);
    UnaryOperator *uoforInc = new (context) UnaryOperator(dreCondVar4,
        UnaryOperatorKind::UO_PostInc, forCondVar->getType(),
        clang::ExprValueKind::VK_RValue, ExprObjectKind::OK_Ordinary,
        SourceLocation());

    ForStmt *forStmt = new (context) ForStmt (context, declStmt, boforCond2,
        condVar, uoforInc, forCompS, SourceLocation(), SourceLocation(),
        SourceLocation());

    ReplaceVarDecl(forCompS, forCondVar, condVar);

    // Build __bdx_flags[0]
    //lhs = DeclRefExpr::Create(context, NestedNameSpecifierLoc(),
    //    SourceLocation(), vdPost, false, *dniFlags, qt,
    //    clang::ExprValueKind::VK_RValue,
    //    dyn_cast<NamedDecl>(vdPost->getCanonicalDecl()));
    //rhs = IntegerLiteral::Create(context,
    //    llvm::APInt(context.getTypeSize(context.IntTy), (uint64_t)
    //      i_component), context.IntTy, SourceLocation());
    //ImplicitCastExpr *iceFlags = ImplicitCastExpr::Create(context,
    //    context.getPointerType(qt), CastKind::CK_ArrayToPointerDecay, lhs,
    //    nullptr, ExprValueKind::VK_RValue);
    //ArraySubscriptExpr *asexpr = new (context) ArraySubscriptExpr(iceFlags,
    //    rhs, context.IntTy, clang::ExprValueKind::VK_RValue,
    //    clang::ExprObjectKind::OK_Ordinary, SourceLocation());

    IdentifierInfo*iiExecute = &(idt->get("execute_in_htm"));
    VarDecl * executeVar = VarDecl::Create(context, declCTX,
        SourceLocation(), SourceLocation(), iiExecute, context.IntTy,
        nullptr, StorageClass::SC_None);

    DeclarationName *dnExecuteVar = new DeclarationName(iiExecute);
    DeclarationNameInfo *dniExecuteVar = new
      DeclarationNameInfo(*dnExecuteVar, SourceLocation());

    Expr *dreExecuteVar = DeclRefExpr::Create(context,
        NestedNameSpecifierLoc(), SourceLocation(), executeVar, false,
        *dniExecuteVar, executeVar->getType(),
        clang::ExprValueKind::VK_RValue);
    //ImplicitCastExpr *iceExecuteVar = ImplicitCastExpr::Create(context,
    //    executeVar->getType(), CastKind::CK_LValueToRValue, dreExecuteVar,
    //    nullptr, ExprValueKind::VK_RValue);


    DeclStmt *declStmt2 = new (context) DeclStmt(DeclGroupRef(executeVar),
        SourceLocation(), SourceLocation());

    IdentifierInfo*iiRetry = &(idt->get("Retry"));
    LabelDecl * RetryLab = LabelDecl::Create(context, declCTX,
        SourceLocation(),  iiRetry);

    //DeclarationName *dnRetryLab = new DeclarationName(iiRetry);
    //DeclarationNameInfo *dniRetryLab = new DeclarationNameInfo(*dnRetryLab,
    //    SourceLocation());

    IdentifierInfo*iiStatus = &(idt->get("status"));
    VarDecl * statusVar = VarDecl::Create(context, declCTX, SourceLocation(),
        SourceLocation(), iiStatus, context.UnsignedIntTy, nullptr,
        StorageClass::SC_None);

    DeclarationName *dnStatusVar = new DeclarationName(iiStatus);
    DeclarationNameInfo *dniStatusVar = new DeclarationNameInfo(*dnStatusVar,
        SourceLocation());

    Expr *dreStatusVar = DeclRefExpr::Create(context,
        NestedNameSpecifierLoc(), SourceLocation(), statusVar, false,
        *dniStatusVar, statusVar->getType(),
        clang::ExprValueKind::VK_RValue);
    //ImplicitCastExpr *iceStatusVar = ImplicitCastExpr::Create(context,
    //    statusVar->getType(), CastKind::CK_LValueToRValue, dreStatusVar,
    //    nullptr, ExprValueKind::VK_RValue);


    DeclStmt *declStmt3 = new (context) DeclStmt(DeclGroupRef(statusVar),
        SourceLocation(), SourceLocation());


    IdentifierInfo*iiNext = &(idt->get("next"));
    VarDecl * nextVar = VarDecl::Create(context, declCTX, SourceLocation(),
        SourceLocation(), iiNext,
        context.getVolatileType(forCondVar->getType()), nullptr,
        StorageClass::SC_None);

    DeclarationName *dnNextVar = new DeclarationName(iiNext);
    DeclarationNameInfo *dniNextVar = new DeclarationNameInfo(*dnNextVar, SourceLocation());

    dreNextVar = DeclRefExpr::Create(context, NestedNameSpecifierLoc(),
        SourceLocation(), nextVar, false, *dniNextVar, nextVar->getType(),
        clang::ExprValueKind::VK_RValue);
    //ImplicitCastExpr *iceNextVar = ImplicitCastExpr::Create(context,
    //    nextVar->getType(), CastKind::CK_LValueToRValue, dreNextVar, nullptr,
    //    ExprValueKind::VK_RValue);

    BinaryOperator *boInitVar = new (context)
      BinaryOperator(dreNextVar,forCondVal , BinaryOperatorKind::BO_Assign,
          forCondVar->getType(), clang::ExprValueKind::VK_RValue,
          ExprObjectKind::OK_Ordinary, SourceLocation(), false);

    CGF.EmitBinaryOperatorLValue(boInitVar);

    BinaryOperator *boAssign1 = new (context) BinaryOperator(dreExecuteVar,
        IntegerLiteral::Create(context,
          llvm::APInt(context.getTypeSize(context.IntTy), (uint64_t) 0),
          context.IntTy, SourceLocation()) , BinaryOperatorKind::BO_Assign,
        context.IntTy, clang::ExprValueKind::VK_RValue,
        ExprObjectKind::OK_Ordinary, SourceLocation(), false);

    LabelStmt *labelStmt = new (context) LabelStmt(SourceLocation(), RetryLab, boAssign1);

    BinaryOperator *boNE = new (context) BinaryOperator(dreNextVar,
        dreCondVar, BinaryOperatorKind::BO_NE, context.IntTy,
        clang::ExprValueKind::VK_RValue, ExprObjectKind::OK_Ordinary,
        SourceLocation(), false);


    IdentifierInfo*iiMask = &(idt->get("mask"));
    VarDecl * maskVar = VarDecl::Create(context, declCTX, SourceLocation(),
        SourceLocation(), iiMask, context.VoidPtrTy, nullptr,
        StorageClass::SC_None);

    DeclarationName *dnMaskVar = new DeclarationName(iiMask);
    DeclarationNameInfo *dniMaskVar = new DeclarationNameInfo(*dnMaskVar, SourceLocation());

    Expr *dreMaskVar = DeclRefExpr::Create(context, NestedNameSpecifierLoc(),
        SourceLocation(), maskVar, false, *dniMaskVar, maskVar->getType(),
        clang::ExprValueKind::VK_RValue);
    //ImplicitCastExpr *iceMaskVar = ImplicitCastExpr::Create(context,
    //    nextVar->getType(), CastKind::CK_LValueToRValue, dreMaskVar, nullptr,
    //    ExprValueKind::VK_RValue);

    //DeclStmt *declStmt4 = new (context) DeclStmt(DeclGroupRef(maskVar),
    //    SourceLocation(), SourceLocation());

    UnaryOperator *uoAd = new (context) UnaryOperator(dreMaskVar,
        UnaryOperatorKind::UO_AddrOf, context.VoidPtrTy,
        clang::ExprValueKind::VK_RValue, ExprObjectKind::OK_Ordinary,
        SourceLocation());

    ArgsCreate.push_back(uoAd);

    //DeclRefExpr *dreCreate = DeclRefExpr::Create (context, *nnsl ,
    //    SourceLocation(), FDCreate, false, *dniCreate, funcTypeCreate,
    //    ExprValueKind::VK_RValue);
    //ImplicitCastExpr *iceCreate = ImplicitCastExpr::Create(context,
    //    context.getPointerType(funcTypeCreate),
    //    CastKind::CK_FunctionToPointerDecay, dreCreate, nullptr,
    //    ExprValueKind::VK_RValue);
    //clang::CallExpr *ceCreate = new (context) CallExpr(context,
    //    iceCreate, ArgsCreate, context.VoidPtrTy,
    //    clang::ExprValueKind::VK_RValue, SourceLocation());

    BinaryOperator *boiSub = new (context) BinaryOperator(dreCondVar,
        forCondVal , BinaryOperatorKind::BO_Sub, context.IntTy,
        clang::ExprValueKind::VK_RValue, ExprObjectKind::OK_Ordinary,
        SourceLocation(), false);

    ParenExpr * peiArg= new (context)
      ParenExpr(SourceLocation(),SourceLocation(),boiSub);

    BinaryOperator *boiDiv = new (context) BinaryOperator(peiArg, chunk,
        BinaryOperatorKind::BO_Div, context.IntTy,
        clang::ExprValueKind::VK_RValue, ExprObjectKind::OK_Ordinary,
        SourceLocation(), false);

    ParenExpr * peiArg2= new (context) ParenExpr(SourceLocation(),SourceLocation(),boiDiv);

    BinaryOperator *boiMod = new (context) BinaryOperator(peiArg2,
        IntegerLiteral::Create(context,
          llvm::APInt(context.getTypeSize(context.IntTy),
            (uint64_t) llvm::sys::getHostNumPhysicalCores()),
          context.IntTy, SourceLocation()) , BinaryOperatorKind::BO_Rem,
        context.IntTy, clang::ExprValueKind::VK_RValue,
        ExprObjectKind::OK_Ordinary, SourceLocation(), false);

    ArgsSetM.push_back(boiMod);
    ArgsSetM.push_back(uoAd);

    //DeclRefExpr *dreSetM = DeclRefExpr::Create (context, *nnsl ,
    //    SourceLocation(), FDSetM, false, *dniSetM, funcTypeSetM,
    //    ExprValueKind::VK_RValue);
    //ImplicitCastExpr *iceSetM = ImplicitCastExpr::Create(context,
    //    context.getPointerType(funcTypeSetM),
    //    CastKind::CK_FunctionToPointerDecay, dreSetM, nullptr,
    //    ExprValueKind::VK_RValue);
    //clang::CallExpr *ceSetM = new (context) CallExpr(context,
    //    iceSetM, ArgsSetM, context.VoidPtrTy,
    //    clang::ExprValueKind::VK_RValue, SourceLocation());

    ArgsSet.push_back(uoAd);

    //DeclRefExpr *dreSet = DeclRefExpr::Create (context, *nnsl ,
    //    SourceLocation(), FDSet, false, *dniSet, funcTypeSet,
    //    ExprValueKind::VK_RValue);
    //ImplicitCastExpr *iceSet = ImplicitCastExpr::Create(context,
    //    context.getPointerType(funcTypeSet),
    //    CastKind::CK_FunctionToPointerDecay, dreSet, nullptr,
    //    ExprValueKind::VK_RValue);
    //clang::CallExpr *ceSet = new (context) CallExpr(context,
    //    iceSet, ArgsSet, context.VoidPtrTy,
    //    clang::ExprValueKind::VK_RValue, SourceLocation());

    // Build an 'begin' call node
    DeclRefExpr *dreBegin = DeclRefExpr::Create (context, *nnsl ,
        SourceLocation(), FDBegin, false, *dniBegin, funcType1,
        ExprValueKind::VK_RValue);
    ImplicitCastExpr *iceBegin = ImplicitCastExpr::Create(context,
        context.getPointerType(funcType1),
        CastKind::CK_FunctionToPointerDecay, dreBegin, nullptr,
        ExprValueKind::VK_RValue);
    clang::CallExpr *ceBegin = new (context) CallExpr(context,
        iceBegin, Args1, context.UnsignedIntTy,
        clang::ExprValueKind::VK_RValue, SourceLocation());

    BinaryOperator *boAssign2 = new (context)
      BinaryOperator(dreExecuteVar, IntegerLiteral::Create(context,
            llvm::APInt(context.getTypeSize(context.IntTy),
              (uint64_t) 1), context.IntTy, SourceLocation()) ,
          BinaryOperatorKind::BO_Assign, context.IntTy,
          clang::ExprValueKind::VK_RValue, ExprObjectKind::OK_Ordinary,
          SourceLocation(), false);

    BinaryOperator *boAssign = new (context)
      BinaryOperator(dreStatusVar, ceBegin , BinaryOperatorKind::BO_Assign,
          context.UnsignedIntTy, clang::ExprValueKind::VK_RValue,
          ExprObjectKind::OK_Ordinary, SourceLocation(), false);

    std::vector<Stmt *> stmts,stmts1,stmts2;

    stmts.push_back(boAssign2);
    stmts.push_back(boAssign);

    IntegerLiteral * startedArg = IntegerLiteral::Create(context,
        llvm::APInt(context.getTypeSize(context.UnsignedIntTy),
          (uint64_t) 0), context.UnsignedIntTy, SourceLocation());

    UnaryOperator *uoNeg = new (context) UnaryOperator(startedArg,
        UnaryOperatorKind::UO_Not, context.UnsignedIntTy,
        clang::ExprValueKind::VK_RValue, ExprObjectKind::OK_Ordinary,
        SourceLocation());

    ParenExpr * peStartedArg= new (context)
      ParenExpr(SourceLocation(),SourceLocation(),uoNeg);

    BinaryOperator *boNE1 = new (context) BinaryOperator(dreStatusVar,
        peStartedArg, BinaryOperatorKind::BO_NE, context.UnsignedIntTy,
        clang::ExprValueKind::VK_RValue, ExprObjectKind::OK_Ordinary,
        SourceLocation(), false);

    GotoStmt *gotos = new (context) GotoStmt(RetryLab,
        SourceLocation(),SourceLocation());

    stmts1.push_back(gotos);

    CompoundStmt *CompSt1 = new (context) CompoundStmt(context,
        stmts1, SourceLocation(), SourceLocation());
    Stmt *stm1 = CompSt1;


    IfStmt *ifs1 = new (context) IfStmt(context, SourceLocation(),
        false, nullptr, nullptr, boNE1, stm1);
    stmts.push_back(ifs1);

    CompoundStmt *CompSt = new (context) CompoundStmt(context,
        stmts, SourceLocation(), SourceLocation());
    Stmt *stm = CompSt;

    IfStmt *ifs = new (context) IfStmt(context, SourceLocation(),
        false, nullptr, nullptr, boNE, stm);

    IntegerLiteral * abortArg = IntegerLiteral::Create(context,
        llvm::APInt(context.getTypeSize(context.IntTy), 255),
        context.IntTy, SourceLocation());

    clang::ParenExpr * peAbortArg= new (context)
      ParenExpr(SourceLocation(),SourceLocation(),abortArg);

    ImplicitCastExpr *iceAbortArg = ImplicitCastExpr::Create(context,
        context.CharTy, CastKind::CK_IntegralCast, peAbortArg, nullptr,
        ExprValueKind::VK_RValue);

    Args2.push_back(iceAbortArg);


    DeclRefExpr *dreEnd = DeclRefExpr::Create (context, *nnsl ,
        SourceLocation(), FDEnd, false, *dniEnd, funcType2,
        ExprValueKind::VK_RValue);
    ImplicitCastExpr *iceEnd = ImplicitCastExpr::Create(context,
        context.getPointerType(funcType2),
        CastKind::CK_FunctionToPointerDecay, dreEnd, nullptr,
        ExprValueKind::VK_RValue);
    clang::CallExpr *ceEnd = new (context) CallExpr(context,
        iceEnd, Args1, context.VoidTy, clang::ExprValueKind::VK_RValue,
        SourceLocation());

    DeclRefExpr *dreAbort = DeclRefExpr::Create (context, *nnsl ,
        SourceLocation(), FDAbort, false, *dniAbort, funcType3,
        ExprValueKind::VK_RValue);

    ImplicitCastExpr *iceAbort = ImplicitCastExpr::Create(context,
        context.getPointerType(funcType3),
        CastKind::CK_FunctionToPointerDecay, dreAbort, nullptr,
        ExprValueKind::VK_RValue);

    clang::CallExpr *ceAbort = new (context) CallExpr(context,
        iceAbort, Args2, FDAbort->getCallResultType(),
        Expr::getValueKindForType(FDAbort->getReturnType()),
        SourceLocation());

    IfStmt *ifs2 = new (context) IfStmt(context, SourceLocation(), false,
        nullptr, nullptr, boNE, ceAbort);

    stmts2.push_back(ifs2);
    stmts2.push_back(ceEnd);

    BinaryOperator *nextAdd = new (context) BinaryOperator(dreNextVar,
        chunk, BinaryOperatorKind::BO_Add, forCondVar->getType(),
        clang::ExprValueKind::VK_RValue, ExprObjectKind::OK_Ordinary,
        SourceLocation(), false);

    BinaryOperator *boAssign3 = new (context) BinaryOperator(dreNextVar,
        nextAdd , BinaryOperatorKind::BO_Assign, forCondVar->getType(),
        clang::ExprValueKind::VK_RValue, ExprObjectKind::OK_Ordinary,
        SourceLocation(), false);

    CompoundStmt *CompSt2 = new (context) CompoundStmt(context,
        stmts2, SourceLocation(), SourceLocation());
    Stmt *stm2 = CompSt2;

    IfStmt *ifs3 = new (context) IfStmt(context, SourceLocation(),
        false, nullptr, nullptr, dreExecuteVar, stm2);

    (*it).clear();
    (*it).push_back(declStmt2);
    (*it).push_back(declStmt3);
    (*it).push_back(labelStmt);
    (*it).push_back(ifs);
    (*it).push_back(forStmt);
    (*it).push_back(ifs3);
    (*it).push_back(boAssign3);
    i_component++;
  }

  CompoundStmt *CompS;
  Stmt *st;
  IntegerLiteral *stageFlagArg;

  // Iterate over the new components, with the already inserted function
  for(auto it = blocos.begin();it != blocos.end();it++) {
    ArgsCond.clear();

    stageFlagArg = IntegerLiteral::Create(context,
        llvm::APInt(context.getTypeSize(context.IntTy), (uint64_t)
          1), context.IntTy, SourceLocation());

    ArgsCond.push_back(stageFlagArg);

    // Create the 'CallExpr' node for the IF condition
    //DeclRefExpr *dreCond = DeclRefExpr::Create (context, *nnsl ,
    //    SourceLocation(), FDCond, false, *dniCond, funcTypeCond,
    //    ExprValueKind::VK_RValue);
    //ImplicitCastExpr *iceCond = ImplicitCastExpr::Create(context,
    //    context.getPointerType(funcTypeCond),
    //    CastKind::CK_FunctionToPointerDecay, dreCond, nullptr,
    //    ExprValueKind::VK_RValue);
    //clang::CallExpr *ceCond = new (context) CallExpr(context,
    //    iceCond, ArgsCond, context.IntTy,
    //    clang::ExprValueKind::VK_RValue, SourceLocation());

    // Create a new 'CompoundStmt' containing the component statements
    CompS = new (context) CompoundStmt(context, *it,
        SourceLocation(), SourceLocation());
    st = CompS;
  }

  // Set the 'CompoundStmt' as the For body in the AST
  dyn_cast<ForStmt>(dyn_cast<CapturedStmt>(S.getAssociatedStmt())->getCapturedStmt())->setBody(st);

  //Create a new 'OMPNumThreadsClause' node to insert the 'num_threads' clause in the pragma
  /*
  IntegerLiteral *numThreads = IntegerLiteral::Create(context,
      llvm::APInt(context.getTypeSize(context.IntTy), (uint64_t)
        llvm::sys::getHostNumPhysicalCores()), context.IntTy,
      SourceLocation());
  OMPNumThreadsClause *numThreadsClause =  new (context)
    OMPNumThreadsClause(numThreads, S.getLocStart(), S.getLocEnd(),
        S.getLocEnd());
  */

  IntegerLiteral *chunkSize = IntegerLiteral::Create(context,
      llvm::APInt(context.getTypeSize(context.IntTy), (uint64_t)
        1), context.IntTy, SourceLocation());
  OMPGrainsizeClause *grainsizeClause = new (context)
    OMPGrainsizeClause(chunkSize,S.getLocStart(), S.getLocEnd(), S.getLocEnd());

  std::vector<OMPClause *> clauses = S.clauses().vec();
/*
  OMPPrivateClause *P = nullptr;
  for(auto cl = clauses.begin(); cl != clauses.end(); cl++) {
    if(isa<OMPPrivateClause>(*cl)) {
      P = dyn_cast<OMPPrivateClause>(*cl);
      break;
    }
  }

  std::vector<Expr *> privates;
  if(P != nullptr) {
    clang::OMPVarListClause<OMPPrivateClause> *Pvars =
      dyn_cast<clang::OMPVarListClause<OMPPrivateClause> >(P);

    for (auto vr = Pvars->varlist_begin(); vr != Pvars->varlist_end(); vr++) {
      DeclRefExpr *dre = dyn_cast<DeclRefExpr>(*vr);
      privates.push_back(dre);
    }
  }

  OMPPrivateClause *privateClause = OMPPrivateClause::Create(context,
      S.getLocStart(), S.getLocEnd(), S.getLocEnd(), privates, privates);
*/

  OMPSharedClause *P = nullptr;
  for(auto cl = clauses.begin(); cl != clauses.end(); cl++) {
    if(isa<OMPSharedClause>(*cl)) {
      P = dyn_cast<OMPSharedClause>(*cl);
      break;
    }
  }

 std::vector<Expr *> shareds;
  if(P != nullptr) {
    clang::OMPVarListClause<OMPSharedClause> *Svars =
      dyn_cast<clang::OMPVarListClause<OMPSharedClause> >(P);

    for (auto vr = Svars->varlist_begin(); vr != Svars->varlist_end(); vr++) {
      DeclRefExpr *dre = dyn_cast<DeclRefExpr>(*vr);
      shareds.push_back(dre);
    }
  }

  shareds.push_back(dreNextVar);



  OMPSharedClause *sharedClause = OMPSharedClause::Create(context,
      S.getLocStart(), S.getLocEnd(), S.getLocEnd(), shareds);


  
  /*
  bool found=false;
  
  for(auto cl = clauses.begin(); cl != clauses.end(); cl++) {
    if(isa<OMPNumThreadsClause>(*cl)) {
      //clauses.erase(cl);
      found=true;
    }
  }
  */

  for(auto cl = clauses.begin(); cl != clauses.end(); cl++) {
    if(isa<OMPOrderedClause>(*cl)) {
      clauses.erase(cl);
      break;
    }
  }
/*
  for(auto cl = clauses.begin(); cl != clauses.end(); cl++) {
    if(isa<OMPPrivateClause>(*cl)) {
      clauses.erase(cl);
      break;
    }
  }
*/
  for(auto cl = clauses.begin(); cl != clauses.end(); cl++) {
    if(isa<OMPSharedClause>(*cl)) {
      clauses.erase(cl);
      break;
    }
  }

  // Insert 'num_threads' and 'grain_size' clause
  //if (!found) clauses.push_back(numThreadsClause);
  clauses.push_back(grainsizeClause);
  //clauses.push_back(privateClause);
  clauses.push_back(sharedClause);

  OMPLoopDirective::HelperExprs B;
  // Copy the struct 'HelperExprs', which is responsible for controlling loop bounds
  B.IterationVarRef = S.getIterationVariable();
  B.LastIteration = S.getLastIteration();
  B.CalcLastIteration = S.getCalcLastIteration();
  B.PreCond = S.getPreCond();
  B.Cond = S.getCond();
  B.Init = S.getInit();
  B.Inc = S.getInc();
  B.IL = S.getIsLastIterVariable();
  B.LB = S.getLowerBoundVariable();
  B.UB = S.getUpperBoundVariable();
  B.ST = S.getStrideVariable();
  B.EUB = S.getEnsureUpperBound();
  B.NLB = S.getNextLowerBound();
  B.NUB = S.getNextUpperBound();
  B.NumIterations = S.getNumIterations();
  B.PrevLB = S.getPrevLowerBoundVariable();
  B.PrevUB = S.getPrevUpperBoundVariable();
  for (auto it=S.counters().begin();it!=S.counters().end();it++) {
    (B.Counters).push_back(*it);
  }
  for (auto it=S.private_counters().begin();it!=S.private_counters().end();it++) {
    (B.PrivateCounters).push_back(*it);
  }
  for (auto it=S.inits().begin();it!=S.inits().end();it++) {
    (B.Inits).push_back(*it);
  }
  for (auto it=S.updates().begin();it!=S.updates().end();it++) {
    (B.Updates).push_back(*it);
  }
  for (auto it=S.finals().begin();it!=S.finals().end();it++) {
    (B.Finals).push_back(*it);
  }
  Stmt* temp = const_cast<Stmt *>(S.getPreInits());
  B.PreInits = temp;

  OMPTaskLoopDirective *newS = OMPTaskLoopDirective::Create(context,
      S.getLocStart(), S.getLocEnd(), S.getCollapsedNumber(), clauses,
      S.getAssociatedStmt(), B);

  newS->dumpPretty(context);
  return newS;
}

static
const OMPTaskLoopDirective*
emitOMPTaskLoopWithUseClause(CodeGenFunction& CGF, CodeGenModule& CGM,
    const OMPTaskLoopDirective& S, const OMPUseClause* useClause) {
  OMPTaskLoopDirective * newS = const_cast<OMPTaskLoopDirective*>(&S);
	
  switch(useClause->getUseKind()) {
    case OMPC_USE_tls:
      newS = emitOMPTaskLoopWithUseTLS(CGF, CGM, S, useClause);
      break;
    default:
      llvm::errs() << "error: Unknown algorithm select!" << '\n';
      break;
  }
  return newS;
}

void CodeGenFunction::EmitOMPTaskLoopDirective(const OMPTaskLoopDirective &S) {
  

  const OMPUseClause *C = S.getSingleClause<OMPUseClause>();

  if (C != nullptr){
	
      const OMPTaskLoopDirective *newS = emitOMPTaskLoopWithUseClause(*this,
CGM,S,C);
      
      EmitOMPTaskLoopBasedDirective(*newS);

  }
  else

      EmitOMPTaskLoopBasedDirective(S);	



}

/*
void CodeGenFunction::EmitOMPParallelForDirective(
    const OMPParallelForDirective &S) {

	const OMPUseClause *C = S.getSingleClause<OMPUseClause>();

  if (C != nullptr) {
    // Emit parallel-for directive with use clause
    const OMPParallelForDirective *newS =
      emitOMPParallelForWithUseClause(*this, CGM, S, C);
	  auto &&CodeGen = [newS](CodeGenFunction &CGF, PrePostActionTy &) {
	    CGF.EmitOMPWorksharingLoop(*newS);
	  };
		emitCommonOMPParallelDirective(*this, S, OMPD_for, CodeGen);
  } else {
    // Emit directive as a combined directive that consists of two implicit
    // directives: 'parallel' with 'for' directive.
	  auto &&CodeGen = [&S](CodeGenFunction &CGF, PrePostActionTy &) {
      CGF.EmitOMPWorksharingLoop(S);
	  };
		emitCommonOMPParallelDirective(*this, S, OMPD_for, CodeGen);
  }
}

*/

/*
void CodeGenFunction::EmitOMPTaskLoopDirective(const OMPTaskLoopDirective &S) {
  
      EmitOMPTaskLoopBasedDirective(S);


}
*/


void CodeGenFunction::EmitOMPTaskLoopSimdDirective(
    const OMPTaskLoopSimdDirective &S) {
  EmitOMPTaskLoopBasedDirective(S);
}

// Generate the instructions for '#pragma omp target update' directive.
void CodeGenFunction::EmitOMPTargetUpdateDirective(
    const OMPTargetUpdateDirective &S) {
  // If we don't have target devices, don't bother emitting the data mapping
  // code.
  if (CGM.getLangOpts().OMPTargetTriples.empty())
    return;

  // Check if we have any if clause associated with the directive.
  const Expr *IfCond = nullptr;
  if (auto *C = S.getSingleClause<OMPIfClause>())
    IfCond = C->getCondition();

  // Check if we have any device clause associated with the directive.
  const Expr *Device = nullptr;
  if (auto *C = S.getSingleClause<OMPDeviceClause>())
    Device = C->getDevice();

  CGM.getOpenMPRuntime().emitTargetDataStandAloneCall(*this, S, IfCond, Device);
}
