#include <clang/Frontend/FrontendActions.h>
#include <clang/Frontend/CompilerInstance.h>

#include <clang/Lex/Lexer.h>

#include <clang/Lex/PPCallbacks.h>
#include <clang/Lex/Preprocessor.h>

#include <clang/AST/ASTConsumer.h>
#include <clang/AST/RecursiveASTVisitor.h>
#include <clang/AST/StmtOpenMP.h>
#include <clang/AST/Decl.h>
#include <clang/AST/Expr.h>
#include <clang/AST/OperationKinds.h>

#include <clang/ASTMatchers/ASTMatchFinder.h>
#include <clang/ASTMatchers/ASTMatchers.h>

#include <llvm/Support/raw_ostream.h>

#include <map>
#include <set>

#include "LoopASTFrontendAction.h"
#include "CallSite.h"

using namespace clang;
using namespace clang::ast_matchers;

std::set<std::string> callsInsideOMPForLoop;
std::map<std::string, CallSite> mapFunctionNameToCallSite;

namespace { //anonymous

static std::set<OMPParallelForDirective *> processedOMPParallelFor;

StatementMatcher callExprMatcher = findAll(
    callExpr().bind("call_expr"));

class CallExprMatcherCallback : public MatchFinder::MatchCallback {

  SourceManager& SM;

  virtual void run(const MatchFinder::MatchResult &Result) {
    if (const CallExpr* callExpr =
        Result.Nodes.getNodeAs<CallExpr>("call_expr")) {
      const FunctionDecl* callee = callExpr->getDirectCallee();
      if (callee != nullptr) {
        std::string callTarget = callee->getQualifiedNameAsString();
        bool invalid = false;
        unsigned callSiteLineNumber =
          SM.getSpellingLineNumber(callExpr->getLocStart(), &invalid);
        if (invalid) {
          llvm::errs() << "fatal error: invalid call-site for call to " <<
            callTarget << '\n';
          exit(EXIT_FAILURE);
        }
        std::string callSiteFileName = SM.getFilename(callExpr->getLocStart());
        //llvm::errs() << "call to '" << callTarget << "' found at " <<
        //  callSiteFileName << " on line " << callSiteLineNumber << "!\n";
        if (callsInsideOMPForLoop.count(callTarget) == 0) {
          callsInsideOMPForLoop.insert(callTarget);
          mapFunctionNameToCallSite[callTarget] =
            CallSite(callSiteLineNumber, callSiteFileName);
        }
      } else {
        llvm::errs() << "warning: indirect call inside OMPForLoop!\n";
        llvm::errs() << "  profiling info might be inacurate!\n";
      }
    }
  }
public:
  CallExprMatcherCallback (SourceManager& SM) : SM(SM) {}
};

class LoopASTVisitor : public RecursiveASTVisitor<LoopASTVisitor> {
private:
  ASTContext &context;
  Rewriter& rewriter;

public:
  LoopASTVisitor(ASTContext &C, Rewriter& R)
      : context(C), rewriter(R) {
    //llvm::errs() << "-------- LoopASTVisitor constructor called\n";
  }

  ~LoopASTVisitor() {
    //llvm::errs() << "-------- LoopASTVisitor destructor called\n";
  }

  bool VisitOMPParallelDirective(OMPParallelDirective* s) {
    //llvm::errs() << "-------- support to OMPParallelDirective not implemented yet!\n";
    return true;
  }

  bool VisitOMPParallelForDirective(OMPParallelForDirective *s) {

    if ( s->getSingleClause<OMPCheckClause>() == nullptr) {
      return true;
    }

    if (processedOMPParallelFor.count(s) == 0) {
      processedOMPParallelFor.insert(s);
      OMPParallelForDirective *ompPFD =
          static_cast<OMPParallelForDirective *>(s);
      CapturedStmt *CS =
          static_cast<CapturedStmt *>(ompPFD->getAssociatedStmt());
      CapturedDecl *CD = CS->getCapturedDecl();
      ForStmt *forStmt = static_cast<ForStmt *>(CD->getBody());
      const CompoundStmt *forBody =
          static_cast<CompoundStmt *>(forStmt->getBody());

      MatchFinder Finder;
      CallExprMatcherCallback callback(context.getSourceManager());
      Finder.addMatcher(callExprMatcher, &callback);
      Finder.match(*forBody, context);

      SourceManager& SM = context.getSourceManager();

      SourceLocation startParallelRegion = SM.getFileLoc(forStmt->getLocStart());
      if (startParallelRegion.isInvalid()) {
        llvm::errs() << "error: invalid startParallelRegion source location\n";
      }

      SourceLocation endParallelRegion = SM.getFileLoc(forBody->getRBracLoc());
      endParallelRegion =
        Lexer::getLocForEndOfToken(endParallelRegion, 0, SM, context.getLangOpts());

      if (endParallelRegion.isInvalid()) {
        llvm::errs() << "error: invalid endParallelRegion source location\n";
      }

      /* *
       * Start of transformation
       *
       * Input:
       * #pragma omp parallel for [<clause0>, <clause1> ...]
       * forStmt
       *
       * Output:
       * #pragma omp parallel [<clause0>, <clause1> ...]
       * {
       *   #pragma omp for
       *   forStmt
       * }
       * */
      const unsigned numClauses = ompPFD->getNumClauses();
      SourceLocation startOfOMPParallelFor = ompPFD->getLocStart();
      SourceLocation endOfOMPParallelFor = ompPFD->getLocEnd();
      std::string newParallelDirectiveString;
      std::string newForDirectiveString;
      int numCollapsed = -1;
      if (numClauses > 0) {
        std::vector<OMPClause*> parallelDirectiveClauses;
        std::vector<OMPClause*> forDirectiveClauses;
        for (unsigned i = 0; i <  numClauses; i++) {
          OMPClause* clause = ompPFD->getClause(i);
          switch (clause->getClauseKind()) {
            case OpenMPClauseKind::OMPC_num_threads:
              // remove num_threads clause
              break;
            case OpenMPClauseKind::OMPC_check:
              // remove check clause
              break;
            case OpenMPClauseKind::OMPC_collapse:
              if (true) {
                const OMPCollapseClause* c =
                  static_cast<OMPCollapseClause*>(clause);
                const IntegerLiteral* numLoopsIL =
                  static_cast<IntegerLiteral*>(c->getNumForLoops());
                llvm::APInt numLoops = numLoopsIL->getValue();
                numCollapsed = numLoops.getZExtValue();
              }
            case OpenMPClauseKind::OMPC_lastprivate:
            case OpenMPClauseKind::OMPC_schedule:
            case OpenMPClauseKind::OMPC_ordered:
            case OpenMPClauseKind::OMPC_linear:
              forDirectiveClauses.push_back(clause);
              break;
            default:
              parallelDirectiveClauses.push_back(clause);
              break;
          }
        }
        OMPParallelDirective* newParallelDirective =
          clang::OMPParallelDirective::Create(context,
              /*StartLoc*/SourceLocation(),
              /*EndLoc*/SourceLocation(),
              parallelDirectiveClauses,
              /*AssociatedStmt*/nullptr,
              /*hasCancel*/false);
        OMPForDirective* newForDirective =
          clang::OMPForDirective::Create(context,
              /*StartLoc*/SourceLocation(),
              /*EndLoc*/SourceLocation(),
              /*numCollapsed*/0, forDirectiveClauses,
              /*AssociatedStmt*/nullptr,
              /*helperExprs*/OMPLoopDirective::HelperExprs(),
              /*hasCancel*/false);

        llvm::raw_string_ostream newOMPPDStream(newParallelDirectiveString);
        llvm::raw_string_ostream newOMPFDStream(newForDirectiveString);
        newParallelDirective->printPretty(newOMPPDStream,
            /*PrinterHelper*/nullptr,
            /*PrintingPolicy*/context.getPrintingPolicy(), /*Indentation*/0);
        newOMPPDStream.str();
        newForDirective->printPretty(newOMPFDStream,
            /*PrinterHelper*/nullptr,
            /*PrintingPolicy*/context.getPrintingPolicy(), /*Indentation*/0);
        newOMPFDStream.str();

        newParallelDirectiveString.erase(0,8); // erase '#pragma '
        size_t n = newParallelDirectiveString.length();
        newParallelDirectiveString.erase(n-1,n); // erase '\n'
        newParallelDirectiveString = newParallelDirectiveString + " num_threads(1)";
      } else {
        newParallelDirectiveString = "omp parallel num_threads(1)";
        newForDirectiveString = "#pragma omp for\n";
      }

      //llvm::errs() << "-------- " << newParallelDirectiveString << '\n';
      //llvm::errs() << "-------- " << newForDirectiveString << '\n';


      rewriter.InsertText(startParallelRegion,
                          (Twine("{ // START OF PARALLEL REGION\n")
                            + "int __threadID__ = omp_get_thread_num();\n"
                            + "int __numThreads__ = omp_get_num_threads();\n"
                            + "__enterParallelRegion(__threadID__, __numThreads__);\n"
                            + newForDirectiveString
                          ).str(),
                          /*insertAfter*/ true, /*indentNewLines*/ true);

      rewriter.InsertText(endParallelRegion, "\n"
                          "__exitParallelRegion();\n"
                          "} // END OF PARALLEL REGION\n",
                          /*insertAfter*/ true, /*indentNewLines*/ true);

      rewriter.ReplaceText(
          SourceRange(startOfOMPParallelFor, endOfOMPParallelFor),
          newParallelDirectiveString);
      /* End of transformation */


      /* Capture iteration VarDecl */
      const Stmt *forInitStmt = forStmt->getInit();
      const VarDecl *forIterationVarDecl;
      if (isa<DeclStmt>(forInitStmt)) {
        const DeclStmt *decl = static_cast<const DeclStmt *>(forInitStmt);
        forIterationVarDecl = static_cast<const VarDecl *>(*decl->decl_begin());
      } else if (isa<BinaryOperator>(forInitStmt)) {
        const BinaryOperator *bo =
            static_cast<const BinaryOperator *>(forInitStmt);
        const DeclRefExpr *lhs = reinterpret_cast<DeclRefExpr *>(bo->getLHS());
        forIterationVarDecl = static_cast<const VarDecl *>(lhs->getDecl());
      } else {
        forIterationVarDecl = nullptr;
        llvm::errs() << "FATAL ERROR: forInitStmt is neither DeclStmt nor "
                        "BinaryOperator!\n";
      }

      if (numCollapsed > 0) {
        for(int i = 0; i < numCollapsed-1; i++) {
          const ForStmt* innerForStmt =
            static_cast<const ForStmt*>(forBody->body_front());
          forBody = static_cast<const CompoundStmt*>(innerForStmt->getBody());
        }
      }

      SourceLocation startLoopBody = forBody->getLBracLoc();
      startLoopBody = Lexer::getLocForEndOfToken(startLoopBody, 0, SM, context.getLangOpts());
      if (startLoopBody.isInvalid()) {
        llvm::errs() << "error: invalid startLoopBody source location\n";
      }

      SourceLocation endLoopBody = SM.getFileLoc(forBody->getRBracLoc());
      endLoopBody = Lexer::GetBeginningOfToken(endLoopBody, SM, context.getLangOpts());
      if (endLoopBody.isInvalid()) {
        llvm::errs() << "error: invalid endLoopBody source location\n";
      }

      rewriter.InsertText(startLoopBody,
          (Twine("\n\n  __start_iter_prof( (unsigned long)")
           + forIterationVarDecl->getNameAsString()
           //+ ", __threadID__"
           + ");\n").str(),
          /*InsertAfter*/true, /*IndentNewLines*/true);
      rewriter.InsertText(endLoopBody,
          (Twine("\n  __stop_iter_prof( (unsigned long)")
           + forIterationVarDecl->getNameAsString()
           //+ ", __threadID__"
           + ");\n\n").str(),
          /*InsertAfter*/false, /*IndentNewLines*/true);
    }

    // continue parsing
    return true;
  }
};

class LoopASTConsumer : public ASTConsumer {
private:
  LoopASTVisitor Visitor;
  SourceManager &SM;
  Rewriter TheRewriter;

public:
  explicit LoopASTConsumer(ASTContext &C) :
      Visitor(C, TheRewriter), SM(C.getSourceManager()) {
    TheRewriter.setSourceMgr(SM, C.getLangOpts());
  }

  ~LoopASTConsumer() {
    const RewriteBuffer* rewriteBuffer =
        TheRewriter.getRewriteBufferFor(SM.getMainFileID());

    // Skip if no modefications were made
    if (rewriteBuffer != nullptr) {
      std::string header =
        (Twine("#include <omp.h>\n")
        + "#include <check-rt/check-rt.h>\n\n").str();
      TheRewriter.getEditBuffer(SM.getMainFileID()).InsertTextBefore(0, header);
      TheRewriter.overwriteChangedFiles();
    }
  }

  // Override the method that is called for each top-level declaration.
  virtual bool HandleTopLevelDecl(DeclGroupRef DR) {
    for (DeclGroupRef::iterator d = DR.begin(), e = DR.end(); d != e; d++) {
      Visitor.TraverseDecl(*d);
    }
    // continue parsing
    return true;
  }
};

class MyPPCallBack : public PPCallbacks {
private:
  Preprocessor& PP;
  SourceManager& SM;
  SourceLocation lastInclusionLocation;

public:

  MyPPCallBack (Preprocessor& PP) : PP(PP), SM(PP.getSourceManager()) {}

  void FileChanged (SourceLocation Loc, FileChangeReason Reason,
      SrcMgr::CharacteristicKind FileType, FileID PrevFID) override {

    if (Reason != FileChangeReason::EnterFile) {
      return;
    } else if (lastInclusionLocation.isInvalid()) {
      return;
    } else if (FileType != SrcMgr::CharacteristicKind::C_User) {
      return;
    } else {
      //FileID parentFileID = SM.getFileID(lastInclusionLocation);
      //const FileEntry* parentFileEntry =
      //  SM.getFileEntryForID(parentFileID);
      //FileID includedFileID = SM.getFileID(Loc);
      //const FileEntry* includedFileEntry =
      //  SM.getFileEntryForID(includedFileID);
      //llvm::errs() << "==== file " <<
      //  includedFileEntry->getName() <<
      //  " included in file " << parentFileEntry->getName()
      //  << '\n';
    }
    lastInclusionLocation = SourceLocation();
  }

  void InclusionDirective (SourceLocation HashLoc, const Token &IncludeTok,
      StringRef FileName, bool IsAngled, CharSourceRange FilenameRange,
      const FileEntry *File, StringRef SearchPath, StringRef RelativePath,
      const Module *Imported) override {

    if (Imported) {
      return;
    } else if ( ! SM.isInMainFile(HashLoc) ) {
      return;
    } else {
      //llvm::errs() << "==== found include " <<
      //  File->getName() << '\n';
      lastInclusionLocation = HashLoc;
    }
  }
};

} // anonymous namespace

namespace clang {

std::unique_ptr<ASTConsumer> LoopASTFrontendAction::CreateASTConsumer(
    CompilerInstance& CI, StringRef InFile) {

  //llvm::errs() << "InFile: " <<  InFile << '\n';
  LangOptions &langOptions = CI.getLangOpts();
  if (!langOptions.OpenMP) {
    llvm::errs() << "error: OpenMP support disabled (missing -fopenmp flag?)\n";
    exit(EXIT_FAILURE);
  }

  Preprocessor& PP = CI.getPreprocessor();
  PP.addPPCallbacks(llvm::make_unique<MyPPCallBack>(PP));

  return llvm::make_unique<LoopASTConsumer>(CI.getASTContext());
}

LoopASTFrontendAction::~LoopASTFrontendAction() {
  //llvm::errs() << "-------- LoopASTFrontendAction destructor called\n";
}

} // namespace clang
