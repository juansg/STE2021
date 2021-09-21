#include <clang/Frontend/FrontendActions.h>
#include <clang/Frontend/CompilerInstance.h>

#include <clang/Lex/Lexer.h>

#include <clang/AST/ASTConsumer.h>
#include <clang/AST/RecursiveASTVisitor.h>
#include <clang/AST/StmtOpenMP.h>
#include <clang/AST/Decl.h>
#include <clang/AST/Expr.h>
#include <clang/AST/OperationKinds.h>

#include <clang/ASTMatchers/ASTMatchFinder.h>
#include <clang/ASTMatchers/ASTMatchers.h>

#include <clang/Rewrite/Core/Rewriter.h>
#include <clang/Rewrite/Core/RewriteBuffer.h>

#include <llvm/Support/PrettyStackTrace.h>
#include <llvm/Support/raw_ostream.h>

#include <set>

#include "PostLoopASTFrontendAction.h"
#include "CallSite.h"

using namespace clang;
using namespace clang::ast_matchers;

extern std::set<std::string> callsInsideOMPForLoop;
extern std::map<std::string, CallSite> mapFunctionNameToCallSite;

bool visitedFunctionsChanged;
std::set<std::string> visitedFunctions;

namespace { //anonymous

StatementMatcher callExprMatcher = findAll(
    callExpr().bind("call_expr"));

class CallExprMatcherCallback : public MatchFinder::MatchCallback {

  virtual void run(const MatchFinder::MatchResult &Result) {
    if (const CallExpr* callExpr =
        Result.Nodes.getNodeAs<CallExpr>("call_expr")) {
      const FunctionDecl* callee = callExpr->getDirectCallee();
      if (callee != nullptr) {
        std::string callTarget = callee->getQualifiedNameAsString();
        //llvm::errs() << "call to '" << callTarget << "' found!\n";
        if (callsInsideOMPForLoop.count(callTarget) == 0) {
          callsInsideOMPForLoop.insert(callTarget);
        }
      } else {
        llvm::errs() << "warning: indirect call inside OMPForLoop!\n";
        llvm::errs() << "  profiling info might be inacurate!\n";
      }
    }
  }
};

class PostLoopASTVisitor : public RecursiveASTVisitor<PostLoopASTVisitor> {
private:
  ASTContext &context;
  SourceManager &SM;
  Rewriter rewriter;

public:
  PostLoopASTVisitor(ASTContext &C)
      : context(C), SM(C.getSourceManager()) {
    rewriter.setSourceMgr(SM, C.getLangOpts());
  }

  ~PostLoopASTVisitor() {
    const RewriteBuffer* rewriteBuffer =
        rewriter.getRewriteBufferFor(SM.getMainFileID());

    // Skip if no modefications were made
    if (rewriteBuffer != nullptr) {
      std::string header =
        (Twine("#include <omp.h>\n")
        + "#include <check-rt/check-rt.h>\n\n").str();
      rewriter.getEditBuffer(SM.getMainFileID()).InsertTextBefore(0, header);
      rewriter.overwriteChangedFiles();
    }
  }

  bool VisitFunctionDecl(FunctionDecl* f) {

    std::string functionName = f->getQualifiedNameAsString();
    if (f->clang::Decl::hasBody()
        && callsInsideOMPForLoop.count(functionName) > 0
        && visitedFunctions.count(functionName) == 0) {
      const CompoundStmt* functionBody =
        static_cast<CompoundStmt*>(f->getBody());

      MatchFinder Finder;
      CallExprMatcherCallback callback;
      Finder.addMatcher(callExprMatcher, &callback);
      Finder.match(*functionBody, context);

      SourceLocation startFunctionBody =
        functionBody->getLocStart().getLocWithOffset(1);

      std::map<std::string, CallSite>::iterator it =
        mapFunctionNameToCallSite.find(functionName);
      std::string callSiteName = "";
      unsigned lineNumber = 0;
      if (it != mapFunctionNameToCallSite.end()) {
        const CallSite& callSite = it->second;
        lineNumber = callSite.lineNumber;
        callSiteName = callSite.fileName;
      }
      rewriter.InsertText(startFunctionBody,
          (Twine("\n\n  __function_entry(")
           + std::to_string(lineNumber)
           +  ", "
           + "__builtin_extract_return_addr(__builtin_return_address(0))"
           + ");\n").str(),
          /*InsertAfter*/true, /*IndentNewLines*/true);

      visitedFunctions.insert(functionName);
      visitedFunctionsChanged = true;
    }
    // continue parsing
    return true;
  }
};

class PostLoopASTConsumer : public ASTConsumer {
private:
  ASTContext& context;
  PostLoopASTVisitor Visitor;

public:
  explicit PostLoopASTConsumer(ASTContext &C) : context(C), Visitor(C) {}
  // Override the method that is called for each top-level declaration.
  virtual bool HandleTopLevelDecl(DeclGroupRef DR) {
    SourceManager& SM =  context.getSourceManager();
    FileID mainFileID = SM.getMainFileID();
    visitedFunctionsChanged = false;
    for (DeclGroupRef::iterator d = DR.begin(), e = DR.end(); d != e; d++) {
      // Only interested on functions declared in the file been parsed
      if (SM.getFileID((*d)->getLocation()) == mainFileID) {
        Visitor.TraverseDecl(*d);
      }
    }
    // continue parsing
    return true;
  }
};

} // anonymous namespace

namespace clang {

std::unique_ptr<ASTConsumer> PostLoopASTFrontendAction::CreateASTConsumer(
    CompilerInstance& CI, StringRef InFile) {

  llvm::errs() << "InFile: " <<  InFile << '\n';
  LangOptions &langOptions = CI.getLangOpts();
  if (!langOptions.OpenMP) {
    llvm::errs() << "error: OpenMP support disabled (missing -fopenmp flag?)\n";
    exit(EXIT_FAILURE);
  }
  return llvm::make_unique<PostLoopASTConsumer>(CI.getASTContext());
}

PostLoopASTFrontendAction::~PostLoopASTFrontendAction() {
  //llvm::errs() << "-------- PostLoopASTFrontendAction destructor called\n";
}

} // namespace clang
