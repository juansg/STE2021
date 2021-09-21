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

#include <map>
#include <set>
#include <regex>

#include "BracesAroundStatementsCheck.h"
#include "BracesFixerASTFrontendAction.h"

using namespace clang;
using namespace clang::ast_matchers;
using namespace clang::tidy::readability;

namespace { //anonymous

class BracesFixerASTVisitor : public RecursiveASTVisitor<BracesFixerASTVisitor> {
private:
  ASTContext &context;
  SourceManager &SM;
  Rewriter rewriter;

public:
  BracesFixerASTVisitor(ASTContext &C)
      : context(C), SM(C.getSourceManager()) {
    rewriter.setSourceMgr(SM, C.getLangOpts());
  }

  ~BracesFixerASTVisitor() {
    const RewriteBuffer* rewriteBuffer =
        rewriter.getRewriteBufferFor(SM.getMainFileID());

    // Skip if no modefications were made
    if (rewriteBuffer != nullptr) {
      const FileEntry *fileEntry = SM.getFileEntryForID(SM.getMainFileID());
      const std::string inputFileName = fileEntry->getName();

      rewriter.overwriteChangedFiles();
      //llvm::errs() << std::string(rewriteBuffer->begin(), rewriteBuffer->end());
    }
  }

  bool VisitFunctionDecl(FunctionDecl* f) {

    if (f->clang::Decl::hasBody()) {
      const CompoundStmt* functionBody =
        static_cast<CompoundStmt*>(f->getBody());

      MatchFinder Finder;
      BracesAroundStatementsCheck checker(rewriter);
      checker.registerMatchers(&Finder);

      Finder.match(*functionBody, context);
    }
    // continue parsing
    return true;
  }
};

class BracesFixerASTConsumer : public ASTConsumer {
private:
  ASTContext& context;
  BracesFixerASTVisitor Visitor;

public:
  explicit BracesFixerASTConsumer(ASTContext &C) : context(C), Visitor(C) {}
  // Override the method that is called for each top-level declaration.
  virtual bool HandleTopLevelDecl(DeclGroupRef DR) {
    SourceManager& SM =  context.getSourceManager();
    FileID mainFileID = SM.getMainFileID();
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

std::unique_ptr<ASTConsumer> BracesFixerASTFrontendAction::CreateASTConsumer(
    CompilerInstance& CI, StringRef InFile) {

  //llvm::errs() << "InFile: " <<  InFile << '\n';
  LangOptions &langOptions = CI.getLangOpts();
  if (!langOptions.OpenMP) {
    llvm::errs() << "error: OpenMP support disabled (missing -fopenmp flag?)\n";
    exit(EXIT_FAILURE);
  }
  return llvm::make_unique<BracesFixerASTConsumer>(CI.getASTContext());
}

BracesFixerASTFrontendAction::~BracesFixerASTFrontendAction() {
  //llvm::errs() << "-------- BracesFixerASTFrontendAction destructor called\n";
}

} // namespace clang
