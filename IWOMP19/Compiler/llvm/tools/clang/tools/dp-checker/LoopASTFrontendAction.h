#pragma once
//#include <clang/Basic/FileManager.h>
//#include <clang/Basic/LLVM.h>
//#include <clang/Basic/Diagnostic.h>

//#include <clang/Frontend/CompilerInvocation.h>
#include <clang/Frontend/FrontendAction.h>

#include <clang/Rewrite/Core/Rewriter.h>
#include <clang/Rewrite/Core/RewriteBuffer.h>

//#include <clang/Tooling/Tooling.h>
//#include <clang/Tooling/ArgumentsAdjusters.h>
//#include <clang/Tooling/CompilationDatabase.h>

#include <llvm/Support/MemoryBuffer.h>
//#include <llvm/Support/Path.h>
//#include <llvm/ADT/StringSet.h>
//#include <llvm/ADT/Twine.h>

namespace clang {

class LoopASTFrontendAction : public clang::ASTFrontendAction {
private:
public:
  //LoopASTFrontendAction();
  ~LoopASTFrontendAction();
  virtual std::unique_ptr<clang::ASTConsumer>
    CreateASTConsumer(clang::CompilerInstance& CI, clang::StringRef InFile) override;
};

} // namespace clang
