#pragma once

#include <clang/Frontend/FrontendAction.h>

#include <llvm/Support/MemoryBuffer.h>

namespace clang {

class PostLoopASTFrontendAction : public clang::ASTFrontendAction {
private:
public:
  //PostLoopASTFrontendAction();
  ~PostLoopASTFrontendAction();
  virtual std::unique_ptr<clang::ASTConsumer>
    CreateASTConsumer(clang::CompilerInstance& CI, clang::StringRef InFile) override;
};

} // namespace clang
