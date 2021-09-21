#pragma once
#include <clang/Basic/FileManager.h>
#include <clang/Basic/LLVM.h>
#include <clang/Basic/Diagnostic.h>

#include <clang/Frontend/CompilerInvocation.h>
#include <clang/Frontend/FrontendAction.h>

#include <clang/CodeGen/CodeGenAction.h>

#include <clang/Tooling/Tooling.h>
#include <clang/Tooling/ArgumentsAdjusters.h>
#include <clang/Tooling/CompilationDatabase.h>

#include <llvm/Support/Path.h>
#include <llvm/ADT/Twine.h>

#include "LoopASTFrontendAction.h"

namespace clang {
namespace tooling {

class CheckerTool {
private:
  const CompilationDatabase& compilations;
  std::vector<std::string> sourcePaths;
  llvm::IntrusiveRefCntPtr<vfs::OverlayFileSystem> overlayFileSystem;
  llvm::IntrusiveRefCntPtr<vfs::InMemoryFileSystem> inMemoryFileSystem;
  llvm::IntrusiveRefCntPtr<FileManager> files;
  //llvm::StringSet<> seenWorkingDirectories;

  ArgumentsAdjuster argsAdjuster;
  DiagnosticConsumer* diagConsumer;

  std::map<std::string, std::string> origSourceToTmpSourceMap;

public:
  CheckerTool (const clang::tooling::CompilationDatabase &_compilations,
    llvm::ArrayRef<std::string> _sourcePaths);

  void appendArgumentsAdjuster(ArgumentsAdjuster Adjuster);
  void clearArgumentsAdjusters();
  
  bool run (std::unique_ptr<FrontendActionFactory>& _actions);
  bool runEmitLLVMOnlyAction();
};

} // namespace tooling
} // namespace clang
