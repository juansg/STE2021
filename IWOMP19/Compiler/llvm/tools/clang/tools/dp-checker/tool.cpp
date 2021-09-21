#include <clang/Frontend/FrontendActions.h>

#include <llvm/Support/raw_ostream.h>
#include <llvm/Support/Signals.h>

// Declares llvm::cl::extrahelp
#include <llvm/Support/CommandLine.h>
#include <clang/Tooling/CommonOptionsParser.h>
#include <clang/Tooling/CompilationDatabase.h>
#include <clang/Tooling/Tooling.h>

//#include <llvm/IR/LLVMContext.h>

#include <vector>

#include "BracesFixerASTFrontendAction.h"
#include "LoopASTFrontendAction.h"
#include "PostLoopASTFrontendAction.h"
#include "CheckerTool.h"

using namespace clang;
using namespace clang::tooling;
using namespace llvm;

extern bool visitedFunctionsChanged;

// Apply a custom category to all command-line options so that they are the
// only ones displayed.
static cl::OptionCategory MyToolCategory("mytool options");

// CommonOptionsParser declares HelpMessage with a description of the common
// command-line options related to the compilation database and input files.
// It's nice to have this help message in all tools.
static cl::extrahelp CommonHelp(CommonOptionsParser::HelpMessage);

// A help message for this specific tool can be added afterwards.
static cl::extrahelp MoreHelp("\nMore help text...");

int main(int argc, const char **argv) {

  sys::PrintStackTraceOnErrorSignal(argv[0]);
  CommonOptionsParser OptionsParser(argc, argv, MyToolCategory);

  CheckerTool check(OptionsParser.getCompilations(),
      OptionsParser.getSourcePathList());

  std::vector<std::unique_ptr<FrontendActionFactory>> actions;
  actions.push_back(newFrontendActionFactory<BracesFixerASTFrontendAction>());
  actions.push_back(newFrontendActionFactory<LoopASTFrontendAction>());
  actions.push_back(newFrontendActionFactory<PostLoopASTFrontendAction>());

  for (std::unique_ptr<FrontendActionFactory>& factory : actions){
    if (int ret = check.run(factory)) {
      llvm::errs() << "error: failed to run front-end action!\n";
      return ret;
    }
  }

  actions.clear();
  actions.push_back(newFrontendActionFactory<PostLoopASTFrontendAction>());

  do {
    if (int ret = check.run(actions[0])) {
      llvm::errs() << "error: failed to run front-end action!\n";
      return ret;
    }
  } while (visitedFunctionsChanged);
  
  if ( check.runEmitLLVMOnlyAction() ) {
    llvm::errs() << "error: failed to run EmitLLVMOnlyAction action!\n";
    exit(EXIT_FAILURE);
  }
  
  return 0;
}
