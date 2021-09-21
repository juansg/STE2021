#include "CheckerTool.h"

#include <clang/Lex/Preprocessor.h>
#include <clang/Frontend/CompilerInstance.h>

#include "clang/Driver/Tool.h"
#include <clang/Driver/Driver.h>
#include <clang/Driver/Compilation.h>

#include <clang/Frontend/FrontendDiagnostic.h>
#include <clang/Frontend/TextDiagnosticPrinter.h>
#include <clang/Basic/LangOptions.h>
#include <clang/Basic/TargetInfo.h>
#include <clang/Basic/TargetOptions.h>

#include <llvm/IR/Module.h>
#include <llvm/IR/LLVMContext.h>

#include <llvm/Support/FileSystem.h>

#include <regex>

namespace clang {
namespace tooling {

  CheckerTool::CheckerTool (const CompilationDatabase &_compilations,
      llvm::ArrayRef<std::string> _sourcePaths)
    : compilations(_compilations), sourcePaths(_sourcePaths),
      overlayFileSystem(new vfs::OverlayFileSystem(vfs::getRealFileSystem())),
      inMemoryFileSystem(new vfs::InMemoryFileSystem),
      files(new FileManager(FileSystemOptions(), overlayFileSystem)),
      diagConsumer(nullptr) {
    overlayFileSystem->pushOverlay(inMemoryFileSystem);
    appendArgumentsAdjuster(getClangStripOutputAdjuster());
    //appendArgumentsAdjuster(getClangSyntaxOnlyAdjuster());
  }

  void CheckerTool::appendArgumentsAdjuster(ArgumentsAdjuster Adjuster) {
    // clang 4.X -- combineAdjusters do not check if either lambda is empty
    if (argsAdjuster == nullptr) {
      argsAdjuster = std::move(Adjuster);
    } else {
      argsAdjuster = combineAdjusters(std::move(argsAdjuster), std::move(Adjuster));
    }
  }

  void CheckerTool::clearArgumentsAdjusters() {
    argsAdjuster = nullptr;
  }

  static std::string getAbsolutePathWithoutDots(StringRef filePath) {
    std::string path = getAbsolutePath(filePath);
    char *fullpath = realpath(path.c_str(), nullptr);
    if (fullpath == nullptr){
      perror("real_path");
      llvm::errs() << "could not open file '" << filePath << "'\n";
      exit(EXIT_FAILURE);
    }
    //llvm::errs() << "path before: " << path << '\n';
    //llvm::errs() << "path after : " << fullpath << '\n';
    return std::string(fullpath);
  }

  static void injectResourceDir(std::vector<std::string> &Args,
      const char *Argv0, void *MainAddr) {
    // Allow users to override the resource dir.
    for (StringRef Arg : Args)
      if (Arg.startswith("-resource-dir"))
        return;

    // If there's no override in place add our resource dir.
    Args.push_back("-resource-dir=" +
      getAbsolutePathWithoutDots(
        CompilerInvocation::GetResourcesPath(Argv0, MainAddr)));
  }

  static std::string getClangIncludePath(int* StaticSymbol) {
    std::string resourcesPath =
      CompilerInvocation::GetResourcesPath("clang_tool", StaticSymbol);
    return getAbsolutePathWithoutDots(resourcesPath+"/../../../include");
  }

  ArgumentsAdjuster getObjectOutputToIROutputAdjuster() {
    return [](const CommandLineArguments &Args, StringRef /*unused*/) {
      CommandLineArguments AdjustedArgs;
      for (size_t i = 0, e = Args.size(); i < e; ++i) {
        StringRef Arg = Args[i];
        if (!Arg.startswith("-o"))
          AdjustedArgs.push_back(Args[i]);

        if (Arg == "-o") {
          AdjustedArgs.push_back("-o");
          ++i;
          size_t lastPointPosition = Args[i].rfind(".");
          std::string prefix(Args[i].substr(0,
            lastPointPosition));
          //llvm::errs() << "prefix: " << prefix << '\n';
          AdjustedArgs.push_back(std::string(prefix + ".ll"));
        }
        // Else, the output is specified as -ofoo. Just do nothing.
      }
      return AdjustedArgs;
    };
  }

  bool CheckerTool::runEmitLLVMOnlyAction() {
    // Exists solely for the purpose of lookup fo the resource path.
    // This just need to be some symbol in the binary.
    static int StaticSymbol;

    llvm::SmallString<128> InitialDirectory;
    if ( std::error_code EC = llvm::sys::fs::current_path(InitialDirectory) ) {
      llvm::report_fatal_error("Cannot detect path: " + llvm::Twine(EC.message()));
    }

    bool ProcessingFailed = false;
    bool FileSkipped = false;
    for (llvm::StringRef sourcePath : sourcePaths) {
      std::string absolutePathSourceFile(getAbsolutePathWithoutDots(sourcePath));

      std::vector<CompileCommand> compileCommandsForFile =
        compilations.getCompileCommands(absolutePathSourceFile);
      if (compileCommandsForFile.empty()) {
        llvm::errs() << "Skipping " << absolutePathSourceFile <<
          ". Compile command not file." << '\n';
        FileSkipped = true;
        continue;
      }

      std::string tmpSourceFile(origSourceToTmpSourceMap[sourcePath]);

      if (tmpSourceFile.empty()) {
        llvm::errs() << "error: runEmitLLVMOnlyAction must be called only after front-end actions\n";
        return false;
      }

      clearArgumentsAdjusters();
      //appendArgumentsAdjuster(getClangStripOutputAdjuster());
      appendArgumentsAdjuster(getObjectOutputToIROutputAdjuster());

      //llvm::errs() << "tmp source: " << tmpSourceFile << '\n';
      for (CompileCommand& compileCommand : compileCommandsForFile) {
        std::vector<std::string> commandLine = compileCommand.CommandLine;
        if (argsAdjuster) {
          commandLine = argsAdjuster(commandLine, tmpSourceFile);
        }

        std::vector<const char*> newCommandLine;
        for (const std::string& arg : commandLine) {
          if ( arg.compare("-c") == 0) continue;
          if ( arg.compare(absolutePathSourceFile) == 0 ) {
            // replace original source file path with the temporary one
            newCommandLine.push_back(tmpSourceFile.c_str());
          } else {
            newCommandLine.push_back(arg.c_str());
          }
        }

        std::string clangIncludePathOption = std::string("-I"+getClangIncludePath(&StaticSymbol));
        newCommandLine.push_back(clangIncludePathOption.c_str());
        newCommandLine.push_back("-Xclang");
        newCommandLine.push_back("-omp-check-instrumentation");
        newCommandLine.push_back("-g");
        newCommandLine.push_back("-S");
        newCommandLine.push_back("-emit-llvm");

        // Add the resource dir based on the binary of this tool. argv[0] in the
        // compilation database may refer to a different compiler and we want to
        // pick up the very same standard library that compiler is using. The
        // builtin headers in the resource dir need to match the exact clang
        // version the tool is using.
        // FIXME: On linux, GetMainExecutable is independent of the value of the
        // first argument, thus allowing ClangTool and runToolOnCode to just
        // pass in made-up names here. Make sure this works on other platforms.
        //injectResourceDir(newCommandLine, "clang_tool", &StaticSymbol);

        //for (const std::string& arg : newCommandLine) {
        //  llvm::errs() << "arg: " << arg << '\n';
        //}

        // The compiler invocation needs a DiagnosticsEngine so it can report problems
        DiagnosticOptions* DiagOptions = new DiagnosticOptions();
        DiagnosticConsumer *DiagClient =
          new TextDiagnosticPrinter(llvm::errs(), DiagOptions);
        llvm::IntrusiveRefCntPtr<clang::DiagnosticIDs> DiagID(new clang::DiagnosticIDs());
        DiagnosticsEngine Diags(DiagID, DiagOptions, DiagClient);

        driver::Driver TheDriver(newCommandLine[0],
          llvm::sys::getDefaultTargetTriple(),
          Diags, files->getVirtualFileSystem());
        const std::unique_ptr<driver::Compilation> compilation(TheDriver.BuildCompilation(newCommandLine));
        if (!compilation) return false;
        llvm::SmallVector<std::pair<int, const driver::Command*>,10> failingCommands;
        int res = TheDriver.ExecuteCompilation(*compilation, failingCommands);
        if (res < 0) {
          ProcessingFailed = true;
          for (const std::pair<int, const driver::Command*>& p : failingCommands) {
            if (p.first) {
              TheDriver.generateCompilationDiagnostics(*compilation, *p.second);
            }
          }
        }
      }
    }
    return ProcessingFailed ? 1 : (FileSkipped ? 2 : 0);

  }
  bool CheckerTool::run (std::unique_ptr<FrontendActionFactory>& actionFactory) {
    // Exists solely for the purpose of lookup fo the resource path.
    // This just need to be some symbol in the binary.
    static int StaticSymbol;

    llvm::SmallString<128> InitialDirectory;
    if ( std::error_code EC = llvm::sys::fs::current_path(InitialDirectory) ) {
      llvm::report_fatal_error("Cannot detect path: " + llvm::Twine(EC.message()));
    }

    bool ProcessingFailed = false;
    bool FileSkipped = false;
    for (llvm::StringRef sourcePath : sourcePaths) {
      std::string absolutePathSourceFile(getAbsolutePathWithoutDots(sourcePath));

      std::vector<CompileCommand> compileCommandsForFile =
        compilations.getCompileCommands(absolutePathSourceFile);
      if (compileCommandsForFile.empty()) {
        llvm::errs() << "Skipping " << absolutePathSourceFile <<
          ". Compile command not found." << '\n';
        FileSkipped = true;
        continue;
      }
      
      std::string tmpSourceFile(origSourceToTmpSourceMap[sourcePath]);
      size_t lastSlashPosition = absolutePathSourceFile.rfind("/");
      std::string BasePath(absolutePathSourceFile.substr(0,lastSlashPosition));

      if (tmpSourceFile.empty()) {
        size_t lastPointPosition = absolutePathSourceFile.rfind(".");
        std::string prefix(absolutePathSourceFile.substr(lastSlashPosition+1,
              lastPointPosition-(lastSlashPosition+1)));
        std::string suffix(absolutePathSourceFile.substr(
              lastPointPosition+1, std::string::npos));
        //llvm::errs() << "prefix: " << prefix << '\n';
        //llvm::errs() << "suffix: " << suffix << '\n';

        std::string resultPath;
        for (CompileCommand& compileCommand : compileCommandsForFile) {
          std::vector<std::string> commandLine = compileCommand.CommandLine;
          bool foundOutputFlag = false;
          for (size_t i = 0; i < commandLine.size(); i++) {
            StringRef Arg = commandLine[i];
            if (Arg == "-o") {
              ++i;
              size_t lastPointPosition = commandLine[i].rfind(".");
              size_t secondLastPointPosition =
                commandLine[i].substr(0, lastPointPosition-1).rfind(".");
              //llvm::errs() << "secondLastPointPosition "
              //  << secondLastPointPosition << '\n';
              std::string prefix(commandLine[i].substr(0,
                    secondLastPointPosition));
              //llvm::errs() << "prefix: " << prefix << '\n';
              //llvm::errs() << "suffix: " << suffix << '\n';
              resultPath = std::string(prefix + "-inst." + suffix);
              foundOutputFlag = true;
              break;
            }
          }
          if (!foundOutputFlag) {
            llvm::errs() << "warning: could not find output flag '-o' on '"
              << absolutePathSourceFile << "'s compilation database\n";
            llvm::errs() << "Creating generic output ";
	    resultPath = std::string(prefix + "-inst." + suffix);
            compileCommand.CommandLine.push_back("-o");
	    compileCommand.CommandLine.push_back(resultPath);
          }
        }
        tmpSourceFile = std::string(resultPath.c_str());
        llvm::sys::fs::copy_file(absolutePathSourceFile,
            tmpSourceFile);

        origSourceToTmpSourceMap[sourcePath] = tmpSourceFile;
      }

      //llvm::errs() << "tmp source: " << tmpSourceFile << '\n';
      for (CompileCommand& compileCommand : compileCommandsForFile) {
        std::vector<std::string> commandLine = compileCommand.CommandLine;
        if (argsAdjuster) {
          commandLine = argsAdjuster(commandLine, tmpSourceFile);
        }
        std::vector<std::string> newCommandLine;
        for (std::string arg : commandLine) {
          if ( arg.compare(absolutePathSourceFile) == 0 ) {
            // replace original source file path with the temporary one
            newCommandLine.push_back(tmpSourceFile);
          } else {
            newCommandLine.push_back(arg);
          }
        }

        newCommandLine.push_back(std::string("-I"+getClangIncludePath(&StaticSymbol)));
        newCommandLine.push_back(std::string("-I"+BasePath));

        // Add the resource dir based on the binary of this tool. argv[0] in the
        // compilation database may refer to a different compiler and we want to
        // pick up the very same standard library that compiler is using. The
        // builtin headers in the resource dir need to match the exact clang
        // version the tool is using.
        // FIXME: On linux, GetMainExecutable is independent of the value of the
        // first argument, thus allowing ClangTool and runToolOnCode to just
        // pass in made-up names here. Make sure this works on other platforms.
        injectResourceDir(newCommandLine, "clang_tool", &StaticSymbol);

        //for (const std::string& arg : newCommandLine) {
        //  llvm::errs() << "arg: " << arg << '\n';
        //}

        ToolInvocation toolInvocation(newCommandLine,
            actionFactory->create(), files.get());
        toolInvocation.setDiagnosticConsumer(diagConsumer);
        if ( ! toolInvocation.run() ) {
          llvm::errs() << "Error while processing " << absolutePathSourceFile
            << "." << '\n';
          ProcessingFailed = true;
        } else {
          // recreate FileManager to reset internal state
          files = new FileManager(FileSystemOptions(), overlayFileSystem);
        }
      }
    }
    return ProcessingFailed ? 1 : (FileSkipped ? 2 : 0);
  }
} // namespace tooling
} // namespace clang
