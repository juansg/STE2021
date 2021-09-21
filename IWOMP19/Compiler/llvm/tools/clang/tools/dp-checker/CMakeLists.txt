set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set( LLVM_LINK_COMPONENTS
  ${LLVM_TARGETS_TO_BUILD}
  Core
  Support
)

add_clang_executable(dp-checker
  tool.cpp
  CheckerTool.cpp
  BracesFixerASTFrontendAction.cpp
  BracesAroundStatementsCheck.cpp
  LoopASTFrontendAction.cpp
  PostLoopASTFrontendAction.cpp
)

target_link_libraries (dp-checker
  clangAST
  clangBasic
  clangDriver
  clangFrontend
  clangParse
  clangRewrite
  clangRewriteFrontend
  clangStaticAnalyzerFrontend
  clangTooling
  clangASTMatchers
  clangLex
  clangCodeGen
  #LLVMCore
  #LLVMSupport
)

install(TARGETS dp-checker
  RUNTIME DESTINATION bin)
