//===--- BracesAroundStatementsCheck.h - clang-tidy -------------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_CLANG_TOOLS_EXTRA_CLANG_TIDY_READABILITY_BRACESAROUNDSTATEMENTSCHECK_H
#define LLVM_CLANG_TOOLS_EXTRA_CLANG_TIDY_READABILITY_BRACESAROUNDSTATEMENTSCHECK_H

#include <clang/ASTMatchers/ASTMatchFinder.h>
#include <clang/ASTMatchers/ASTMatchers.h>

#include <clang/Rewrite/Core/Rewriter.h>

#include <set>

namespace clang {
namespace tidy {
namespace readability {

/// Checks that bodies of `if` statements and loops (`for`, `range-for`,
/// `do-while`, and `while`) are inside braces
///
/// Before:
///
/// \code
///   if (condition)
///     statement;
/// \endcode
///
/// After:
///
/// \code
///   if (condition) {
///     statement;
///   }
/// \endcode
///
/// Additionally, one can define an option `ShortStatementLines` defining the
/// minimal number of lines that the statement should have in order to trigger
/// this check.
///
/// The number of lines is counted from the end of condition or initial keyword
/// (`do`/`else`) until the last line of the inner statement.  Default value 0
/// means that braces will be added to all statements (not having them already).
//class BracesAroundStatementsCheck : public ClangTidyCheck {
class BracesAroundStatementsCheck : public ast_matchers::MatchFinder::MatchCallback {
public:
  //BracesAroundStatementsCheck(StringRef Name, ClangTidyContext *Context);
  BracesAroundStatementsCheck(Rewriter& R) : TheRewriter(R) {}
  //void storeOptions(ClangTidyOptions::OptionMap &Opts) override;
  //void registerMatchers(ast_matchers::MatchFinder *Finder) override;
  void registerMatchers(ast_matchers::MatchFinder *Finder);
  //void check(const ast_matchers::MatchFinder::MatchResult &Result) override;
  void run(const ast_matchers::MatchFinder::MatchResult &Result) override;
  //void onEndOfTranslationUnit() override;

private:
  bool checkStmt(const ast_matchers::MatchFinder::MatchResult &Result,
                 const Stmt *S, SourceLocation StartLoc,
                 SourceLocation EndLocHint = SourceLocation());
  template <typename IfOrWhileStmt>
  SourceLocation findRParenLoc(const IfOrWhileStmt *S, const SourceManager &SM,
                               const ASTContext *Context);

private:
  Rewriter& TheRewriter;
  std::set<const Stmt *> ForceBracesStmts;
  //const unsigned ShortStatementLines;
};

} // namespace readability
} // namespace tidy
} // namespace clang

#endif // LLVM_CLANG_TOOLS_EXTRA_CLANG_TIDY_READABILITY_BRACESAROUNDSTATEMENTSCHECK_H
