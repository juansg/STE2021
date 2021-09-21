; ModuleID = 'bitcnt_2.c'
source_filename = "bitcnt_2.c"
target datalayout = "e-m:e-i64:64-f80:128-n8:16:32:64-S128"
target triple = "x86_64-unknown-linux-gnu"

; Function Attrs: norecurse nounwind readnone uwtable
define i32 @bitcount(i64) local_unnamed_addr #0 {
  %2 = lshr i64 %0, 1
  %3 = and i64 %2, 1431655765
  %4 = and i64 %0, 1431655765
  %5 = add nuw nsw i64 %3, %4
  %6 = lshr i64 %5, 2
  %7 = and i64 %6, 858993459
  %8 = and i64 %5, 858993459
  %9 = add nuw nsw i64 %7, %8
  %10 = lshr i64 %9, 4
  %11 = and i64 %10, 117901063
  %12 = and i64 %9, 117901063
  %13 = add nuw nsw i64 %11, %12
  %14 = lshr i64 %13, 8
  %15 = and i64 %14, 983055
  %16 = and i64 %13, 983055
  %17 = add nuw nsw i64 %15, %16
  %18 = lshr i64 %17, 16
  %19 = and i64 %17, 31
  %20 = add nuw nsw i64 %18, %19
  %21 = trunc i64 %20 to i32
  ret i32 %21
}

attributes #0 = { norecurse nounwind readnone uwtable "disable-tail-calls"="false" "less-precise-fpmad"="false" "no-frame-pointer-elim"="false" "no-infs-fp-math"="false" "no-jump-tables"="false" "no-nans-fp-math"="false" "no-signed-zeros-fp-math"="false" "stack-protector-buffer-size"="8" "target-cpu"="x86-64" "target-features"="+fxsr,+mmx,+rtm,+sse,+sse2,+x87" "unsafe-fp-math"="false" "use-soft-float"="false" }

!llvm.ident = !{!0}

!0 = !{!"clang version 4.0.0 (http://gitlab.lsc.ic.unicamp.br/luis.mattos/DoAcrossClause-clang.git 9f12bb5a981b40920201c4dbfd7f8b1ba31bdd14) (http://gitlab.lsc.ic.unicamp.br/luis.mattos/DoAcrossClause.git 041019426da53cb3e8e9d731a855c63e9b22b05f)"}
