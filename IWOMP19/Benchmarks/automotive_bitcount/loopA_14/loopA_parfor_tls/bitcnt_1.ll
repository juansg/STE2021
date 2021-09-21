; ModuleID = 'bitcnt_1.c'
source_filename = "bitcnt_1.c"
target datalayout = "e-m:e-i64:64-f80:128-n8:16:32:64-S128"
target triple = "x86_64-unknown-linux-gnu"

; Function Attrs: norecurse nounwind readnone uwtable
define i32 @bit_count(i64) local_unnamed_addr #0 {
  %2 = icmp eq i64 %0, 0
  br i1 %2, label %12, label %3

; <label>:3:                                      ; preds = %1
  br label %4

; <label>:4:                                      ; preds = %3, %4
  %5 = phi i64 [ %9, %4 ], [ %0, %3 ]
  %6 = phi i32 [ %7, %4 ], [ 0, %3 ]
  %7 = add nuw nsw i32 %6, 1
  %8 = add nsw i64 %5, -1
  %9 = and i64 %8, %5
  %10 = icmp eq i64 %9, 0
  br i1 %10, label %11, label %4

; <label>:11:                                     ; preds = %4
  br label %12

; <label>:12:                                     ; preds = %11, %1
  %13 = phi i32 [ 0, %1 ], [ %7, %11 ]
  ret i32 %13
}

attributes #0 = { norecurse nounwind readnone uwtable "disable-tail-calls"="false" "less-precise-fpmad"="false" "no-frame-pointer-elim"="false" "no-infs-fp-math"="false" "no-jump-tables"="false" "no-nans-fp-math"="false" "no-signed-zeros-fp-math"="false" "stack-protector-buffer-size"="8" "target-cpu"="x86-64" "target-features"="+fxsr,+mmx,+rtm,+sse,+sse2,+x87" "unsafe-fp-math"="false" "use-soft-float"="false" }

!llvm.ident = !{!0}

!0 = !{!"clang version 4.0.0 (http://gitlab.lsc.ic.unicamp.br/luis.mattos/DoAcrossClause-clang.git 9f12bb5a981b40920201c4dbfd7f8b1ba31bdd14) (http://gitlab.lsc.ic.unicamp.br/luis.mattos/DoAcrossClause.git 041019426da53cb3e8e9d731a855c63e9b22b05f)"}
