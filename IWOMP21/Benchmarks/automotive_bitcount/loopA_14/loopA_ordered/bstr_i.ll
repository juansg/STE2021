; ModuleID = 'bstr_i.c'
source_filename = "bstr_i.c"
target datalayout = "e-m:e-i64:64-f80:128-n8:16:32:64-S128"
target triple = "x86_64-unknown-linux-gnu"

; Function Attrs: norecurse nounwind readonly uwtable
define i32 @bstr_i(i8* readonly) local_unnamed_addr #0 {
  %2 = icmp eq i8* %0, null
  br i1 %2, label %28, label %3

; <label>:3:                                      ; preds = %1
  %4 = load i8, i8* %0, align 1, !tbaa !1
  %5 = icmp eq i8 %4, 0
  br i1 %5, label %28, label %6

; <label>:6:                                      ; preds = %3
  br label %7

; <label>:7:                                      ; preds = %6, %18
  %8 = phi i8 [ %24, %18 ], [ %4, %6 ]
  %9 = phi i8* [ %19, %18 ], [ %0, %6 ]
  %10 = phi i32 [ %23, %18 ], [ 0, %6 ]
  %11 = sext i8 %8 to i64
  %12 = and i64 %11, 4294967295
  %13 = icmp ult i64 %12, 64
  %14 = shl i64 1, %12
  %15 = and i64 %14, 844424930131969
  %16 = icmp ne i64 %15, 0
  %17 = and i1 %13, %16
  br i1 %17, label %18, label %26

; <label>:18:                                     ; preds = %7
  %19 = getelementptr inbounds i8, i8* %9, i64 1
  %20 = zext i8 %8 to i32
  %21 = shl i32 %10, 1
  %22 = and i32 %20, 1
  %23 = or i32 %22, %21
  %24 = load i8, i8* %19, align 1, !tbaa !1
  %25 = icmp eq i8 %24, 0
  br i1 %25, label %26, label %7

; <label>:26:                                     ; preds = %18, %7
  %27 = phi i32 [ %23, %18 ], [ %10, %7 ]
  br label %28

; <label>:28:                                     ; preds = %26, %3, %1
  %29 = phi i32 [ 0, %1 ], [ 0, %3 ], [ %27, %26 ]
  ret i32 %29
}

attributes #0 = { norecurse nounwind readonly uwtable "disable-tail-calls"="false" "less-precise-fpmad"="false" "no-frame-pointer-elim"="false" "no-infs-fp-math"="false" "no-jump-tables"="false" "no-nans-fp-math"="false" "no-signed-zeros-fp-math"="false" "stack-protector-buffer-size"="8" "target-cpu"="x86-64" "target-features"="+fxsr,+mmx,+rtm,+sse,+sse2,+x87" "unsafe-fp-math"="false" "use-soft-float"="false" }

!llvm.ident = !{!0}

!0 = !{!"clang version 4.0.0 (http://gitlab.lsc.ic.unicamp.br/luis.mattos/DoAcrossClause-clang.git 9f12bb5a981b40920201c4dbfd7f8b1ba31bdd14) (http://gitlab.lsc.ic.unicamp.br/luis.mattos/DoAcrossClause.git 041019426da53cb3e8e9d731a855c63e9b22b05f)"}
!1 = !{!2, !2, i64 0}
!2 = !{!"omnipotent char", !3, i64 0}
!3 = !{!"Simple C/C++ TBAA"}
