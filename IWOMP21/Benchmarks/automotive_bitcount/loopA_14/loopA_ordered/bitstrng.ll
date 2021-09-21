; ModuleID = 'bitstrng.c'
source_filename = "bitstrng.c"
target datalayout = "e-m:e-i64:64-f80:128-n8:16:32:64-S128"
target triple = "x86_64-unknown-linux-gnu"

; Function Attrs: norecurse nounwind uwtable
define void @bitstring(i8* nocapture, i64, i32, i32) local_unnamed_addr #0 {
  %5 = ashr i32 %2, 2
  %6 = and i32 %2, 3
  %7 = icmp eq i32 %6, 0
  %8 = zext i1 %7 to i32
  %9 = add i32 %5, %2
  %10 = sub i32 %3, %9
  %11 = add i32 %10, %8
  %12 = icmp sgt i32 %11, 0
  br i1 %12, label %13, label %20

; <label>:13:                                     ; preds = %4
  %14 = add i32 %8, %3
  %15 = add i32 %14, -1
  %16 = sub i32 %15, %9
  %17 = zext i32 %16 to i64
  %18 = add nuw nsw i64 %17, 1
  call void @llvm.memset.p0i8.i64(i8* %0, i8 32, i64 %18, i32 1, i1 false)
  %19 = getelementptr i8, i8* %0, i64 %18
  br label %20

; <label>:20:                                     ; preds = %13, %4
  %21 = phi i8* [ %0, %4 ], [ %19, %13 ]
  %22 = icmp sgt i32 %2, 0
  br i1 %22, label %23, label %47

; <label>:23:                                     ; preds = %20
  %24 = zext i32 %2 to i64
  br label %25

; <label>:25:                                     ; preds = %23, %40
  %26 = phi i64 [ %24, %23 ], [ %43, %40 ]
  %27 = phi i8* [ %21, %23 ], [ %41, %40 ]
  %28 = trunc i64 %26 to i32
  %29 = add nsw i32 %28, -1
  %30 = zext i32 %29 to i64
  %31 = lshr i64 %1, %30
  %32 = and i64 %31, 1
  %33 = or i64 %32, 48
  %34 = trunc i64 %33 to i8
  %35 = getelementptr inbounds i8, i8* %27, i64 1
  store i8 %34, i8* %27, align 1, !tbaa !1
  %36 = and i32 %29, 3
  %37 = icmp eq i32 %36, 0
  %38 = icmp ne i32 %29, 0
  %39 = and i1 %38, %37
  br i1 %39, label %44, label %40

; <label>:40:                                     ; preds = %25, %44
  %41 = phi i8* [ %45, %44 ], [ %35, %25 ]
  %42 = icmp sgt i32 %28, 1
  %43 = add nsw i64 %26, -1
  br i1 %42, label %25, label %46

; <label>:44:                                     ; preds = %25
  %45 = getelementptr inbounds i8, i8* %27, i64 2
  store i8 32, i8* %35, align 1, !tbaa !1
  br label %40

; <label>:46:                                     ; preds = %40
  br label %47

; <label>:47:                                     ; preds = %46, %20
  %48 = phi i8* [ %21, %20 ], [ %41, %46 ]
  store i8 0, i8* %48, align 1, !tbaa !1
  ret void
}

; Function Attrs: argmemonly nounwind
declare void @llvm.memset.p0i8.i64(i8* nocapture writeonly, i8, i64, i32, i1) #1

attributes #0 = { norecurse nounwind uwtable "disable-tail-calls"="false" "less-precise-fpmad"="false" "no-frame-pointer-elim"="false" "no-infs-fp-math"="false" "no-jump-tables"="false" "no-nans-fp-math"="false" "no-signed-zeros-fp-math"="false" "stack-protector-buffer-size"="8" "target-cpu"="x86-64" "target-features"="+fxsr,+mmx,+rtm,+sse,+sse2,+x87" "unsafe-fp-math"="false" "use-soft-float"="false" }
attributes #1 = { argmemonly nounwind }

!llvm.ident = !{!0}

!0 = !{!"clang version 4.0.0 (http://gitlab.lsc.ic.unicamp.br/luis.mattos/DoAcrossClause-clang.git 9f12bb5a981b40920201c4dbfd7f8b1ba31bdd14) (http://gitlab.lsc.ic.unicamp.br/luis.mattos/DoAcrossClause.git 041019426da53cb3e8e9d731a855c63e9b22b05f)"}
!1 = !{!2, !2, i64 0}
!2 = !{!"omnipotent char", !3, i64 0}
!3 = !{!"Simple C/C++ TBAA"}
