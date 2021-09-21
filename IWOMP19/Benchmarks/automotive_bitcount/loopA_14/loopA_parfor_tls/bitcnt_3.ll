; ModuleID = 'bitcnt_3.c'
source_filename = "bitcnt_3.c"
target datalayout = "e-m:e-i64:64-f80:128-n8:16:32:64-S128"
target triple = "x86_64-unknown-linux-gnu"

@bits = internal unnamed_addr constant [256 x i8] c"\00\01\01\02\01\02\02\03\01\02\02\03\02\03\03\04\01\02\02\03\02\03\03\04\02\03\03\04\03\04\04\05\01\02\02\03\02\03\03\04\02\03\03\04\03\04\04\05\02\03\03\04\03\04\04\05\03\04\04\05\04\05\05\06\01\02\02\03\02\03\03\04\02\03\03\04\03\04\04\05\02\03\03\04\03\04\04\05\03\04\04\05\04\05\05\06\02\03\03\04\03\04\04\05\03\04\04\05\04\05\05\06\03\04\04\05\04\05\05\06\04\05\05\06\05\06\06\07\01\02\02\03\02\03\03\04\02\03\03\04\03\04\04\05\02\03\03\04\03\04\04\05\03\04\04\05\04\05\05\06\02\03\03\04\03\04\04\05\03\04\04\05\04\05\05\06\03\04\04\05\04\05\05\06\04\05\05\06\05\06\06\07\02\03\03\04\03\04\04\05\03\04\04\05\04\05\05\06\03\04\04\05\04\05\05\06\04\05\05\06\05\06\06\07\03\04\04\05\04\05\05\06\04\05\05\06\05\06\06\07\04\05\05\06\05\06\06\07\05\06\06\07\06\07\07\08", align 16

; Function Attrs: norecurse nounwind readnone uwtable
define i32 @ntbl_bitcount(i64) local_unnamed_addr #0 {
  %2 = and i64 %0, 15
  %3 = getelementptr inbounds [256 x i8], [256 x i8]* @bits, i64 0, i64 %2
  %4 = load i8, i8* %3, align 1, !tbaa !1
  %5 = sext i8 %4 to i32
  %6 = lshr i64 %0, 4
  %7 = and i64 %6, 15
  %8 = getelementptr inbounds [256 x i8], [256 x i8]* @bits, i64 0, i64 %7
  %9 = load i8, i8* %8, align 1, !tbaa !1
  %10 = sext i8 %9 to i32
  %11 = add nsw i32 %10, %5
  %12 = lshr i64 %0, 8
  %13 = and i64 %12, 15
  %14 = getelementptr inbounds [256 x i8], [256 x i8]* @bits, i64 0, i64 %13
  %15 = load i8, i8* %14, align 1, !tbaa !1
  %16 = sext i8 %15 to i32
  %17 = add nsw i32 %11, %16
  %18 = lshr i64 %0, 12
  %19 = and i64 %18, 15
  %20 = getelementptr inbounds [256 x i8], [256 x i8]* @bits, i64 0, i64 %19
  %21 = load i8, i8* %20, align 1, !tbaa !1
  %22 = sext i8 %21 to i32
  %23 = add nsw i32 %17, %22
  %24 = lshr i64 %0, 16
  %25 = and i64 %24, 15
  %26 = getelementptr inbounds [256 x i8], [256 x i8]* @bits, i64 0, i64 %25
  %27 = load i8, i8* %26, align 1, !tbaa !1
  %28 = sext i8 %27 to i32
  %29 = add nsw i32 %23, %28
  %30 = lshr i64 %0, 20
  %31 = and i64 %30, 15
  %32 = getelementptr inbounds [256 x i8], [256 x i8]* @bits, i64 0, i64 %31
  %33 = load i8, i8* %32, align 1, !tbaa !1
  %34 = sext i8 %33 to i32
  %35 = add nsw i32 %29, %34
  %36 = lshr i64 %0, 24
  %37 = and i64 %36, 15
  %38 = getelementptr inbounds [256 x i8], [256 x i8]* @bits, i64 0, i64 %37
  %39 = load i8, i8* %38, align 1, !tbaa !1
  %40 = sext i8 %39 to i32
  %41 = add nsw i32 %35, %40
  %42 = lshr i64 %0, 28
  %43 = and i64 %42, 15
  %44 = getelementptr inbounds [256 x i8], [256 x i8]* @bits, i64 0, i64 %43
  %45 = load i8, i8* %44, align 1, !tbaa !1
  %46 = sext i8 %45 to i32
  %47 = add nsw i32 %41, %46
  ret i32 %47
}

; Function Attrs: norecurse nounwind readnone uwtable
define i32 @BW_btbl_bitcount(i64) local_unnamed_addr #0 {
  %2 = lshr i64 %0, 8
  %3 = lshr i64 %0, 16
  %4 = lshr i64 %0, 24
  %5 = and i64 %0, 255
  %6 = getelementptr inbounds [256 x i8], [256 x i8]* @bits, i64 0, i64 %5
  %7 = load i8, i8* %6, align 1, !tbaa !1
  %8 = sext i8 %7 to i32
  %9 = and i64 %2, 255
  %10 = getelementptr inbounds [256 x i8], [256 x i8]* @bits, i64 0, i64 %9
  %11 = load i8, i8* %10, align 1, !tbaa !1
  %12 = sext i8 %11 to i32
  %13 = add nsw i32 %12, %8
  %14 = and i64 %4, 255
  %15 = getelementptr inbounds [256 x i8], [256 x i8]* @bits, i64 0, i64 %14
  %16 = load i8, i8* %15, align 1, !tbaa !1
  %17 = sext i8 %16 to i32
  %18 = add nsw i32 %13, %17
  %19 = and i64 %3, 255
  %20 = getelementptr inbounds [256 x i8], [256 x i8]* @bits, i64 0, i64 %19
  %21 = load i8, i8* %20, align 1, !tbaa !1
  %22 = sext i8 %21 to i32
  %23 = add nsw i32 %18, %22
  ret i32 %23
}

; Function Attrs: norecurse nounwind readnone uwtable
define i32 @AR_btbl_bitcount(i64) local_unnamed_addr #0 {
  %2 = lshr i64 %0, 8
  %3 = lshr i64 %0, 16
  %4 = lshr i64 %0, 24
  %5 = and i64 %0, 255
  %6 = getelementptr inbounds [256 x i8], [256 x i8]* @bits, i64 0, i64 %5
  %7 = load i8, i8* %6, align 1, !tbaa !1
  %8 = sext i8 %7 to i32
  %9 = and i64 %2, 255
  %10 = getelementptr inbounds [256 x i8], [256 x i8]* @bits, i64 0, i64 %9
  %11 = load i8, i8* %10, align 1, !tbaa !1
  %12 = sext i8 %11 to i32
  %13 = add nsw i32 %12, %8
  %14 = and i64 %3, 255
  %15 = getelementptr inbounds [256 x i8], [256 x i8]* @bits, i64 0, i64 %14
  %16 = load i8, i8* %15, align 1, !tbaa !1
  %17 = sext i8 %16 to i32
  %18 = add nsw i32 %13, %17
  %19 = and i64 %4, 255
  %20 = getelementptr inbounds [256 x i8], [256 x i8]* @bits, i64 0, i64 %19
  %21 = load i8, i8* %20, align 1, !tbaa !1
  %22 = sext i8 %21 to i32
  %23 = add nsw i32 %18, %22
  ret i32 %23
}

attributes #0 = { norecurse nounwind readnone uwtable "disable-tail-calls"="false" "less-precise-fpmad"="false" "no-frame-pointer-elim"="false" "no-infs-fp-math"="false" "no-jump-tables"="false" "no-nans-fp-math"="false" "no-signed-zeros-fp-math"="false" "stack-protector-buffer-size"="8" "target-cpu"="x86-64" "target-features"="+fxsr,+mmx,+rtm,+sse,+sse2,+x87" "unsafe-fp-math"="false" "use-soft-float"="false" }

!llvm.ident = !{!0}

!0 = !{!"clang version 4.0.0 (http://gitlab.lsc.ic.unicamp.br/luis.mattos/DoAcrossClause-clang.git 9f12bb5a981b40920201c4dbfd7f8b1ba31bdd14) (http://gitlab.lsc.ic.unicamp.br/luis.mattos/DoAcrossClause.git 041019426da53cb3e8e9d731a855c63e9b22b05f)"}
!1 = !{!2, !2, i64 0}
!2 = !{!"omnipotent char", !3, i64 0}
!3 = !{!"Simple C/C++ TBAA"}
