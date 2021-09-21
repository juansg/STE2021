; ModuleID = 'bitarray.c'
source_filename = "bitarray.c"
target datalayout = "e-m:e-i64:64-f80:128-n8:16:32:64-S128"
target triple = "x86_64-unknown-linux-gnu"

; Function Attrs: nounwind uwtable
define noalias i8* @alloc_bit_array(i64) local_unnamed_addr #0 {
  %2 = add i64 %0, 7
  %3 = lshr i64 %2, 3
  %4 = tail call noalias i8* @calloc(i64 %3, i64 1) #4
  ret i8* %4
}

; Function Attrs: nounwind
declare noalias i8* @calloc(i64, i64) local_unnamed_addr #1

; Function Attrs: norecurse nounwind readonly uwtable
define i32 @getbit(i8* nocapture readonly, i32) local_unnamed_addr #2 {
  %3 = sdiv i32 %1, 8
  %4 = sext i32 %3 to i64
  %5 = getelementptr inbounds i8, i8* %0, i64 %4
  %6 = load i8, i8* %5, align 1, !tbaa !1
  %7 = sext i8 %6 to i32
  %8 = and i32 %1, 7
  %9 = shl i32 1, %8
  %10 = and i32 %7, %9
  %11 = icmp ne i32 %10, 0
  %12 = zext i1 %11 to i32
  ret i32 %12
}

; Function Attrs: norecurse nounwind uwtable
define void @setbit(i8* nocapture, i32, i32) local_unnamed_addr #3 {
  %4 = sdiv i32 %1, 8
  %5 = sext i32 %4 to i64
  %6 = getelementptr inbounds i8, i8* %0, i64 %5
  %7 = icmp eq i32 %2, 0
  %8 = and i32 %1, 7
  %9 = shl i32 1, %8
  %10 = load i8, i8* %6, align 1, !tbaa !1
  %11 = sext i8 %10 to i32
  %12 = xor i32 %9, 255
  %13 = and i32 %11, %12
  %14 = or i32 %11, %9
  %15 = select i1 %7, i32 %13, i32 %14
  %16 = trunc i32 %15 to i8
  store i8 %16, i8* %6, align 1, !tbaa !1
  ret void
}

; Function Attrs: norecurse nounwind uwtable
define void @flipbit(i8* nocapture, i32) local_unnamed_addr #3 {
  %3 = sdiv i32 %1, 8
  %4 = sext i32 %3 to i64
  %5 = getelementptr inbounds i8, i8* %0, i64 %4
  %6 = and i32 %1, 7
  %7 = shl i32 1, %6
  %8 = load i8, i8* %5, align 1, !tbaa !1
  %9 = zext i8 %8 to i32
  %10 = xor i32 %9, %7
  %11 = trunc i32 %10 to i8
  store i8 %11, i8* %5, align 1, !tbaa !1
  ret void
}

attributes #0 = { nounwind uwtable "disable-tail-calls"="false" "less-precise-fpmad"="false" "no-frame-pointer-elim"="false" "no-infs-fp-math"="false" "no-jump-tables"="false" "no-nans-fp-math"="false" "no-signed-zeros-fp-math"="false" "stack-protector-buffer-size"="8" "target-cpu"="x86-64" "target-features"="+fxsr,+mmx,+rtm,+sse,+sse2,+x87" "unsafe-fp-math"="false" "use-soft-float"="false" }
attributes #1 = { nounwind "disable-tail-calls"="false" "less-precise-fpmad"="false" "no-frame-pointer-elim"="false" "no-infs-fp-math"="false" "no-nans-fp-math"="false" "no-signed-zeros-fp-math"="false" "stack-protector-buffer-size"="8" "target-cpu"="x86-64" "target-features"="+fxsr,+mmx,+rtm,+sse,+sse2,+x87" "unsafe-fp-math"="false" "use-soft-float"="false" }
attributes #2 = { norecurse nounwind readonly uwtable "disable-tail-calls"="false" "less-precise-fpmad"="false" "no-frame-pointer-elim"="false" "no-infs-fp-math"="false" "no-jump-tables"="false" "no-nans-fp-math"="false" "no-signed-zeros-fp-math"="false" "stack-protector-buffer-size"="8" "target-cpu"="x86-64" "target-features"="+fxsr,+mmx,+rtm,+sse,+sse2,+x87" "unsafe-fp-math"="false" "use-soft-float"="false" }
attributes #3 = { norecurse nounwind uwtable "disable-tail-calls"="false" "less-precise-fpmad"="false" "no-frame-pointer-elim"="false" "no-infs-fp-math"="false" "no-jump-tables"="false" "no-nans-fp-math"="false" "no-signed-zeros-fp-math"="false" "stack-protector-buffer-size"="8" "target-cpu"="x86-64" "target-features"="+fxsr,+mmx,+rtm,+sse,+sse2,+x87" "unsafe-fp-math"="false" "use-soft-float"="false" }
attributes #4 = { nounwind }

!llvm.ident = !{!0}

!0 = !{!"clang version 4.0.0 (http://gitlab.lsc.ic.unicamp.br/luis.mattos/DoAcrossClause-clang.git 9f12bb5a981b40920201c4dbfd7f8b1ba31bdd14) (http://gitlab.lsc.ic.unicamp.br/luis.mattos/DoAcrossClause.git 041019426da53cb3e8e9d731a855c63e9b22b05f)"}
!1 = !{!2, !2, i64 0}
!2 = !{!"omnipotent char", !3, i64 0}
!3 = !{!"Simple C/C++ TBAA"}
