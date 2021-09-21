; ModuleID = 'loop-wrap.c'
source_filename = "loop-wrap.c"
target datalayout = "e-m:e-i64:64-f80:128-n8:16:32:64-S128"
target triple = "x86_64-unknown-linux-gnu"

%struct._IO_FILE = type { i32, i8*, i8*, i8*, i8*, i8*, i8*, i8*, i8*, i8*, i8*, i8*, %struct._IO_marker*, %struct._IO_FILE*, i32, i32, i64, i16, i8, [1 x i8], i8*, i64, i8*, i8*, i8*, i8*, i64, i32, [20 x i8] }
%struct._IO_marker = type { %struct._IO_marker*, %struct._IO_FILE*, i32 }
%struct.timeval = type { i64, i64 }
%struct.timezone = type { i32, i32 }

@.str = private unnamed_addr constant [15 x i8] c"_finfo_dataset\00", align 1
@.str.1 = private unnamed_addr constant [3 x i8] c"rt\00", align 1
@stderr = external local_unnamed_addr global %struct._IO_FILE*, align 8
@.str.2 = private unnamed_addr constant [29 x i8] c"\0AError: Can't find dataset!\0A\00", align 1
@.str.3 = private unnamed_addr constant [4 x i8] c"%ld\00", align 1
@.str.4 = private unnamed_addr constant [11 x i8] c"Timer %lf\0A\00", align 1

; Function Attrs: nounwind uwtable
define i32 @main(i32, i8**) local_unnamed_addr #0 {
  %3 = alloca %struct.timeval, align 8
  %4 = alloca %struct.timeval, align 8
  %5 = alloca i64, align 8
  %6 = bitcast %struct.timeval* %3 to i8*
  call void @llvm.lifetime.start(i64 16, i8* %6) #4
  %7 = bitcast %struct.timeval* %4 to i8*
  call void @llvm.lifetime.start(i64 16, i8* %7) #4
  %8 = bitcast i64* %5 to i8*
  call void @llvm.lifetime.start(i64 8, i8* %8) #4
  %9 = call i32 @gettimeofday(%struct.timeval* nonnull %3, %struct.timezone* null) #4
  %10 = tail call %struct._IO_FILE* @fopen(i8* getelementptr inbounds ([15 x i8], [15 x i8]* @.str, i64 0, i64 0), i8* getelementptr inbounds ([3 x i8], [3 x i8]* @.str.1, i64 0, i64 0))
  %11 = icmp eq %struct._IO_FILE* %10, null
  br i1 %11, label %12, label %15

; <label>:12:                                     ; preds = %2
  %13 = load %struct._IO_FILE*, %struct._IO_FILE** @stderr, align 8, !tbaa !1
  %14 = tail call i64 @fwrite(i8* getelementptr inbounds ([29 x i8], [29 x i8]* @.str.2, i64 0, i64 0), i64 28, i64 1, %struct._IO_FILE* %13) #5
  br label %48

; <label>:15:                                     ; preds = %2
  %16 = call i32 (%struct._IO_FILE*, i8*, ...) @__isoc99_fscanf(%struct._IO_FILE* nonnull %10, i8* getelementptr inbounds ([4 x i8], [4 x i8]* @.str.3, i64 0, i64 0), i64* nonnull %5) #4
  %17 = call i32 @fclose(%struct._IO_FILE* nonnull %10)
  %18 = load i64, i64* %5, align 8, !tbaa !5
  %19 = icmp sgt i64 %18, 0
  br i1 %19, label %20, label %31

; <label>:20:                                     ; preds = %15
  br label %21

; <label>:21:                                     ; preds = %20, %21
  %22 = phi i64 [ %28, %21 ], [ %18, %20 ]
  %23 = phi i64 [ %24, %21 ], [ 0, %20 ]
  %24 = add nuw nsw i64 %23, 1
  %25 = icmp eq i64 %24, %22
  %26 = zext i1 %25 to i32
  %27 = call i32 @main1(i32 %0, i8** %1, i32 %26) #4
  %28 = load i64, i64* %5, align 8, !tbaa !5
  %29 = icmp slt i64 %24, %28
  br i1 %29, label %21, label %30

; <label>:30:                                     ; preds = %21
  br label %31

; <label>:31:                                     ; preds = %30, %15
  %32 = call i32 @gettimeofday(%struct.timeval* nonnull %4, %struct.timezone* null) #4
  %33 = getelementptr inbounds %struct.timeval, %struct.timeval* %4, i64 0, i32 0
  %34 = load i64, i64* %33, align 8, !tbaa !7
  %35 = getelementptr inbounds %struct.timeval, %struct.timeval* %3, i64 0, i32 0
  %36 = load i64, i64* %35, align 8, !tbaa !7
  %37 = sub nsw i64 %34, %36
  %38 = mul nsw i64 %37, 1000000
  %39 = getelementptr inbounds %struct.timeval, %struct.timeval* %4, i64 0, i32 1
  %40 = load i64, i64* %39, align 8, !tbaa !9
  %41 = add nsw i64 %38, %40
  %42 = getelementptr inbounds %struct.timeval, %struct.timeval* %3, i64 0, i32 1
  %43 = load i64, i64* %42, align 8, !tbaa !9
  %44 = sub i64 %41, %43
  %45 = sitofp i64 %44 to double
  %46 = fdiv double %45, 1.000000e+06
  %47 = call i32 (i8*, ...) @printf(i8* getelementptr inbounds ([11 x i8], [11 x i8]* @.str.4, i64 0, i64 0), double %46)
  br label %48

; <label>:48:                                     ; preds = %31, %12
  %49 = phi i32 [ 1, %12 ], [ 0, %31 ]
  call void @llvm.lifetime.end(i64 8, i8* %8) #4
  call void @llvm.lifetime.end(i64 16, i8* %7) #4
  call void @llvm.lifetime.end(i64 16, i8* %6) #4
  ret i32 %49
}

; Function Attrs: argmemonly nounwind
declare void @llvm.lifetime.start(i64, i8* nocapture) #1

; Function Attrs: nounwind
declare i32 @gettimeofday(%struct.timeval* nocapture, %struct.timezone* nocapture) local_unnamed_addr #2

; Function Attrs: nounwind
declare noalias %struct._IO_FILE* @fopen(i8* nocapture readonly, i8* nocapture readonly) local_unnamed_addr #2

declare i32 @__isoc99_fscanf(%struct._IO_FILE*, i8*, ...) local_unnamed_addr #3

; Function Attrs: nounwind
declare i32 @fclose(%struct._IO_FILE* nocapture) local_unnamed_addr #2

declare i32 @main1(i32, i8**, i32) local_unnamed_addr #3

; Function Attrs: nounwind
declare i32 @printf(i8* nocapture readonly, ...) local_unnamed_addr #2

; Function Attrs: argmemonly nounwind
declare void @llvm.lifetime.end(i64, i8* nocapture) #1

; Function Attrs: nounwind
declare i64 @fwrite(i8* nocapture, i64, i64, %struct._IO_FILE* nocapture) #4

attributes #0 = { nounwind uwtable "disable-tail-calls"="false" "less-precise-fpmad"="false" "no-frame-pointer-elim"="false" "no-infs-fp-math"="false" "no-jump-tables"="false" "no-nans-fp-math"="false" "no-signed-zeros-fp-math"="false" "stack-protector-buffer-size"="8" "target-cpu"="x86-64" "target-features"="+fxsr,+mmx,+rtm,+sse,+sse2,+x87" "unsafe-fp-math"="false" "use-soft-float"="false" }
attributes #1 = { argmemonly nounwind }
attributes #2 = { nounwind "disable-tail-calls"="false" "less-precise-fpmad"="false" "no-frame-pointer-elim"="false" "no-infs-fp-math"="false" "no-nans-fp-math"="false" "no-signed-zeros-fp-math"="false" "stack-protector-buffer-size"="8" "target-cpu"="x86-64" "target-features"="+fxsr,+mmx,+rtm,+sse,+sse2,+x87" "unsafe-fp-math"="false" "use-soft-float"="false" }
attributes #3 = { "disable-tail-calls"="false" "less-precise-fpmad"="false" "no-frame-pointer-elim"="false" "no-infs-fp-math"="false" "no-nans-fp-math"="false" "no-signed-zeros-fp-math"="false" "stack-protector-buffer-size"="8" "target-cpu"="x86-64" "target-features"="+fxsr,+mmx,+rtm,+sse,+sse2,+x87" "unsafe-fp-math"="false" "use-soft-float"="false" }
attributes #4 = { nounwind }
attributes #5 = { cold }

!llvm.ident = !{!0}

!0 = !{!"clang version 4.0.0 (http://gitlab.lsc.ic.unicamp.br/luis.mattos/DoAcrossClause-clang.git 9f12bb5a981b40920201c4dbfd7f8b1ba31bdd14) (http://gitlab.lsc.ic.unicamp.br/luis.mattos/DoAcrossClause.git 041019426da53cb3e8e9d731a855c63e9b22b05f)"}
!1 = !{!2, !2, i64 0}
!2 = !{!"any pointer", !3, i64 0}
!3 = !{!"omnipotent char", !4, i64 0}
!4 = !{!"Simple C/C++ TBAA"}
!5 = !{!6, !6, i64 0}
!6 = !{!"long", !3, i64 0}
!7 = !{!8, !6, i64 0}
!8 = !{!"timeval", !6, i64 0, !6, i64 8}
!9 = !{!8, !6, i64 8}
