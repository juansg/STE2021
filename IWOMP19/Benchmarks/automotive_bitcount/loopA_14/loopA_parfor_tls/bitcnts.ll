; ModuleID = 'bitcnts.c'
source_filename = "bitcnts.c"
target datalayout = "e-m:e-i64:64-f80:128-n8:16:32:64-S128"
target triple = "x86_64-unknown-linux-gnu"

%struct._IO_FILE = type { i32, i8*, i8*, i8*, i8*, i8*, i8*, i8*, i8*, i8*, i8*, i8*, %struct._IO_marker*, %struct._IO_FILE*, i32, i32, i64, i16, i8, [1 x i8], i8*, i64, i8*, i8*, i8*, i8*, i64, i32, [20 x i8] }
%struct._IO_marker = type { %struct._IO_marker*, %struct._IO_FILE*, i32 }
%ident_t = type { i32, i32, i32, i32, i8* }
%struct.timeval = type { i64, i64 }
%struct.timezone = type { i32, i32 }

@main1.pBitCntFunc = internal global [7 x i32 (i64)*] [i32 (i64)* @bit_count, i32 (i64)* @bitcount, i32 (i64)* @ntbl_bitcnt, i32 (i64)* @ntbl_bitcount, i32 (i64)* @BW_btbl_bitcount, i32 (i64)* @AR_btbl_bitcount, i32 (i64)* @bit_shifter], align 16
@main1.text = internal unnamed_addr constant [7 x i8*] [i8* getelementptr inbounds ([29 x i8], [29 x i8]* @.str, i32 0, i32 0), i8* getelementptr inbounds ([26 x i8], [26 x i8]* @.str.1, i32 0, i32 0), i8* getelementptr inbounds ([31 x i8], [31 x i8]* @.str.2, i32 0, i32 0), i8* getelementptr inbounds ([35 x i8], [35 x i8]* @.str.3, i32 0, i32 0), i8* getelementptr inbounds ([38 x i8], [38 x i8]* @.str.4, i32 0, i32 0), i8* getelementptr inbounds ([38 x i8], [38 x i8]* @.str.5, i32 0, i32 0), i8* getelementptr inbounds ([21 x i8], [21 x i8]* @.str.6, i32 0, i32 0)], align 16
@.str = private unnamed_addr constant [29 x i8] c"Optimized 1 bit/loop counter\00", align 1
@.str.1 = private unnamed_addr constant [26 x i8] c"Ratko's mystery algorithm\00", align 1
@.str.2 = private unnamed_addr constant [31 x i8] c"Recursive bit count by nybbles\00", align 1
@.str.3 = private unnamed_addr constant [35 x i8] c"Non-recursive bit count by nybbles\00", align 1
@.str.4 = private unnamed_addr constant [38 x i8] c"Non-recursive bit count by bytes (BW)\00", align 1
@.str.5 = private unnamed_addr constant [38 x i8] c"Non-recursive bit count by bytes (AR)\00", align 1
@.str.6 = private unnamed_addr constant [21 x i8] c"Shift and count bits\00", align 1
@stderr = external local_unnamed_addr global %struct._IO_FILE*, align 8
@.str.7 = private unnamed_addr constant [29 x i8] c"Usage: bitcnts <iterations>\0A\00", align 1
@.str.8 = private unnamed_addr constant [33 x i8] c"Bit counter algorithm benchmark\0A\00", align 1
@n = common local_unnamed_addr global i64 0, align 8
@main1.next = internal global i64 0, align 8
@.str.9 = private unnamed_addr constant [23 x i8] c";unknown;unknown;0;0;;\00", align 1
@0 = private unnamed_addr constant %ident_t { i32 0, i32 2, i32 0, i32 0, i8* getelementptr inbounds ([23 x i8], [23 x i8]* @.str.9, i32 0, i32 0) }, align 8
@tiempo_promedio = local_unnamed_addr global double 0.000000e+00, align 8
@.str.10 = private unnamed_addr constant [21 x i8] c"Tiempo promedio %lf\0A\00", align 1
@.str.11 = private unnamed_addr constant [18 x i8] c"%-38s> Bits: %ld\0A\00", align 1

; Function Attrs: nounwind uwtable
define void @_xabort2(i8 signext) local_unnamed_addr #0 {
  tail call void @llvm.x86.xabort(i8 -1)
  ret void
}

; Function Attrs: nounwind
declare void @llvm.x86.xabort(i8) #1

; Function Attrs: nounwind uwtable
define i32 @main1(i32, i8** nocapture readonly, i32) local_unnamed_addr #0 {
  %4 = alloca %struct.timeval, align 8
  %5 = alloca %struct.timeval, align 8
  %6 = alloca i64, align 8
  %7 = alloca i32, align 4
  %8 = tail call i32 @__kmpc_global_thread_num(%ident_t* nonnull @0) #1
  %9 = bitcast %struct.timeval* %4 to i8*
  call void @llvm.lifetime.start(i64 16, i8* %9) #1
  %10 = bitcast %struct.timeval* %5 to i8*
  call void @llvm.lifetime.start(i64 16, i8* %10) #1
  %11 = bitcast i64* %6 to i8*
  call void @llvm.lifetime.start(i64 8, i8* %11) #1
  %12 = bitcast i32* %7 to i8*
  call void @llvm.lifetime.start(i64 4, i8* %12) #1
  %13 = icmp slt i32 %0, 2
  br i1 %13, label %14, label %17

; <label>:14:                                     ; preds = %3
  %15 = load %struct._IO_FILE*, %struct._IO_FILE** @stderr, align 8, !tbaa !1
  %16 = tail call i64 @fwrite(i8* getelementptr inbounds ([29 x i8], [29 x i8]* @.str.7, i64 0, i64 0), i64 28, i64 1, %struct._IO_FILE* %15) #7
  tail call void @exit(i32 1) #8
  unreachable

; <label>:17:                                     ; preds = %3
  %18 = getelementptr inbounds i8*, i8** %1, i64 1
  %19 = load i8*, i8** %18, align 8, !tbaa !1
  %20 = tail call i64 @strtol(i8* nocapture nonnull %19, i8** null, i32 10) #1
  %21 = trunc i64 %20 to i32
  store i32 %21, i32* %7, align 4, !tbaa !5
  %22 = icmp eq i32 %2, 0
  br i1 %22, label %23, label %28

; <label>:23:                                     ; preds = %17
  %24 = getelementptr inbounds %struct.timeval, %struct.timeval* %5, i64 0, i32 0
  %25 = getelementptr inbounds %struct.timeval, %struct.timeval* %4, i64 0, i32 0
  %26 = getelementptr inbounds %struct.timeval, %struct.timeval* %5, i64 0, i32 1
  %27 = getelementptr inbounds %struct.timeval, %struct.timeval* %4, i64 0, i32 1
  br label %59

; <label>:28:                                     ; preds = %17
  %29 = tail call i32 @puts(i8* getelementptr inbounds ([33 x i8], [33 x i8]* @.str.8, i64 0, i64 0))
  %30 = getelementptr inbounds %struct.timeval, %struct.timeval* %5, i64 0, i32 0
  %31 = getelementptr inbounds %struct.timeval, %struct.timeval* %4, i64 0, i32 0
  %32 = getelementptr inbounds %struct.timeval, %struct.timeval* %5, i64 0, i32 1
  %33 = getelementptr inbounds %struct.timeval, %struct.timeval* %4, i64 0, i32 1
  br label %34

; <label>:34:                                     ; preds = %34, %28
  %35 = phi i64 [ 0, %28 ], [ %57, %34 ]
  %36 = call i32 @gettimeofday(%struct.timeval* nonnull %4, %struct.timezone* null) #1
  store i64 0, i64* @n, align 8, !tbaa !7
  store volatile i64 0, i64* @main1.next, align 8, !tbaa !7
  call void @__kmpc_push_num_threads(%ident_t* nonnull @0, i32 %8, i32 4) #1
  call void (%ident_t*, i32, void (i32*, i32*, ...)*, ...) @__kmpc_fork_call(%ident_t* nonnull @0, i32 4, void (i32*, i32*, ...)* bitcast (void (i32*, i32*, i64*, i32*, [7 x i32 (i64)*]*, i64)* @.omp_outlined. to void (i32*, i32*, ...)*), i64* nonnull %6, i32* nonnull %7, [7 x i32 (i64)*]* nonnull @main1.pBitCntFunc, i64 %35) #1
  %37 = call i32 @gettimeofday(%struct.timeval* nonnull %5, %struct.timezone* null) #1
  %38 = load i64, i64* %30, align 8, !tbaa !9
  %39 = load i64, i64* %31, align 8, !tbaa !9
  %40 = sub nsw i64 %38, %39
  %41 = mul nsw i64 %40, 1000000
  %42 = load i64, i64* %32, align 8, !tbaa !11
  %43 = add nsw i64 %41, %42
  %44 = load i64, i64* %33, align 8, !tbaa !11
  %45 = sub i64 %43, %44
  %46 = sitofp i64 %45 to double
  %47 = fdiv double %46, 1.000000e+06
  %48 = load double, double* @tiempo_promedio, align 8, !tbaa !12
  %49 = fadd double %48, %47
  store double %49, double* @tiempo_promedio, align 8, !tbaa !12
  %50 = call i32 (i8*, ...) @printf(i8* getelementptr inbounds ([21 x i8], [21 x i8]* @.str.10, i64 0, i64 0), double %49)
  %51 = shl i64 %35, 32
  %52 = ashr exact i64 %51, 32
  %53 = getelementptr inbounds [7 x i8*], [7 x i8*]* @main1.text, i64 0, i64 %52
  %54 = load i8*, i8** %53, align 8, !tbaa !1
  %55 = load i64, i64* @n, align 8, !tbaa !7
  %56 = call i32 (i8*, ...) @printf(i8* getelementptr inbounds ([18 x i8], [18 x i8]* @.str.11, i64 0, i64 0), i8* %54, i64 %55)
  %57 = add nuw nsw i64 %35, 1
  %58 = icmp eq i64 %57, 7
  br i1 %58, label %79, label %34

; <label>:59:                                     ; preds = %59, %23
  %60 = phi i64 [ 0, %23 ], [ %76, %59 ]
  %61 = call i32 @gettimeofday(%struct.timeval* nonnull %4, %struct.timezone* null) #1
  store i64 0, i64* @n, align 8, !tbaa !7
  store volatile i64 0, i64* @main1.next, align 8, !tbaa !7
  call void @__kmpc_push_num_threads(%ident_t* nonnull @0, i32 %8, i32 4) #1
  call void (%ident_t*, i32, void (i32*, i32*, ...)*, ...) @__kmpc_fork_call(%ident_t* nonnull @0, i32 4, void (i32*, i32*, ...)* bitcast (void (i32*, i32*, i64*, i32*, [7 x i32 (i64)*]*, i64)* @.omp_outlined. to void (i32*, i32*, ...)*), i64* nonnull %6, i32* nonnull %7, [7 x i32 (i64)*]* nonnull @main1.pBitCntFunc, i64 %60) #1
  %62 = call i32 @gettimeofday(%struct.timeval* nonnull %5, %struct.timezone* null) #1
  %63 = load i64, i64* %24, align 8, !tbaa !9
  %64 = load i64, i64* %25, align 8, !tbaa !9
  %65 = sub nsw i64 %63, %64
  %66 = mul nsw i64 %65, 1000000
  %67 = load i64, i64* %26, align 8, !tbaa !11
  %68 = add nsw i64 %66, %67
  %69 = load i64, i64* %27, align 8, !tbaa !11
  %70 = sub i64 %68, %69
  %71 = sitofp i64 %70 to double
  %72 = fdiv double %71, 1.000000e+06
  %73 = load double, double* @tiempo_promedio, align 8, !tbaa !12
  %74 = fadd double %73, %72
  store double %74, double* @tiempo_promedio, align 8, !tbaa !12
  %75 = call i32 (i8*, ...) @printf(i8* getelementptr inbounds ([21 x i8], [21 x i8]* @.str.10, i64 0, i64 0), double %74)
  %76 = add nuw nsw i64 %60, 1
  %77 = icmp eq i64 %76, 7
  br i1 %77, label %78, label %59

; <label>:78:                                     ; preds = %59
  br label %80

; <label>:79:                                     ; preds = %34
  br label %80

; <label>:80:                                     ; preds = %79, %78
  call void @llvm.lifetime.end(i64 4, i8* %12) #1
  call void @llvm.lifetime.end(i64 8, i8* %11) #1
  call void @llvm.lifetime.end(i64 16, i8* nonnull %10) #1
  call void @llvm.lifetime.end(i64 16, i8* nonnull %9) #1
  ret i32 0
}

; Function Attrs: argmemonly nounwind
declare void @llvm.lifetime.start(i64, i8* nocapture) #2

declare i32 @bit_count(i64) #3

declare i32 @bitcount(i64) #3

declare i32 @ntbl_bitcnt(i64) #3

declare i32 @ntbl_bitcount(i64) #3

declare i32 @BW_btbl_bitcount(i64) #3

declare i32 @AR_btbl_bitcount(i64) #3

; Function Attrs: norecurse nounwind readnone uwtable
define internal i32 @bit_shifter(i64) #4 {
  %2 = icmp eq i64 %0, 0
  br i1 %2, label %17, label %3

; <label>:3:                                      ; preds = %1
  br label %4

; <label>:4:                                      ; preds = %3, %4
  %5 = phi i32 [ %10, %4 ], [ 0, %3 ]
  %6 = phi i32 [ %11, %4 ], [ 0, %3 ]
  %7 = phi i64 [ %12, %4 ], [ %0, %3 ]
  %8 = trunc i64 %7 to i32
  %9 = and i32 %8, 1
  %10 = add nsw i32 %5, %9
  %11 = add nuw nsw i32 %6, 1
  %12 = ashr i64 %7, 1
  %13 = icmp ne i64 %12, 0
  %14 = icmp ult i32 %11, 64
  %15 = and i1 %13, %14
  br i1 %15, label %4, label %16

; <label>:16:                                     ; preds = %4
  br label %17

; <label>:17:                                     ; preds = %16, %1
  %18 = phi i32 [ 0, %1 ], [ %10, %16 ]
  ret i32 %18
}

; Function Attrs: noreturn nounwind
declare void @exit(i32) local_unnamed_addr #5

; Function Attrs: nounwind
declare i32 @puts(i8* nocapture readonly) local_unnamed_addr #6

; Function Attrs: nounwind
declare i32 @gettimeofday(%struct.timeval* nocapture, %struct.timezone* nocapture) local_unnamed_addr #6

; Function Attrs: nounwind uwtable
define internal void @.omp_outlined.(i32* noalias nocapture readonly, i32* noalias nocapture readnone, i64* nocapture readnone dereferenceable(8), i32* nocapture readonly dereferenceable(4), [7 x i32 (i64)*]* nocapture readonly dereferenceable(56), i64) #0 {
  %7 = alloca i64, align 8
  %8 = alloca i64, align 8
  %9 = alloca i64, align 8
  %10 = alloca i32, align 4
  %11 = alloca [7 x i32 (i64)*], align 16
  %12 = load i32, i32* %3, align 4, !tbaa !5
  %13 = sext i32 %12 to i64
  %14 = add nsw i64 %13, 501
  %15 = sdiv i64 %14, 502
  %16 = add nsw i64 %15, -1
  %17 = icmp sgt i32 %12, 0
  br i1 %17, label %23, label %18

; <label>:18:                                     ; preds = %6
  %19 = bitcast i32* %10 to i8*
  %20 = bitcast i64* %9 to i8*
  %21 = bitcast i64* %8 to i8*
  %22 = bitcast i64* %7 to i8*
  br label %98

; <label>:23:                                     ; preds = %6
  %24 = bitcast i64* %7 to i8*
  call void @llvm.lifetime.start(i64 8, i8* %24) #1
  store i64 0, i64* %7, align 8, !tbaa !7
  %25 = bitcast i64* %8 to i8*
  call void @llvm.lifetime.start(i64 8, i8* %25) #1
  store i64 %16, i64* %8, align 8, !tbaa !7
  %26 = bitcast i64* %9 to i8*
  call void @llvm.lifetime.start(i64 8, i8* %26) #1
  store i64 1, i64* %9, align 8, !tbaa !7
  %27 = bitcast i32* %10 to i8*
  call void @llvm.lifetime.start(i64 4, i8* %27) #1
  store i32 0, i32* %10, align 4, !tbaa !5
  %28 = bitcast [7 x i32 (i64)*]* %11 to i8*
  call void @llvm.lifetime.start(i64 56, i8* %28) #1
  %29 = bitcast [7 x i32 (i64)*]* %4 to i8*
  call void @llvm.memcpy.p0i8.p0i8.i64(i8* %28, i8* %29, i64 56, i32 16, i1 false), !tbaa.struct !14
  %30 = load i32, i32* %0, align 4, !tbaa !5
  call void @__kmpc_for_static_init_8(%ident_t* nonnull @0, i32 %30, i32 33, i32* nonnull %10, i64* nonnull %7, i64* nonnull %8, i64* nonnull %9, i64 1, i64 1) #1
  %31 = load i64, i64* %8, align 8, !tbaa !7
  %32 = icmp sgt i64 %31, %16
  %33 = select i1 %32, i64 %16, i64 %31
  store i64 %33, i64* %8, align 8, !tbaa !7
  %34 = load i64, i64* %7, align 8, !tbaa !7
  %35 = icmp sgt i64 %34, %33
  br i1 %35, label %97, label %36

; <label>:36:                                     ; preds = %23
  %37 = shl i64 %5, 32
  %38 = ashr exact i64 %37, 32
  %39 = getelementptr inbounds [7 x i32 (i64)*], [7 x i32 (i64)*]* %11, i64 0, i64 %38
  br label %40

; <label>:40:                                     ; preds = %36, %87
  %41 = phi i64 [ %33, %36 ], [ %94, %87 ]
  %42 = phi i64 [ %34, %36 ], [ %91, %87 ]
  %43 = icmp sgt i64 %42, %41
  br i1 %43, label %87, label %44

; <label>:44:                                     ; preds = %40
  br label %45

; <label>:45:                                     ; preds = %44, %79
  %46 = phi i64 [ %82, %79 ], [ %42, %44 ]
  %47 = mul nsw i64 %46, 502
  br label %48

; <label>:48:                                     ; preds = %51, %45
  %49 = load volatile i64, i64* @main1.next, align 8, !tbaa !7
  %50 = icmp eq i64 %49, %47
  br i1 %50, label %54, label %51

; <label>:51:                                     ; preds = %48
  %52 = call i32 @llvm.x86.xbegin() #1
  %53 = icmp eq i32 %52, -1
  br i1 %53, label %54, label %48

; <label>:54:                                     ; preds = %51, %48
  %55 = phi i32 [ 1, %51 ], [ 0, %48 ]
  %56 = add nsw i64 %47, i32 501
  br label %57

; <label>:57:                                     ; preds = %54, %62
  %58 = phi i64 [ %47, %54 ], [ %70, %62 ]
  %59 = load i32, i32* %3, align 4, !tbaa !5
  %60 = sext i32 %59 to i64
  %61 = icmp slt i64 %58, %60
  br i1 %61, label %62, label %72

; <label>:62:                                     ; preds = %57
  %63 = mul nsw i64 %58, 13
  %64 = add nsw i64 %63, 1
  %65 = load i32 (i64)*, i32 (i64)** %39, align 8, !tbaa !1
  %66 = call i32 %65(i64 %64) #1
  %67 = sext i32 %66 to i64
  %68 = load i64, i64* @n, align 8, !tbaa !7
  %69 = add nsw i64 %68, %67
  store i64 %69, i64* @n, align 8, !tbaa !7
  %70 = add nsw i64 %58, 1
  %71 = icmp slt i64 %58, %56
  br i1 %71, label %57, label %72

; <label>:72:                                     ; preds = %62, %57
  %73 = icmp eq i32 %55, 0
  br i1 %73, label %79, label %74

; <label>:74:                                     ; preds = %72
  %75 = load volatile i64, i64* @main1.next, align 8, !tbaa !7
  %76 = icmp eq i64 %75, %47
  br i1 %76, label %78, label %77

; <label>:77:                                     ; preds = %74
  call void @llvm.x86.xabort(i8 -1) #1
  br label %78

; <label>:78:                                     ; preds = %74, %77
  call void @llvm.x86.xend() #1
  br label %79

; <label>:79:                                     ; preds = %72, %78
  %80 = load volatile i64, i64* @main1.next, align 8, !tbaa !7
  %81 = add nsw i64 %80, i32 502
  store volatile i64 %81, i64* @main1.next, align 8, !tbaa !7
  %82 = add nsw i64 %46, 1
  %83 = load i64, i64* %8, align 8, !tbaa !7
  %84 = icmp slt i64 %46, %83
  br i1 %84, label %45, label %85

; <label>:85:                                     ; preds = %79
  %86 = load i64, i64* %7, align 8, !tbaa !7
  br label %87

; <label>:87:                                     ; preds = %85, %40
  %88 = phi i64 [ %42, %40 ], [ %86, %85 ]
  %89 = phi i64 [ %41, %40 ], [ %83, %85 ]
  %90 = load i64, i64* %9, align 8, !tbaa !7
  %91 = add nsw i64 %90, %88
  store i64 %91, i64* %7, align 8, !tbaa !7
  %92 = add nsw i64 %90, %89
  %93 = icmp sgt i64 %92, %16
  %94 = select i1 %93, i64 %16, i64 %92
  store i64 %94, i64* %8, align 8, !tbaa !7
  %95 = icmp sgt i64 %91, %94
  br i1 %95, label %96, label %40

; <label>:96:                                     ; preds = %87
  br label %97

; <label>:97:                                     ; preds = %96, %23
  call void @__kmpc_for_static_fini(%ident_t* nonnull @0, i32 %30) #1
  call void @llvm.lifetime.end(i64 56, i8* %28) #1
  br label %98

; <label>:98:                                     ; preds = %18, %97
  %99 = phi i8* [ %22, %18 ], [ %24, %97 ]
  %100 = phi i8* [ %21, %18 ], [ %25, %97 ]
  %101 = phi i8* [ %20, %18 ], [ %26, %97 ]
  %102 = phi i8* [ %19, %18 ], [ %27, %97 ]
  call void @llvm.lifetime.end(i64 4, i8* %102) #1
  call void @llvm.lifetime.end(i64 8, i8* %101) #1
  call void @llvm.lifetime.end(i64 8, i8* %100) #1
  call void @llvm.lifetime.end(i64 8, i8* %99) #1
  ret void
}

; Function Attrs: argmemonly nounwind
declare void @llvm.lifetime.end(i64, i8* nocapture) #2

; Function Attrs: argmemonly nounwind
declare void @llvm.memcpy.p0i8.p0i8.i64(i8* nocapture writeonly, i8* nocapture readonly, i64, i32, i1) #2

declare void @__kmpc_for_static_init_8(%ident_t*, i32, i32, i32*, i64*, i64*, i64*, i64, i64) local_unnamed_addr

declare void @__kmpc_for_static_fini(%ident_t*, i32) local_unnamed_addr

declare i32 @__kmpc_global_thread_num(%ident_t*) local_unnamed_addr

declare void @__kmpc_push_num_threads(%ident_t*, i32, i32) local_unnamed_addr

declare void @__kmpc_fork_call(%ident_t*, i32, void (i32*, i32*, ...)*, ...) local_unnamed_addr

; Function Attrs: nounwind
declare i32 @printf(i8* nocapture readonly, ...) local_unnamed_addr #6

; Function Attrs: nounwind
declare i64 @strtol(i8* readonly, i8** nocapture, i32) local_unnamed_addr #6

; Function Attrs: nounwind
declare i32 @llvm.x86.xbegin() #1

; Function Attrs: nounwind
declare void @llvm.x86.xend() #1

; Function Attrs: nounwind
declare i64 @fwrite(i8* nocapture, i64, i64, %struct._IO_FILE* nocapture) #1

attributes #0 = { nounwind uwtable "disable-tail-calls"="false" "less-precise-fpmad"="false" "no-frame-pointer-elim"="false" "no-infs-fp-math"="false" "no-jump-tables"="false" "no-nans-fp-math"="false" "no-signed-zeros-fp-math"="false" "stack-protector-buffer-size"="8" "target-cpu"="x86-64" "target-features"="+fxsr,+mmx,+rtm,+sse,+sse2,+x87" "unsafe-fp-math"="false" "use-soft-float"="false" }
attributes #1 = { nounwind }
attributes #2 = { argmemonly nounwind }
attributes #3 = { "disable-tail-calls"="false" "less-precise-fpmad"="false" "no-frame-pointer-elim"="false" "no-infs-fp-math"="false" "no-nans-fp-math"="false" "no-signed-zeros-fp-math"="false" "stack-protector-buffer-size"="8" "target-cpu"="x86-64" "target-features"="+fxsr,+mmx,+rtm,+sse,+sse2,+x87" "unsafe-fp-math"="false" "use-soft-float"="false" }
attributes #4 = { norecurse nounwind readnone uwtable "disable-tail-calls"="false" "less-precise-fpmad"="false" "no-frame-pointer-elim"="false" "no-infs-fp-math"="false" "no-jump-tables"="false" "no-nans-fp-math"="false" "no-signed-zeros-fp-math"="false" "stack-protector-buffer-size"="8" "target-cpu"="x86-64" "target-features"="+fxsr,+mmx,+rtm,+sse,+sse2,+x87" "unsafe-fp-math"="false" "use-soft-float"="false" }
attributes #5 = { noreturn nounwind "disable-tail-calls"="false" "less-precise-fpmad"="false" "no-frame-pointer-elim"="false" "no-infs-fp-math"="false" "no-nans-fp-math"="false" "no-signed-zeros-fp-math"="false" "stack-protector-buffer-size"="8" "target-cpu"="x86-64" "target-features"="+fxsr,+mmx,+rtm,+sse,+sse2,+x87" "unsafe-fp-math"="false" "use-soft-float"="false" }
attributes #6 = { nounwind "disable-tail-calls"="false" "less-precise-fpmad"="false" "no-frame-pointer-elim"="false" "no-infs-fp-math"="false" "no-nans-fp-math"="false" "no-signed-zeros-fp-math"="false" "stack-protector-buffer-size"="8" "target-cpu"="x86-64" "target-features"="+fxsr,+mmx,+rtm,+sse,+sse2,+x87" "unsafe-fp-math"="false" "use-soft-float"="false" }
attributes #7 = { cold }
attributes #8 = { noreturn nounwind }

!llvm.ident = !{!0}

!0 = !{!"clang version 4.0.0 (http://gitlab.lsc.ic.unicamp.br/luis.mattos/DoAcrossClause-clang.git 9f12bb5a981b40920201c4dbfd7f8b1ba31bdd14) (http://gitlab.lsc.ic.unicamp.br/luis.mattos/DoAcrossClause.git 041019426da53cb3e8e9d731a855c63e9b22b05f)"}
!1 = !{!2, !2, i64 0}
!2 = !{!"any pointer", !3, i64 0}
!3 = !{!"omnipotent char", !4, i64 0}
!4 = !{!"Simple C/C++ TBAA"}
!5 = !{!6, !6, i64 0}
!6 = !{!"int", !3, i64 0}
!7 = !{!8, !8, i64 0}
!8 = !{!"long", !3, i64 0}
!9 = !{!10, !8, i64 0}
!10 = !{!"timeval", !8, i64 0, !8, i64 8}
!11 = !{!10, !8, i64 8}
!12 = !{!13, !13, i64 0}
!13 = !{!"double", !3, i64 0}
!14 = !{i64 0, i64 56, !15}
!15 = !{!3, !3, i64 0}
