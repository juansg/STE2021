; NOTE: Assertions have been autogenerated by utils/update_test_checks.py
; RUN: opt -instsimplify -S < %s | FileCheck %s

declare float @llvm.minnum.f32(float, float)
declare <4 x float> @llvm.minnum.v4f32(<4 x float>, <4 x float>)

declare float @llvm.maxnum.f32(float, float)
declare <4 x float> @llvm.maxnum.v4f32(<4 x float>, <4 x float>)

declare float @llvm.minimum.f32(float, float)
declare <4 x float> @llvm.minimum.v4f32(<4 x float>, <4 x float>)

declare float @llvm.maximum.f32(float, float)
declare <4 x float> @llvm.maximum.v4f32(<4 x float>, <4 x float>)

declare i8 @llvm.smax.i8(i8, i8)
declare <5 x i8> @llvm.smax.v5i8(<5 x i8>, <5 x i8>)

declare i8 @llvm.smin.i8(i8, i8)
declare <5 x i8> @llvm.smin.v5i8(<5 x i8>, <5 x i8>)

declare i8 @llvm.umax.i8(i8, i8)
declare <5 x i8> @llvm.umax.v5i8(<5 x i8>, <5 x i8>)

declare i8 @llvm.umin.i8(i8, i8)
declare <5 x i8> @llvm.umin.v5i8(<5 x i8>, <5 x i8>)

define float @minnum_float() {
; CHECK-LABEL: @minnum_float(
; CHECK-NEXT:    ret float 5.000000e+00
;
  %1 = call float @llvm.minnum.f32(float 5.0, float 42.0)
  ret float %1
}

; Check that minnum constant folds to propagate non-NaN or smaller argument

define <4 x float> @minnum_float_vec() {
; CHECK-LABEL: @minnum_float_vec(
; CHECK-NEXT:    ret <4 x float> <float 0x7FF8000000000000, float 5.000000e+00, float 4.200000e+01, float 5.000000e+00>
;
  %1 = call <4 x float> @llvm.minnum.v4f32(<4 x float> <float 0x7FF8000000000000, float 0x7FF8000000000000, float 42., float 42.>, <4 x float> <float 0x7FF8000000000000, float 5., float 0x7FF8000000000000, float 5.>)
  ret <4 x float> %1
}

; Check that minnum constant folds to propagate one of its argument zeros

define <4 x float> @minnum_float_zeros_vec() {
; CHECK-LABEL: @minnum_float_zeros_vec(
; CHECK-NEXT:    ret <4 x float> <float 0.000000e+00, float -0.000000e+00, float 0.000000e+00, float -0.000000e+00>
;
  %1 = call <4 x float> @llvm.minnum.v4f32(<4 x float> <float 0.0, float -0.0, float 0.0, float -0.0>, <4 x float> <float 0.0, float 0.0, float -0.0, float -0.0>)
  ret <4 x float> %1
}

define float @maxnum_float() {
; CHECK-LABEL: @maxnum_float(
; CHECK-NEXT:    ret float 4.200000e+01
;
  %1 = call float @llvm.maxnum.f32(float 5.0, float 42.0)
  ret float %1
}

; Check that maxnum constant folds to propagate non-NaN or greater argument

define <4 x float> @maxnum_float_vec() {
; CHECK-LABEL: @maxnum_float_vec(
; CHECK-NEXT:    ret <4 x float> <float 0x7FF8000000000000, float 5.000000e+00, float 4.200000e+01, float 4.200000e+01>
;
  %1 = call <4 x float> @llvm.maxnum.v4f32(<4 x float> <float 0x7FF8000000000000, float 0x7FF8000000000000, float 42., float 42.>, <4 x float> <float 0x7FF8000000000000, float 5., float 0x7FF8000000000000, float 5.>)
  ret <4 x float> %1
}

; Check that maxnum constant folds to propagate one of its argument zeros

define <4 x float> @maxnum_float_zeros_vec() {
; CHECK-LABEL: @maxnum_float_zeros_vec(
; CHECK-NEXT:    ret <4 x float> <float 0.000000e+00, float -0.000000e+00, float 0.000000e+00, float -0.000000e+00>
;
  %1 = call <4 x float> @llvm.maxnum.v4f32(<4 x float> <float 0.0, float -0.0, float 0.0, float -0.0>, <4 x float> <float 0.0, float 0.0, float -0.0, float -0.0>)
  ret <4 x float> %1
}

define float @minimum_float() {
; CHECK-LABEL: @minimum_float(
; CHECK-NEXT:    ret float 5.000000e+00
;
  %1 = call float @llvm.minimum.f32(float 5.0, float 42.0)
  ret float %1
}

; Check that minimum propagates its NaN or smaller argument

define <4 x float> @minimum_float_vec() {
; CHECK-LABEL: @minimum_float_vec(
; CHECK-NEXT:    ret <4 x float> <float 0x7FF8000000000000, float 0x7FF8000000000000, float 0x7FF8000000000000, float 5.000000e+00>
;
  %1 = call <4 x float> @llvm.minimum.v4f32(<4 x float> <float 0x7FF8000000000000, float 0x7FF8000000000000, float 42., float 42.>, <4 x float> <float 0x7FF8000000000000, float 5., float 0x7FF8000000000000, float 5.>)
  ret <4 x float> %1
}

; Check that minimum treats -0.0 as smaller than 0.0 while constant folding

define <4 x float> @minimum_float_zeros_vec() {
; CHECK-LABEL: @minimum_float_zeros_vec(
; CHECK-NEXT:    ret <4 x float> <float 0.000000e+00, float -0.000000e+00, float -0.000000e+00, float -0.000000e+00>
;
  %1 = call <4 x float> @llvm.minimum.v4f32(<4 x float> <float 0.0, float -0.0, float 0.0, float -0.0>, <4 x float> <float 0.0, float 0.0, float -0.0, float -0.0>)
  ret <4 x float> %1
}

define float @maximum_float() {
; CHECK-LABEL: @maximum_float(
; CHECK-NEXT:    ret float 4.200000e+01
;
  %1 = call float @llvm.maximum.f32(float 5.0, float 42.0)
  ret float %1
}

; Check that maximum propagates its NaN or greater argument

define <4 x float> @maximum_float_vec() {
; CHECK-LABEL: @maximum_float_vec(
; CHECK-NEXT:    ret <4 x float> <float 0x7FF8000000000000, float 0x7FF8000000000000, float 0x7FF8000000000000, float 4.200000e+01>
;
  %1 = call <4 x float> @llvm.maximum.v4f32(<4 x float> <float 0x7FF8000000000000, float 0x7FF8000000000000, float 42., float 42.>, <4 x float> <float 0x7FF8000000000000, float 5., float 0x7FF8000000000000, float 5.>)
  ret <4 x float> %1
}

; Check that maximum treats -0.0 as smaller than 0.0 while constant folding

define <4 x float> @maximum_float_zeros_vec() {
; CHECK-LABEL: @maximum_float_zeros_vec(
; CHECK-NEXT:    ret <4 x float> <float 0.000000e+00, float 0.000000e+00, float 0.000000e+00, float -0.000000e+00>
;
  %1 = call <4 x float> @llvm.maximum.v4f32(<4 x float> <float 0.0, float -0.0, float 0.0, float -0.0>, <4 x float> <float 0.0, float 0.0, float -0.0, float -0.0>)
  ret <4 x float> %1
}

define i8 @smax() {
; CHECK-LABEL: @smax(
; CHECK-NEXT:    ret i8 -127
;
  %r = call i8 @llvm.smax.i8(i8 128, i8 129)
  ret i8 %r
}

define <5 x i8> @smax_vec() {
; CHECK-LABEL: @smax_vec(
; CHECK-NEXT:    ret <5 x i8> <i8 undef, i8 127, i8 127, i8 42, i8 127>
;
  %r = call <5 x i8> @llvm.smax.v5i8(<5 x i8> <i8 undef, i8 undef, i8 1, i8 42, i8 42>, <5 x i8> <i8 undef, i8 1, i8 undef, i8 42, i8 127>)
  ret <5 x i8> %r
}

define i8 @smin() {
; CHECK-LABEL: @smin(
; CHECK-NEXT:    ret i8 -128
;
  %r = call i8 @llvm.smin.i8(i8 128, i8 127)
  ret i8 %r
}

define <5 x i8> @smin_vec() {
; CHECK-LABEL: @smin_vec(
; CHECK-NEXT:    ret <5 x i8> <i8 undef, i8 -128, i8 -128, i8 42, i8 -127>
;
  %r = call <5 x i8> @llvm.smin.v5i8(<5 x i8> <i8 undef, i8 undef, i8 1, i8 42, i8 42>, <5 x i8> <i8 undef, i8 1, i8 undef, i8 42, i8 129>)
  ret <5 x i8> %r
}

define i8 @umax() {
; CHECK-LABEL: @umax(
; CHECK-NEXT:    ret i8 -128
;
  %r = call i8 @llvm.umax.i8(i8 128, i8 127)
  ret i8 %r
}

define <5 x i8> @umax_vec() {
; CHECK-LABEL: @umax_vec(
; CHECK-NEXT:    ret <5 x i8> <i8 undef, i8 -1, i8 -1, i8 42, i8 -128>
;
  %r = call <5 x i8> @llvm.umax.v5i8(<5 x i8> <i8 undef, i8 undef, i8 1, i8 42, i8 42>, <5 x i8> <i8 undef, i8 1, i8 undef, i8 42, i8 128>)
  ret <5 x i8> %r
}

define i8 @umin() {
; CHECK-LABEL: @umin(
; CHECK-NEXT:    ret i8 127
;
  %r = call i8 @llvm.umin.i8(i8 128, i8 127)
  ret i8 %r
}

define <5 x i8> @umin_vec() {
; CHECK-LABEL: @umin_vec(
; CHECK-NEXT:    ret <5 x i8> <i8 undef, i8 0, i8 0, i8 42, i8 42>
;
  %r = call <5 x i8> @llvm.umin.v5i8(<5 x i8> <i8 undef, i8 undef, i8 1, i8 42, i8 42>, <5 x i8> <i8 undef, i8 1, i8 undef, i8 42, i8 128>)
  ret <5 x i8> %r
}