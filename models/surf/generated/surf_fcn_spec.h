//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  surf_fcn_spec.h
//
//  Code generation for function 'surf_fcn'
//


#ifndef SURF_FCN_SPEC_H
#define SURF_FCN_SPEC_H

// Include files
#ifdef SURF_FCN_XIL_BUILD
#if defined(_MSC_VER) || defined(__LCC__)
#define SURF_FCN_DLL_EXPORT            __declspec(dllimport)
#else
#define SURF_FCN_DLL_EXPORT            __attribute__ ((visibility("default")))
#endif

#elif defined(BUILDING_SURF_FCN)
#if defined(_MSC_VER) || defined(__LCC__)
#define SURF_FCN_DLL_EXPORT            __declspec(dllexport)
#else
#define SURF_FCN_DLL_EXPORT            __attribute__ ((visibility("default")))
#endif

#else
#define SURF_FCN_DLL_EXPORT
#endif
#endif

// End of code generation (surf_fcn_spec.h)
