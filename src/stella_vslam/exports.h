#pragma once
 
#if (defined _MSC_VER && defined _DLL)
#if (defined STELLA_VSLAM_DLL_EXPORT)
    #define STELLA_VSLAM_API __declspec(dllexport)
#else
    #define STELLA_VSLAM_API __declspec(dllimport)
#endif
#else
    #define STELLA_VSLAM_API
#endif
