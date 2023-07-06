#pragma once
 
#if (defined _MSC_VER && defined _DLL)
#if (defined PANGOLIN_VIEWER_DLL_EXPORT)
#define PANGOLIN_VIEWER_API __declspec(dllexport)
#else
#define PANGOLIN_VIEWER_API __declspec(dllimport)
#endif
#else
#define PANGOLIN_VIEWER_API
#endif
