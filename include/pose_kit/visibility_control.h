#ifndef POSE_KIT__VISIBILITY_CONTROL_H_
#define POSE_KIT__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define POSE_KIT_EXPORT __attribute__ ((dllexport))
    #define POSE_KIT_IMPORT __attribute__ ((dllimport))
  #else
    #define POSE_KIT_EXPORT __declspec(dllexport)
    #define POSE_KIT_IMPORT __declspec(dllimport)
  #endif
  #ifdef POSE_KIT_BUILDING_LIBRARY
    #define POSE_KIT_PUBLIC POSE_KIT_EXPORT
  #else
    #define POSE_KIT_PUBLIC POSE_KIT_IMPORT
  #endif
  #define POSE_KIT_PUBLIC_TYPE POSE_KIT_PUBLIC
  #define POSE_KIT_LOCAL
#else
  #define POSE_KIT_EXPORT __attribute__ ((visibility("default")))
  #define POSE_KIT_IMPORT
  #if __GNUC__ >= 4
    #define POSE_KIT_PUBLIC __attribute__ ((visibility("default")))
    #define POSE_KIT_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define POSE_KIT_PUBLIC
    #define POSE_KIT_LOCAL
  #endif
  #define POSE_KIT_PUBLIC_TYPE
#endif

#endif // POSE_KIT__VISIBILITY_CONTROL_H_
