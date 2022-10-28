#ifndef DZ_MINIMAL_CONTROLLER__VISIBILITY_CONTROL_H_
#define DZ_MINIMAL_CONTROLLER__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define DZ_MINIMAL_CONTROLLER_EXPORT __attribute__ ((dllexport))
    #define DZ_MINIMAL_CONTROLLER_IMPORT __attribute__ ((dllimport))
  #else
    #define DZ_MINIMAL_CONTROLLER_EXPORT __declspec(dllexport)
    #define DZ_MINIMAL_CONTROLLER_IMPORT __declspec(dllimport)
  #endif
  #ifdef DZ_MINIMAL_CONTROLLER_BUILDING_LIBRARY
    #define DZ_MINIMAL_CONTROLLER_PUBLIC DZ_MINIMAL_CONTROLLER_EXPORT
  #else
    #define DZ_MINIMAL_CONTROLLER_PUBLIC DZ_MINIMAL_CONTROLLER_IMPORT
  #endif
  #define DZ_MINIMAL_CONTROLLER_PUBLIC_TYPE DZ_MINIMAL_CONTROLLER_PUBLIC
  #define DZ_MINIMAL_CONTROLLER_LOCAL
#else
  #define DZ_MINIMAL_CONTROLLER_EXPORT __attribute__ ((visibility("default")))
  #define DZ_MINIMAL_CONTROLLER_IMPORT
  #if __GNUC__ >= 4
    #define DZ_MINIMAL_CONTROLLER_PUBLIC __attribute__ ((visibility("default")))
    #define DZ_MINIMAL_CONTROLLER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define DZ_MINIMAL_CONTROLLER_PUBLIC
    #define DZ_MINIMAL_CONTROLLER_LOCAL
  #endif
  #define DZ_MINIMAL_CONTROLLER_PUBLIC_TYPE
#endif

#endif  // DZ_MINIMAL_CONTROLLER__VISIBILITY_CONTROL_H_
