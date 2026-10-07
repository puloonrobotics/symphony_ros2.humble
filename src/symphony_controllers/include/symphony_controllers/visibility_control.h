#ifndef SYMPHONY_CONTROLLERS__VISIBILITY_CONTROL_H_
#define SYMPHONY_CONTROLLERS__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define SYMPHONY_CONTROLLERS_EXPORT __attribute__((dllexport))
#define SYMPHONY_CONTROLLERS_IMPORT __attribute__((dllimport))
#else
#define SYMPHONY_CONTROLLERS_EXPORT __declspec(dllexport)
#define SYMPHONY_CONTROLLERS_IMPORT __declspec(dllimport)
#endif
#ifdef SYMPHONY_CONTROLLERS_BUILDING_LIBRARY
#define SYMPHONY_CONTROLLERS_PUBLIC SYMPHONY_CONTROLLERS_EXPORT
#else
#define SYMPHONY_CONTROLLERS_PUBLIC SYMPHONY_CONTROLLERS_IMPORT
#endif
#define SYMPHONY_CONTROLLERS_PUBLIC_TYPE SYMPHONY_CONTROLLERS_PUBLIC
#define SYMPHONY_CONTROLLERS_LOCAL
#else
#define SYMPHONY_CONTROLLERS_EXPORT __attribute__((visibility("default")))
#define SYMPHONY_CONTROLLERS_IMPORT
#if __GNUC__ >= 4
#define SYMPHONY_CONTROLLERS_PUBLIC __attribute__((visibility("default")))
#define SYMPHONY_CONTROLLERS_LOCAL __attribute__((visibility("hidden")))
#else
#define SYMPHONY_CONTROLLERS_PUBLIC
#define SYMPHONY_CONTROLLERS_LOCAL
#endif
#define SYMPHONY_CONTROLLERS_PUBLIC_TYPE
#endif

#endif  // SYMPHONY_CONTROLLERS__VISIBILITY_CONTROL_H_
