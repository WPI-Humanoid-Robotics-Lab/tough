#ifndef DRCSIM_GAZEBO_COMPAT_HH
#define DRCSIM_GAZEBO_COMPAT_HH

#include <gazebo/gazebo_config.h>

#ifndef GAZEBO_DRCSIM_USING_DYNAMIC_POINTER_CAST
# if GAZEBO_MAJOR_VERSION >= 7
#define GAZEBO_DRCSIM_USING_DYNAMIC_POINTER_CAST using std::dynamic_pointer_cast
# else
#define GAZEBO_DRCSIM_USING_DYNAMIC_POINTER_CAST using boost::dynamic_pointer_cast
# endif
#endif

#endif
