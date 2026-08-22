# External Dependencies via FetchContent
# Modeled after REACH cmake/Dependencies.cmake pattern
#
# Currently empty — libfreenect2 has no git-sourced dependencies.
# Reserve this file for future FetchContent declarations.

include(FetchContent)

set(FETCHCONTENT_UPDATES_DISCONNECTED ON CACHE BOOL "Skip FetchContent update checks")

# Example (uncomment when needed):
# FetchContent_Declare(somelib
#   GIT_REPOSITORY https://github.com/...
#   GIT_TAG v1.0.0
# )
# FetchContent_MakeAvailable(somelib)
