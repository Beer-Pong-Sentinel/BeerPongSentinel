# Additional clean files
cmake_minimum_required(VERSION 3.16)

if("${CONFIG}" STREQUAL "" OR "${CONFIG}" STREQUAL "Debug")
  file(REMOVE_RECURSE
  "BeerPongSentinel_autogen"
  "CMakeFiles\\BeerPongSentinel_autogen.dir\\AutogenUsed.txt"
  "CMakeFiles\\BeerPongSentinel_autogen.dir\\ParseCache.txt"
  )
endif()
