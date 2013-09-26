#----------------------------------------------------------------
# Generated CMake target import file for configuration "Debug".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "DGtal" for configuration "Debug"
set_property(TARGET DGtal APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(DGtal PROPERTIES
  IMPORTED_LINK_INTERFACE_LIBRARIES_DEBUG "/usr/local/lib/libhdf5_hl.dylib;/usr/local/lib/libhdf5.dylib;/usr/local/lib/libsz.dylib;/usr/lib/libz.dylib;/usr/lib/libdl.dylib;/usr/lib/libm.dylib"
  IMPORTED_LOCATION_DEBUG "/usr/local/lib/libDGtal.dylib"
  IMPORTED_NO_SONAME_DEBUG "TRUE"
  )

list(APPEND _IMPORT_CHECK_TARGETS DGtal )
list(APPEND _IMPORT_CHECK_FILES_FOR_DGtal "/usr/local/lib/libDGtal.dylib" )

# Import target "DGtalIO" for configuration "Debug"
set_property(TARGET DGtalIO APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(DGtalIO PROPERTIES
  IMPORTED_LINK_INTERFACE_LIBRARIES_DEBUG "DGtal;/usr/local/lib/libhdf5_hl.dylib;/usr/local/lib/libhdf5.dylib;/usr/local/lib/libsz.dylib;/usr/lib/libz.dylib;/usr/lib/libdl.dylib;/usr/lib/libm.dylib"
  IMPORTED_LOCATION_DEBUG "/usr/local/lib/libDGtalIO.dylib"
  IMPORTED_NO_SONAME_DEBUG "TRUE"
  )

list(APPEND _IMPORT_CHECK_TARGETS DGtalIO )
list(APPEND _IMPORT_CHECK_FILES_FOR_DGtalIO "/usr/local/lib/libDGtalIO.dylib" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
