SET(DGTAL_TESTS_GMP_SRC
   python_exposure.cpp
)

FOREACH(FILE ${DGTAL_TESTS_SRC})
  add_executable(${FILE} ${FILE})
  target_link_libraries (${FILE} DGtalIO DGtal ${DGtalLibDependencies})
ENDFOREACH(FILE)

IF(GMP_FOUND)
  FOREACH(FILE ${DGTAL_TESTS_GMP_SRC})
    add_executable(${FILE} ${FILE})
    target_link_libraries (${FILE} DGtalIO DGtal ${DGtalLibDependencies})
  ENDFOREACH(FILE)
ENDIF(GMP_FOUND)

