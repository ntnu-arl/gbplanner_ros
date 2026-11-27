set(TEST_TYPE "PERFORMANCE")

set(tests
  scene_factory.cc
)

link_directories(${PROJECT_BINARY_DIR}/test)

ign_build_tests(TYPE PERFORMANCE SOURCES ${tests})
