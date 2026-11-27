# GLOB all the source files
file(GLOB source_files "*.cc")
list(SORT source_files)

set(tmp_sources)

foreach(source_file ${source_files})
  get_filename_component(source ${source_file} NAME)
  list(APPEND tmp_sources base/${source})
endforeach()

set (sources ${sources}
  ${tmp_sources}
  PARENT_SCOPE
)

add_subdirectory(media)

