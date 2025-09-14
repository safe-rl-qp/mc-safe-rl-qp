if(MC_RTC_DEVELOPER_MODE)
  set(MC_RTC_VERSION "${PROJECT_VERSION}")
else()
  find_program(GIT git)
  if(IS_DIRECTORY ${PROJECT_SOURCE_DIR}/.git AND GIT)
    execute_process(
      COMMAND ${GIT} rev-parse --short HEAD
      OUTPUT_VARIABLE GIT_REVISION
      WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
      OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    execute_process(
      COMMAND ${GIT} diff-index --name-only HEAD ":(exclude,top)debian/*"
      RESULT_VARIABLE GIT_DIFF_INDEX_RESULT
      OUTPUT_VARIABLE GIT_DIFF_INDEX_OUTPUT
      ERROR_VARIABLE GIT_DIFF_INDEX_ERROR
      OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    if(GIT_DIFF_INDEX_RESULT OR GIT_DIFF_INDEX_OUTPUT)
      set(GIT_REVISION "${GIT_REVISION}-dirty")
    endif()
    set(MC_RTC_VERSION "${PROJECT_VERSION}-${GIT_REVISION}")
  else()
    set(MC_RTC_VERSION "${PROJECT_VERSION}")
  endif()
endif()
set(MC_RTC_VERSION_MAJOR "${PROJECT_VERSION_MAJOR}")

set(FILE_IN "${PROJECT_SOURCE_DIR}/include/mc_rtc/version.h.in")
set(FILE_OUT "${PROJECT_BINARY_DIR}/include/mc_rtc/version.h")

configure_file("${FILE_IN}" "${FILE_OUT}")
