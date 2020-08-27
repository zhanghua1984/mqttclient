set(SUBDIRS "emqx" "onenet" "baidu" "ali" "wljt")

foreach(subdir ${SUBDIRS})
    add_subdirectory(${subdir})
endforeach()
