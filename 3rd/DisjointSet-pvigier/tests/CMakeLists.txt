# Executables

add_executable(modulo modulo.cpp)
target_link_libraries(modulo PRIVATE disjoint_set)

# Tests

foreach(n RANGE 1 30)
    foreach(m RANGE 1 30)
        add_test(NAME "modulo${n}_${m}" COMMAND modulo ${n} ${m})
    endforeach()
endforeach()