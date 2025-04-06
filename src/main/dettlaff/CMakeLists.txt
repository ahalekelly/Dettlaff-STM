# Define the library for the dettlaff module
add_library(dettlaff STATIC dettlaff.c)

# Link the dettlaff library against necessary Betaflight libraries
target_link_libraries(dettlaff PRIVATE common fc pg platform)

# Optional: Add unit tests if needed later
# add_executable(dettlaff_unittest dettlaff.c)
# target_link_libraries(dettlaff_unittest PRIVATE common fc pg unittest)
# register_unittest(dettlaff_unittest)